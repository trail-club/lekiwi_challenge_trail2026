"""Feetech STS3215 サーボバスの最小ラッパ。

lerobot の ``FeetechMotorsBus`` が内部で行っている処理のうち、速度制御に必要な
分だけを ``scservo_sdk`` 直叩きで再現したもの。lerobot 本体は torch を引き込む
ため ROS 2 の Docker イメージには入れられない。

参照元の実機検証済みコードは ``lerobot_examples/lekiwi_base_keyboard.py``。

────────────────────────────────────────────────────────────────────────
必ず守ること (間違えるとサーボが暴走するか通信が不安定になる)
────────────────────────────────────────────────────────────────────────
1. ``Goal_Velocity`` は **2 の補数ではなく sign-magnitude (符号ビット 15)**。
   -500 は 0x81F4 であって 0xFE0C ではない。
2. ``scs.PacketHandler(n)`` は **プロセスグローバル** ``SCS_END`` を書き換える。
   init で 1 度だけ呼ぶこと。同一プロセスで別プロトコルの機器を扱わないこと。
3. PyPI 版 ``scservo_sdk`` の ``setPacketTimeout`` は計算式が壊れている。
   lerobot と同じモンキーパッチを当てないと散発的に RX タイムアウトする。
4. ``Operating_Mode`` (addr 33) は EEPROM 領域。書く前に ``Lock`` (addr 55) を
   0 にして解錠する必要がある。

単体でも動くので、実機の切り分けに使える::

    python3 -m lekiwi_base_bringup.sts_bus --port /dev/lekiwi --ping
    python3 -m lekiwi_base_bringup.sts_bus --port /dev/lekiwi --diagnostics
"""

from __future__ import annotations

import time

import scservo_sdk as scs

# ── レジスタ (アドレス, バイト数) ───────────────────────────────────────
OPERATING_MODE = (33, 1)  # EEPROM. 0=position, 1=velocity, 2=pwm, 3=step
TORQUE_ENABLE = (40, 1)
ACCELERATION = (41, 1)
GOAL_VELOCITY = (46, 2)  # sign-magnitude, 符号ビット 15
LOCK = (55, 1)  # EEPROM ロック. 0=解錠, 1=施錠
PRESENT_POSITION = (56, 2)  # sign-magnitude, 符号ビット 15
PRESENT_VELOCITY = (58, 2)  # sign-magnitude, 符号ビット 15
PRESENT_LOAD = (60, 2)  # sign-magnitude, 符号ビット 10 (15 ではない)
PRESENT_VOLTAGE = (62, 1)  # 単位 0.1V
PRESENT_TEMPERATURE = (63, 1)  # 単位 °C

VELOCITY_MODE = 1
STS3215_MODEL_NUMBER = 777

SIGN_BIT_VELOCITY = 15
SIGN_BIT_LOAD = 10


class StsBusError(RuntimeError):
    """サーボバスの通信・応答エラー。"""


def encode_sign_magnitude(value: int, sign_bit: int = SIGN_BIT_VELOCITY) -> int:
    """符号付き整数を sign-magnitude 表現へ。

    ``encode_sign_magnitude(-500)`` -> ``0x81F4``
    """
    max_magnitude = (1 << sign_bit) - 1
    magnitude = abs(int(value))
    if magnitude > max_magnitude:
        raise ValueError(f"magnitude {magnitude} exceeds {max_magnitude} for sign_bit={sign_bit}")
    return ((1 if value < 0 else 0) << sign_bit) | magnitude


def decode_sign_magnitude(raw: int, sign_bit: int = SIGN_BIT_VELOCITY) -> int:
    """sign-magnitude 表現を符号付き整数へ。"""
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if (raw >> sign_bit) & 1 else magnitude


def _patched_set_packet_timeout(self, packet_length):
    """lerobot の ``patch_setPacketTimeout`` と同じ。

    素の SDK は ``LATENCY_TIMER * 2.0 + 2.0`` を足すが、この値は Feetech には
    小さすぎて散発的に RX タイムアウトを起こす。
    """
    self.packet_start_time = self.getCurrentTime()
    self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + 50


class StsBus:
    """STS3215 を速度モードで回すための最小バス。

    Args:
        port: シリアルポート (例 ``/dev/lekiwi``)
        ids: モータ ID のリスト。順序は呼び出し側の規約に従う (LeKiwi は 7,8,9)
        baudrate: 既定 1 Mbps (lerobot と同じ)
        protocol_end: STS/SMS 系は 0 (リトルエンディアン)。0 以外は未サポート
    """

    def __init__(
        self,
        port: str,
        ids: list[int],
        baudrate: int = 1_000_000,
        protocol_end: int = 0,
        num_retry: int = 5,
    ) -> None:
        if protocol_end != 0:
            raise ValueError("STS/SMS 系は protocol_end=0 のみ。SCS_LOBYTE の意味が変わる")
        self.port_name = port
        self.ids = list(ids)
        self.baudrate = baudrate
        self.protocol_end = protocol_end
        self.num_retry = num_retry

        self._port = None
        self._packet = None
        self._sync_writer = None

    # ── 接続 ───────────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._port is not None

    def connect(self) -> None:
        port = scs.PortHandler(self.port_name)
        # ★ 素の SDK のタイムアウト式は壊れているので差し替える
        port.setPacketTimeout = _patched_set_packet_timeout.__get__(port, scs.PortHandler)

        # ★ これはプロセスグローバル SCS_END を書き換える。1 度だけ。
        packet = scs.PacketHandler(self.protocol_end)

        if not port.openPort():
            raise StsBusError(f"ポートを開けない: {self.port_name}")
        if not port.setBaudRate(self.baudrate):
            port.closePort()
            raise StsBusError(f"ボーレートを設定できない: {self.baudrate}")

        self._port = port
        self._packet = packet
        self._sync_writer = scs.GroupSyncWrite(port, packet, GOAL_VELOCITY[0], GOAL_VELOCITY[1])

    def close(self) -> None:
        if self._port is not None:
            self._port.closePort()
        self._port = None
        self._packet = None
        self._sync_writer = None

    def _assert_connected(self) -> None:
        if self._port is None:
            raise StsBusError("未接続。先に connect() を呼ぶこと")

    # ── 低レベル read/write ────────────────────────────────────────────

    def _write(self, reg: tuple[int, int], motor_id: int, value: int) -> None:
        self._assert_connected()
        addr, size = reg
        writer = self._packet.write1ByteTxRx if size == 1 else self._packet.write2ByteTxRx

        last = None
        for _ in range(1 + self.num_retry):
            comm, err = writer(self._port, motor_id, addr, int(value))
            if comm == scs.COMM_SUCCESS and err == 0:
                return
            last = (comm, err)
        comm, err = last
        raise StsBusError(
            f"ID {motor_id} の addr {addr} への書き込み失敗: "
            f"{self._packet.getTxRxResult(comm)} / {self._packet.getRxPacketError(err)}"
        )

    def _read(self, reg: tuple[int, int], motor_id: int) -> int:
        self._assert_connected()
        addr, size = reg
        reader = self._packet.read1ByteTxRx if size == 1 else self._packet.read2ByteTxRx

        last = None
        for _ in range(1 + self.num_retry):
            value, comm, err = reader(self._port, motor_id, addr)
            if comm == scs.COMM_SUCCESS and err == 0:
                return value
            last = (comm, err)
        comm, err = last
        raise StsBusError(
            f"ID {motor_id} の addr {addr} からの読み出し失敗: "
            f"{self._packet.getTxRxResult(comm)} / {self._packet.getRxPacketError(err)}"
        )

    # ── 高レベル操作 ───────────────────────────────────────────────────

    def ping_all(self) -> dict[int, int]:
        """全 ID を ping し ``{id: model_number}`` を返す。STS3215 は 777。

        応答しない ID は値 0 で返す (例外にしない)。起動時の診断用。
        """
        self._assert_connected()
        result = {}
        for motor_id in self.ids:
            model, comm, _err = self._packet.ping(self._port, motor_id)
            result[motor_id] = model if comm == scs.COMM_SUCCESS else 0
        return result

    def configure_velocity_mode(self, acceleration: int | None = None) -> None:
        """全モータを速度モードにしてトルクを入れる。

        順序が重要: Torque_Enable=0 → Lock=0 (解錠) → Operating_Mode=1
                    → Torque_Enable=1 → Lock=1 (施錠)
        ``Operating_Mode`` は EEPROM 領域なので解錠が先に必要。
        """
        for motor_id in self.ids:
            self._write(TORQUE_ENABLE, motor_id, 0)
            self._write(LOCK, motor_id, 0)

        if acceleration is not None:
            # addr 41 は SRAM なのでロック状態に関係なく書けるが、解錠中に済ませる
            for motor_id in self.ids:
                self._write(ACCELERATION, motor_id, int(acceleration))

        for motor_id in self.ids:
            self._write(OPERATING_MODE, motor_id, VELOCITY_MODE)

        for motor_id in self.ids:
            self._write(TORQUE_ENABLE, motor_id, 1)
            self._write(LOCK, motor_id, 1)

    def sync_write_velocity(self, ticks: dict[int, int]) -> None:
        """全モータへ速度指令をまとめて送る (ブロードキャスト、応答なし)。"""
        self._assert_connected()
        self._sync_writer.clearParam()
        for motor_id, value in ticks.items():
            raw = encode_sign_magnitude(value, SIGN_BIT_VELOCITY)
            # SCS_LOBYTE/HIBYTE は SCS_END を見る。protocol_end=0 を強制済み
            self._sync_writer.addParam(motor_id, [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])

        comm = self._sync_writer.txPacket()
        if comm != scs.COMM_SUCCESS:
            raise StsBusError(f"速度指令の送信失敗: {self._packet.getTxRxResult(comm)}")

    def stop(self) -> None:
        """全モータの速度をゼロにする。失敗しても例外を投げない。"""
        try:
            self.sync_write_velocity(dict.fromkeys(self.ids, 0))
        except StsBusError:
            pass

    def disable_torque(self) -> None:
        """全モータのトルクを切る。失敗しても他の ID の処理は続ける。

        ``Torque_Enable=0`` の書き込みは過負荷エラーのラッチ解除も兼ねる。
        """
        for motor_id in self.ids:
            try:
                self._write(TORQUE_ENABLE, motor_id, 0)
            except StsBusError:
                pass

    def read_torque_enable(self) -> dict[int, int | None]:
        """各 ID の ``Torque_Enable`` を読む。読めなかった ID は ``None``。

        ``disable_torque()`` は ID ごとの失敗を握り潰すので、「呼べた」ことは
        「切れた」ことを意味しない。本当に切れたかはこれで確かめる。
        ``lekiwi_so101_bringup.release_all`` が結果表示に使う。
        """
        result: dict[int, int | None] = {}
        for motor_id in self.ids:
            try:
                result[motor_id] = self._read(TORQUE_ENABLE, motor_id)
            except StsBusError:
                result[motor_id] = None
        return result

    def recover(self, acceleration: int | None = None) -> None:
        """過負荷ラッチを解除して速度モードを再設定する。"""
        self.stop()
        self.disable_torque()
        time.sleep(0.1)
        self.configure_velocity_mode(acceleration)

    def read_diagnostics(self) -> dict[int, dict[str, float]]:
        """電圧・温度・負荷を読む。低頻度でのみ呼ぶこと (往復通信が発生する)。"""
        out = {}
        for motor_id in self.ids:
            try:
                out[motor_id] = {
                    "voltage": self._read(PRESENT_VOLTAGE, motor_id) / 10.0,
                    "temperature": float(self._read(PRESENT_TEMPERATURE, motor_id)),
                    "load": decode_sign_magnitude(self._read(PRESENT_LOAD, motor_id), SIGN_BIT_LOAD),
                }
            except StsBusError as exc:
                out[motor_id] = {"error": str(exc)}
        return out

    def read_velocities(self) -> dict[int, int]:
        """実測速度 [ticks/s] を読む。診断用。制御ループでは使わない。"""
        return {
            motor_id: decode_sign_magnitude(self._read(PRESENT_VELOCITY, motor_id), SIGN_BIT_VELOCITY)
            for motor_id in self.ids
        }


def main() -> None:
    """実機切り分け用の簡易 CLI。"""
    import argparse

    parser = argparse.ArgumentParser(description="STS3215 バスの疎通確認")
    parser.add_argument("--port", default="/dev/lekiwi")
    parser.add_argument("--ids", default="7,8,9")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ping", action="store_true", help="全 ID を ping する")
    parser.add_argument("--diagnostics", action="store_true", help="電圧・温度・負荷を読む")
    parser.add_argument("--torque-off", action="store_true", help="トルクを切る (過負荷ラッチ解除)")
    args = parser.parse_args()

    ids = [int(x) for x in args.ids.split(",")]
    bus = StsBus(args.port, ids, baudrate=args.baudrate)
    bus.connect()
    try:
        if args.ping or not (args.diagnostics or args.torque_off):
            for motor_id, model in bus.ping_all().items():
                state = f"model {model}" if model else "応答なし"
                mark = "OK " if model == STS3215_MODEL_NUMBER else "!! "
                print(f"{mark}ID {motor_id}: {state}")
        if args.diagnostics:
            for motor_id, values in bus.read_diagnostics().items():
                print(f"ID {motor_id}: {values}")
        if args.torque_off:
            bus.stop()
            bus.disable_torque()
            print("トルクを切りました")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
