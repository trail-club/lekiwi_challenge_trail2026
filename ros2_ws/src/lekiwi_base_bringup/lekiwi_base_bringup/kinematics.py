"""LeKiwi 3輪オムニベースの運動学。

``lerobot_examples/lekiwi_base_keyboard.py`` (実機検証済み) からの移植で、計算内容は
lerobot の ``LeKiwi._body_to_wheel_raw`` と同一。

``.dockerignore`` が ``lerobot_examples/`` を除外しているためコンテナ内から import
できない。意図的な複製である。

行の順序は全体を通して **left(ID 7), back(ID 8), right(ID 9)** で統一する。

座標系の規約は ``lekiwi_description/urdf/lekiwi_base.macro.xacro`` の冒頭コメ
ントを参照。base_link 上での車輪の方位角は left=60°, back=180°, right=300°
で、ここの ``wheel_angles_deg`` + ``angle_offset_deg`` と同じ規約から導かれて
いる。**片方だけ変えないこと。**
"""

from __future__ import annotations

import numpy as np

#: STS3215 の 1 回転あたりのエンコーダステップ数
TICKS_PER_REV = 4096.0

#: 生の速度値 (ticks/s) と deg/s の換算係数
STEPS_PER_DEG = TICKS_PER_REV / 360.0

#: Goal_Velocity の sign-magnitude 表現で表せる最大の大きさ (符号ビットは 15)
MAX_TICKS = 32767


def wheel_matrix(
    base_radius: float,
    angle_offset_deg: float = -90.0,
    wheel_angles_deg: tuple[float, float, float] = (240.0, 0.0, 120.0),
) -> np.ndarray:
    """車体速度 → 各車輪の周速度 [m/s] を与える 3x3 行列を作る。

    各行は ``[cos(a), sin(a), base_radius]`` で、``a`` は車輪の取付角。
    ``angle_offset_deg`` が実機合わせで唯一いじるノブ (Phase D2)。
    """
    angles = np.radians(np.asarray(wheel_angles_deg, dtype=float) + angle_offset_deg)
    return np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])


def degps_to_ticks(degps: float) -> int:
    """deg/s を生の速度値へ。

    sign-magnitude 表現の上限は ±32767 なので、そこで飽和させる
    (lerobot は -32768 まで許しているが、その値は encode 時に例外になる)。
    """
    ticks = int(round(degps * STEPS_PER_DEG))
    return max(-MAX_TICKS, min(MAX_TICKS, ticks))


def body_to_wheel_ticks(
    vx: float,
    vy: float,
    wz_degps: float,
    m: np.ndarray,
    wheel_radius: float,
    max_ticks: int = 3000,
    direction_signs: np.ndarray | None = None,
) -> list[int]:
    """車体速度 (m/s, m/s, deg/s) → 各車輪の生の速度指令 [ticks/s]。

    ``max_ticks`` を超える場合は **3輪すべてを比例縮小** する。1輪だけ飽和させる
    と合成速度の向きが変わってしまうため、この処理は必須。
    """
    velocity = np.array([vx, vy, np.radians(wz_degps)])
    wheel_degps = (m.dot(velocity) / wheel_radius) * (180.0 / np.pi)

    magnitudes = np.abs(wheel_degps) * STEPS_PER_DEG
    peak = float(magnitudes.max())
    if peak > max_ticks:
        wheel_degps = wheel_degps * (max_ticks / peak)

    if direction_signs is not None:
        wheel_degps = wheel_degps * direction_signs

    return [degps_to_ticks(d) for d in wheel_degps]


def wheel_ticks_to_body(
    ticks,
    m_inv: np.ndarray,
    wheel_radius: float,
    direction_signs: np.ndarray | None = None,
) -> tuple[float, float, float]:
    """各車輪の生の速度値 → 車体速度 (m/s, m/s, deg/s)。

    実際に送った tick 値をそのまま渡すことで、飽和による縮小と整数丸めを含んだ
    **本当の指令値** が得られる。オープンループのオドメトリはこれを積分する。
    """
    wheel_degps = np.asarray(ticks, dtype=float) / STEPS_PER_DEG
    if direction_signs is not None:
        wheel_degps = wheel_degps * direction_signs

    wheel_linear = np.radians(wheel_degps) * wheel_radius
    vx, vy, wz_rad = m_inv.dot(wheel_linear)
    return float(vx), float(vy), float(np.degrees(wz_rad))


def ticks_to_radps(ticks: float, direction_sign: float = 1.0) -> float:
    """生の速度値 → 車輪の角速度 [rad/s] (``/joint_states`` 用)。"""
    return np.radians(ticks / STEPS_PER_DEG) * direction_sign
