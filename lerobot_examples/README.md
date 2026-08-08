# lerobot の examples（ROS 2 を使わない直叩き）

**ROS 2 を経由せず、lerobot から SO-101 を直接動かすスクリプト**です。
ROS 2 での実機開発（[`../README.md`](../README.md)）とは**別系統**で、
Docker も使いません。

> ★ **LeKiwi の実機開発にこれは要りません。** アームだけを手早く触りたいとき、
> または leader/follower のテレオペを試したいときに使ってください。
>
> ★ ROS 2 側のアーム記述は `ros2_so_arm/so_arm101_description`（xacro）で、
> ここで使う `lerobot_examples/SO101/*.urdf` とは**別物**です。
> メッシュだけがバイト単位で同一です（詳細は
> [`../docs/internals.md`](../docs/internals.md) と `ros2_ws/so101_upstream.repos`）。

## セットアップ

```bash
# Mac（開発機）
brew install ffmpeg
cd lerobot_examples && uv sync

# Linux PC（実機を繋ぐほう）
sudo apt-get update && sudo apt-get install -y ffmpeg
cd lerobot_examples && uv sync
```

> ★ **`lerobot_examples/` の中で叩くこと。** venv は `lerobot_examples/.venv` に作られます。
> リポジトリ直下の `pyproject.toml` / `.venv` は**ROS 2 開発用**（コンテナの中で
> 使う Linux バイナリ）で、こちらとは別物です → [`../README.md`](../README.md)。

## 設定

`config.toml` に書きます（ポート / ID・補間・IK の重み・カメラインデックス）。

```bash
# ポートの調べ方
uv run python -m lerobot.scripts.lerobot_find_port

# カメラインデックスの調べ方
uv run python -m lerobot.scripts.lerobot_find_cameras
```

## スクリプト

| スクリプト | 内容 |
| --- | --- |
| `record_and_move.py` | leader の関節角度を記録し、follower を同じ角度へ移動 |
| `record_and_move_ik.py` | leader の EE（エンドエフェクタ）位置を記録し、IK で解いて follower を移動 |
| `capture_camera.py` | SO-101 付属カメラのライブ映像を表示し、`s` キーで画像を保存 |

```bash
# Linux PC（実機）
uv run python record_and_move.py
uv run python record_and_move_ik.py
uv run python capture_camera.py
```

## モデルファイル

`SO101/` に URDF と MuJoCo（MJCF）が入っています。

| ファイル | 用途 |
| --- | --- |
| `so101_new_calib.urdf` | **既定**。`record_and_move_ik.py` が placo に食わせる |
| `so101_old_calib.urdf` | 旧較正（ゼロ姿勢が「水平に伸び切った状態」） |
| `scene.xml` / `so101_*_calib.xml` | MuJoCo 用 |

**new / old の違いは関節のゼロ位置の取り方**です。

| | ゼロの定義 | shoulder_lift の可動域 |
| --- | --- | --- |
| new_calib（既定） | 可動域の**中央** | ±1.745 |
| old_calib | **水平に伸び切った姿勢** | −3.316 … +0.175 |

> ★ **ROS 2 側（`so_arm101_description`）は new_calib と同じ規約**です。
> 全関節ゼロでの `base_link → gripper_frame_link` を順運動学で計算すると
> 小数点以下 4 桁まで一致します（`(0.3914, 0.0000, 0.2265)`）。
> ただし実機では `so101_bringup/calibration_limits.py` が LeRobot の較正 JSON から
> URDF の可動域を書き換えるので、**真の基準はサーボの EEPROM** です。

## ★ IK（placo）の注意 — macOS

`record_and_move_ik.py` は逆運動学に [placo](https://github.com/Rhoban/placo) を
使います（`uv sync` で入ります）。macOS では placo がリンクする
`liburdfdom_*.4.0.dylib` と、依存解決で入る `liburdfdom_*.6.0.0.dylib` の
**soname 不一致**で、こうなります。

```
Library not loaded: @rpath/liburdfdom_sensor.4.0.dylib
```

暫定対処として 6.0 の実体へ 4.0 名のシンボリックリンクを張ります
（ABI 互換のため FK/IK は正常動作を確認済み）。

```bash
# Mac
cd "$(uv run python -c 'import cmeel, pathlib; print(pathlib.Path(cmeel.__file__).resolve().parent.parent/"cmeel.prefix"/"lib")')"
for n in sensor model world; do
  ln -sf "liburdfdom_${n}.6.0.0.dylib" "liburdfdom_${n}.4.0.dylib"
done
```

`uv sync` 等で cmeel-urdfdom が再インストールされた場合は、上記を再実行してください。

## ★ 較正（EEPROM を書き換える前に必ず控えを取る）

較正値の実体は**サーボの EEPROM** で、lerobot の JSON はその控えにすぎません。
**書き込みは永続的です。**

```bash
# Linux PC（実機）。lerobot_examples/ の中で
cp -a ~/.cache/huggingface/lerobot/calibration/robots/so_follower \
      ~/so_follower_backup_$(date +%Y%m%d_%H%M)

uv run lerobot-calibrate --robot.type=so101_follower \
  --robot.port=/dev/so101_follower --robot.id=my_follower
```

★ **較正は ROS を止めた状態で行ってください。** ROS 側からは変更できませんし、
Feetech のバスはマスタが 1 つしか居られません。

初回の手順は [`../README.md`](../README.md) の 4 章にまとめてあります。
