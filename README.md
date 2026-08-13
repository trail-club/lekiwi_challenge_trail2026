# trail_SO101

**LeKiwi 移動ベースに SO-101 アームを載せた実機ロボット**を、ROS 2 Jazzy と
lerobot で動かすリポジトリです。

モーターバスは次の2構成を明示的に選択します。自動判定はしません。

| モード | アーム | ベース | 起動コマンド |
| --- | --- | --- | --- |
| `split` | 7.4V、ID 1〜6、`/dev/so101_follower` | 12V、ID 7〜9、`/dev/lekiwi` | `make run-split SO101_ROBOT_ID=my_follower` |
| `shared` | 12V、ID 1〜6 | 12V、ID 7〜9、両方とも `/dev/lekiwi` | `make run-shared LEKIWI_ROBOT_ID=my_lekiwi` |

sharedの車輪IDは `7=left, 8=back, 9=right` 固定です。split機の7.4Vアームを
12Vのベースバスへ接続してはいけません。

## 目次

1. [リポジトリを取得する](#1-リポジトリを取得する)
2. [Docker イメージをビルドする](#2-docker-イメージをビルドする)（初回のみ・時間がかかります）
3. [udev ルールを入れる](#3-udev-ルールを入れる-linux-のみ初回のみ)（★ Linux のみ・初回のみ）
4. [アームを較正する](#4-アームを較正する-linux-のみ初回のみ)（★ Linux のみ・初回のみ）
5. [ワークスペースを初期化する](#5-ワークスペースを初期化する)（初回とパッケージ追加時）
6. [起動方法](#6-起動方法)（★ 安全上の注意）
7. [RViz で動かす](#7-rviz-で動かす)
8. [サブシステム別の Topic / Service / Action](#8-サブシステム別の-topic--service--action)
9. [自分でノードを書く](#9-自分でノードを書く)

- [この機体の構成](#この機体の構成)
- [リポジトリ構成](#リポジトリ構成)
- [ドキュメント一覧（読む順番）](#ドキュメント一覧読む順番)

---

## 1. リポジトリを取得する

リポジトリをForkし、cloneする。
```bash
git clone git@github.com:<ForkしたGitHubユーザー名>/lekiwi_challenge_trail2026.git
cd lekiwi_challenge_trail2026
```

---

## 2. Docker イメージをビルドする

**初回のみ。20〜40 分かかります**（約 7.9GB）。

```bash
make build
```

ビルドを待つ間に、3 章と 4 章を済ませておけます。

---

## 3. udev ルールを入れる（★ Linux のみ・初回のみ）

シリアル 3 本に固定名（`/dev/lekiwi` `/dev/so101_follower` `/dev/rplidar`）を
付けます。ルールは `SYMLINK+=` で `/dev/<名前>` を作り、`GROUP:="dialout"` を
付けます。**入れないとコンテナがデバイスを掴めません。**

> ★ **アームとベースはシリアル番号で識別します。** 両方 WaveShare の同じ設計で
> **VID:PID が同一（`1a86:55d3`）**のため、VID:PID で書くと `/dev/lekiwi` と
> `/dev/so101_follower` が**どちらも「最後に認識された方」の同じ基板を指し**、
> 12V のホイール指令が 7.4V のアームサーボへ飛びます。

**① 自分の基板のシリアル番号を調べる**

★ **1 本ずつ挿してください。** 2 本同時に挿すと VID:PID が同じなので、
どちらのシリアルがどちらの基板か区別できません。

```bash
for d in /dev/ttyACM*; do
  echo "$d  $(udevadm info -q property -n "$d" | grep -m1 ID_SERIAL_SHORT)"
done
```

**② `.env` に①の値を書く**

追跡対象の `.rules` はテンプレートです。機体固有値で直接編集しません。

```bash
cp docker/robot/.env.example docker/robot/.env  # 初回だけ
```

```dotenv
# docker/robot/.env
LEKIWI_SERIAL=<sharedバスまたはベース基板のID_SERIAL_SHORT>
SO101_SERIAL=<split機のアーム基板のID_SERIAL_SHORT>
```

shared機では `LEKIWI_SERIAL` だけが必須です。split機では両方必要です。
RPLIDARは `10c4:ea60` で識別するためシリアル設定はありません。

**基板を交換したらシリアルが変わります。**その都度①からやり直してください。

**③ 入れて反映する**

```bash
# リポジトリ直下で
make udev-dry-run BUS_MODE=shared  # 生成内容を見るだけ
make install-udev BUS_MODE=shared  # shared機

# split機の場合
make install-udev BUS_MODE=split
```

過去のCompose起動で `/dev/lekiwi` 等が空ディレクトリになっている場合、
インストーラは安全のため停止します。表示された対象だけ `sudo rmdir` して再実行します。

**④ 必要なデバイスが見えることを確認する**

```bash
ls -l /dev/lekiwi /dev/rplidar
ls -l /dev/so101_follower  # split機だけ必要
```

### `.env` を用意する

```bash
cp docker/robot/.env.example docker/robot/.env
getent group dialout        # 出力の 3 番目の数字が DIALOUT_GID（Ubuntu なら 20）
```

USBシリアルを含む `.env` はGit管理外です。別機体ではその機体の `.env` を作ります。

`DIALOUT_GID` が 20 で、④ のデバイスが見えているなら**編集不要**です。
違うときだけ `.env` を書き換えてください。

---

## 4. アームを較正する（★ Linux のみ・初回のみ）

較正しないとアームは正しい角度で動きません。**ROS 2 とは別系統**で、
`lerobot_examples/` の lerobot から実行します（Docker は使いません）。

```bash
sudo apt-get install -y ffmpeg       # lerobot が要求する（Mac は brew install ffmpeg）

# リポジトリ直下で
cd lerobot_examples
uv sync                              # ★ 初回のみ。lerobot を入れる
uv run lerobot-find-port             # 使用するモータードライバが出るか確認

# split機（7.4Vアーム、6モーター）
uv run lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/so101_follower \
  --robot.id=my_follower

# shared機（全モータ12V、ID 1〜9）
uv run lerobot-calibrate \
  --robot.type=lekiwi \
  --robot.port=/dev/lekiwi \
  --robot.id=my_lekiwi
```

画面の指示に従って各関節を可動域の端まで動かします。終わるとここに JSON が
できます。**`--robot.id` に与えた名前がファイル名**になり、起動時の
`robot_id`（6 章）になります。

```bash
ls ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
#    -> my_follower.json
ls ~/.cache/huggingface/lerobot/calibration/robots/lekiwi/
#    -> my_lekiwi.json
```

shared機は9モーターを接続して新規較正します。split用の6モーターJSONを変換・流用
しません。

---

## 5. ワークスペースを初期化する

```bash
make bootstrap
```

bootstrapがuv syncとcolcon build --symlink-installをします。
これが 2 つのことをやります。

1. **Python 環境を作る** — `uv sync` がリポジトリ直下に `.venv` を作ります
   （★ 初回だけ約 1.7GB。torch を含むので時間がかかります）
2. **ワークスペースを建てる** — 上流の配置 → `colcon build` → 静的検査

> ★ Python の依存変更時はリポジトリ直下の `pyproject.toml` に足して、コンテナ内で
> `cd /app && uv sync` してください。

---

## 6. 起動方法

> ## ★ サーボのトルクの切り替えについて
>
> robot.launchの起動時にトルクON、終了時にトルクOFFとなります。
> 
> トルクがOFFになるとアームは姿勢を保てず落ちることに注意してください。
>
> 最初からトルクを入れずに起動するには `arm_torque:=false`（後述）。

よく使うコマンドはMakefileにまとめられています。

```bash
make run-split SO101_ROBOT_ID=my_follower
make run-shared LEKIWI_ROBOT_ID=my_lekiwi
```

どちらもコンテナを起動した後、`motor_bus_mode` と4章の較正IDを渡して
`robot.launch.py` を前面実行します。コンテナだけ起動する場合は
`make up-split` / `make up-shared`、シェルへ入る場合は `make shell` です。

### 手でアームを動かして角度を読む

`arm_torque:=false` で起動すると、**トルクを入れず指令も書きません。**
アームを手で動かして `/joint_states` で角度を読めます。

```bash
ros2 launch lekiwi_so101_bringup robot.launch.py \
    backend:=lerobot robot_id:=my_follower arm_torque:=false
```

```bash
# 別ターミナル（コンテナの中）
ros2 topic echo /joint_states
```

> ★ **読み出しはトルクに関係なく動く**ので `/joint_states` も TF も出ます。
> ベース・LiDAR・カメラは通常どおり動きます。
>
> アームを動かすには `arm_torque:=true`（既定）で起動し直してください。

### 終了方法

launchをctrl + cで終了。（トルクが落ちることに注意）

コンテナを閉じる場合は

```bash
make down
```

### 異常終了したとき（launch が落ちた / SIGKILL / OOM）

以下のコマンドでサーボのトルクを落とします。

robot.launchが実行中だと失敗することに注意してください。

```bash
make release BUS_MODE=split   # split機
make release BUS_MODE=shared  # shared機
```

---

## 7. RViz で動かす

### ツール（上部のツールバー）

| ツール | 出すもの | 何が起きるか |
| --- | --- | --- |
| **2D Goal Pose** | `/goal_pose` | **その姿勢へ走る**（Nav2、または `lekiwi_examples` の `goal_drive`） |
| **2D Pose Estimate** | `/initialpose` | AMCL の初期姿勢。★ **保存地図構成（`use_saved_map:=true`）のときだけ**意味があります |

### 表示（Displays パネル）

| 表示名 | トピック | 既定 |
| --- | --- | --- |
| RobotModel | `/robot_description` | ON |
| TF | — | ON |
| Map | `/map` | ON |
| Scan (filtered) | `/scan_filtered` | ON |
| Global Plan | `/plan` | ON |
| Optimal Trajectory (MPPI) | `/optimal_trajectory` | ON |
| Footprint | `/local_costmap/published_footprint` | ON |
| Reach Target | `/so101/reach_markers` | ON（**緑 = 受理 / 赤 = 棄却**） |
| Global / Local Costmap | `/*_costmap/costmap` | **OFF**（必要なときだけ ON） |
| Wrist Camera Cloud | `/wrist_camera/wrist_camera/depth/color/points` | **OFF** |
| Wrist Camera（軸） | `wrist_camera_link` | **OFF** |

> ★ **Wrist Camera Cloud は既定 OFF です。** `realsense2_camera` は購読者が
> ゼロなら点群の生成自体をスキップします。
> 見たいときだけ ON にしてください。

## 8. サブシステム別の Topic / Service / Action

**よく使うものだけ**を挙げます。全部を見るなら `ros2 topic list -t` /
`ros2 action list -t` / `ros2 service list -t` が確実です。

### 8-1. アーム（SO-101）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/joint_trajectory_controller/follow_joint_trajectory` | **Action** | 関節を直接動かす（5 関節） |
| `/parallel_gripper_action_controller/GripperActionController` | **Action** | グリッパ |
| `/joint_states` | Topic `JointState` | 関節角。★ publisher は 2 つ（車輪 / アーム） |
| `/so101/lerobot_bridge/shutdown` | **Service** | トルク OFF して終了 |

```bash
# ★ 🔴 実際に動きます。人が立ち会うこと
ros2 action send_goal -f /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [arm_shoulder_pan_joint, arm_shoulder_lift_joint,
     arm_elbow_flex_joint, arm_wrist_flex_joint, arm_wrist_roll_joint],
    points: [{positions: [0.0, 0.0, 0.5, 0.5, 0.0], time_from_start: {sec: 3}}]}}'
```

> ★ `map` 上の点へアームを伸ばす「リーチ」が `lekiwi_examples` にあります。

### 8-2. ベース（走行・ナビゲーション）
が厳しい気がする
| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/goal_pose` | Topic `PoseStamped` | ナビ目標。RViz の "2D Goal Pose"。Nav2 なしなら `ros2 run lekiwi_examples goal_drive` |
| `/navigate_to_pose` | **Action** | ナビの本筋。**結果とフィードバックが得られる** |
| `/compute_path_to_pose` | **Action** | 経路計画だけ（走らない）。到達可能かの確認 |
| `/cmd_vel` | Topic `Twist` | 速度指令。★ **安全機構より下流**（下記） |
| `/odom` | Topic `Odometry` | 自己位置。★ **指令値の積分**で実測ではない |
| `/map` | Topic `OccupancyGrid` | SLAM が作った地図 |
| `/plan` `/optimal_trajectory` | Topic `Path` | 大域経路 / 局所軌道。★ 他構成でよくある `/local_plan` は**存在しません** |
| `/lekiwi_base_driver/recover` | **Service** | 過負荷ラッチ解除 + 速度モード再設定 |
| `/global_costmap/clear_entirely_global_costmap` | **Service** | コストマップに幻の障害物が焼き付いたとき |

```bash
# ナビ目標（アクション。結果が返る）
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}'

# ★ 2 秒前進する（watchdog が 0.5 秒なので --once では止まる）
ros2 topic pub -r 10 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'

ros2 lifecycle get /bt_navigator     # active でなければ経路計画も走りません
```

### 8-3. LiDAR（RPLIDAR A1）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/scan` | Topic `LaserScan` | 生スキャン（360°） |
| `/scan_filtered` | Topic `LaserScan` | **前方 ±60° に絞ったもの** |

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link laser_link   # 実測 (0.10, 0, 0.03) yaw −7°
```

> ロボット本体を障害物として認識させないためにフィルタをかけたものが/scan_filteredです。

### 8-4. RealSense（手首カメラ D435i）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/wrist_camera/wrist_camera/depth/color/points` | Topic `PointCloud2` | 点群。★ **BEST_EFFORT** |
| `/wrist_camera/wrist_camera/color/image_raw` | Topic `Image` | カラー画像 |

```bash
ros2 topic hz /wrist_camera/wrist_camera/depth/color/points
ros2 run tf2_ros tf2_echo map wrist_camera_depth_optical_frame
```

> ★ **購読するときは QoS を `BEST_EFFORT` にしてください。** 既定は RELIABLE で、
> そのままだと**1 通も届きません**（エラーも出ません）。
>
> ★ 取付姿勢の設定が適当なので大きな誤差が残っています。
>
> ★ **カメラが動くので、点群を使う側は TF を「メッセージのタイムスタンプ」で
> 引くこと。** 最新 TF で解決すると腕の動作中に点群がずれます。

### 8-5. 動いているかを確かめる

```bash
make check
```

`/robot_description` = **1**、`/joint_states` = **2**、コントローラ 3 つが `active`、
`/navigate_to_pose` と `follow_joint_trajectory` が**両方**見えれば正常です。

```bash
ros2 run tf2_ros tf2_echo map base_footprint              # 自己位置
ros2 run tf2_ros tf2_echo base_link arm_gripper_frame_link
#   ★ 全関節ゼロなら (0.471, 0.000, 0.283) になるはず
```

> ★ **`/robot_description` の publisher は 1 つでなければなりません。**
> 2 つ以上あると RViz に別のロボットが重なって出ます。
>
> ★ `ros2 topic list` は **discovery 待ちで最初は少なく出ます**。
> 足りないように見えても数秒待ってから読み直してください。

---

## 9. 自分でノードを書く

**まず動く例を読んでください。** すべて `lekiwi_examples` にあります
→ [`docs/examples.md`](docs/examples.md)

| 例 | 何を示しているか |
| --- | --- |
| **`example_sequence`** | **★ 最小構成。** アクションの呼び方、`Ctrl+C` の畳み方 |
| `image_saver` | 画像を numpy にする（★ `cv_bridge` を使わない） |
| `reach_to_point` | TF、複数スレッドでの実行、動かす前の可否判定 |
| `teleop_keyboard` | 端末入力とタイマ |

### パッケージを作る

```bash
# コンテナ内 /ros2_ws/src/
ros2 pkg create --build-type ament_python --license Apache-2.0 \
  --node-name hello my_first_pkg
```

```bash
# コンテナ内 /ros2_ws/
colcon build --symlink-install --packages-select my_first_pkg
source install/setup.bash
ros2 run my_first_pkg hello
#    -> Hi from my_first_pkg.
```

**★ `make bootstrap` を打ち直せば、以降は自動でビルド対象に入ります。**

### 依存を足す

`package.xml` に書けるのは rosdep キーがあるものだけです。`lerobot` と
`feetech-servo-sdk`（import 名 `scservo_sdk`）にはありません。

**Python の依存はリポジトリ直下の `pyproject.toml` に足します。**

```bash
# コンテナ内
cd /app && uv sync
```

`uv.lock` も一緒に更新されるので、`pyproject.toml` と両方を commit してください。

> ★ **ホスト側で `uv sync` しないでください。** macOS には system の Python 3.12 が
> 無いので `No interpreter found for Python 3.12` で止まります。

### ノードを書く。

`<パッケージ名>/<パッケージ名>/`の下にpythonファイルを作成し、ノードの定義を書きましょう。

lekiwi_examples/example_sequence.py, lekiwi_examples/image_saver.pyを参考にしてください。

### つまずきポイント

| 症状 | 原因と対処 |
| --- | --- |
| `ros2: command not found` | `docker exec` は ENTRYPOINT を通りません。`/entrypoint.sh` を前置する |
| `Package '...' not found` | `make bootstrap` を打っていません（`install/` は `.gitignore` 済み） |
| ビルドしたのに反映されない | `source install/setup.bash` を打ち直す |
| `ros2 topic echo` に何も出ない | QoS 不一致。センサ系は `qos_profile_sensor_data`、CLI なら `--qos-reliability best_effort` |
| **画像を扱うノードが Segmentation fault** | **★ `cv_bridge` を使わないこと**。numpyのバージョンの関係で動かない。 |
| アームが `unconfigured` のまま | 実機の姿勢が URDF の可動域の外です。手で範囲内へ戻す |
| `colcon build` が `can't copy '...'` | `--symlink-install` の壊れたリンク。`make bootstrap` が検出して直す |
| RViz に見覚えのないロボットが出る | `ROS_DOMAIN_ID` の衝突。`docker ps` で他スタックが動いていないか見る |

---

## 調整すべきパラメータ

[lekiwi_so101.urdf.xacro](ros2_ws/src/lekiwi_so101_bringup/urdf/lekiwi_so101.urdf.xacro)の`wrist_camera_pitch`は実機に応じて調整が必要です。

rvizでWrist Camera Cloudの点群が実際の環境と一致するように調整してください。

## この機体の構成

```
        map (slam_toolbox が出す)
         └ odom (base_driver のオドメトリ積分)
            └ base_footprint → base_link
                               ├ laser_link      ← RPLIDAR A1
                               └ arm_mount_link
                                  └ arm_base_link … arm_gripper_link
                                       ├ arm_gripper_frame_link  ← リーチの手先
                                       └ wrist_camera_link       ← RealSense D435i
```

| サブシステム | ハードウェア | ポート |
| --- | --- | --- |
| **アーム** | Feetech STS3215 × 6（ID 1–6） | split: 7.4V `/dev/so101_follower` / shared: 12V `/dev/lekiwi` |
| **ベース** | Feetech STS3215 × 3（ID 7/8/9）、**12V**、3輪オムニ | `/dev/lekiwi` |
| **LiDAR** | RPLIDAR A1M8 | `/dev/rplidar` |
| **RealSense** | D435i（アームの手首に載せる） | USB |

> ★ split機では**アームとホイールを同じシリアルバスに繋がないこと。**
> どちらも STS3215 の 1 Mbps で ID も分かれているため**物理的には繋がってしまい**、
> 繋いだ瞬間に 12V が 7.4V のアームサーボに掛かって壊れます。

> ★ **実機を繋げるのは Linux だけです。** macOS の Docker はシリアル / USB
> デバイスをコンテナへ渡せません。Mac では `make mock-split` または
> `make mock-shared`（実機に触れない構成）
> までしか実行できません。

---

## リポジトリ構成

```
trail_SO101/
├── docker/
│   ├── robot/                  ★ 統合スタック。ふだん使うのはこれだけ
│   │   ├── Dockerfile          1 イメージ（ROS + Nav2 + LeRobot + RealSense）
│   │   ├── compose.yaml        共通設定
│   │   ├── compose.split.yaml  split機の2モーターポート
│   │   ├── compose.shared.yaml shared機の1モーターポート
│   │   ├── compose.mock.yaml   1 コンテナ（実機に触れない）
│   │   ├── bootstrap.sh        上流の配置 + colcon build + 静的検査
│   │   └── Makefile            build / bootstrap / run-* / mock-* / check / release
│   ├── so101_ros2/             以下は 1 サブシステムだけ切り分けたいとき用
│   ├── lekiwi_base_ros2/
│   ├── rplidar_ros2/
│   └── realsense_ros2/
├── ros2_ws/src/
│   ├── lekiwi_examples/        ロボットの上で動くもの。リーチ、逆運動学、キーボード操作
│   ├── so101_bringup/          アーム。LeRobot ブリッジ、較正（ハードウェアに触る側）
│   ├── lekiwi_base_bringup/    ベース。ドライバ、オドメトリ、スキャン処理
│   ├── lekiwi_hardware_interfaces/ sharedバス内部用WheelCommand
│   ├── lekiwi_so101_bringup/   合成のみ。結合 URDF、robot.launch.py、release_all
│   ├── lekiwi_description/     ベースの URDF
│   ├── rplidar_bringup/        LiDAR
│   └── realsense_bringup/      カメラ
├── lerobot_examples/           ROS 2 を使わない lerobot 直叩き（+ SO101 モデル）
│   ├── pyproject.toml          ★ そちら専用の uv。ホストで動かす
│   └── uv.lock
├── pyproject.toml              ★ ROS 2 開発用の uv。コンテナの中で使う
├── uv.lock
└── docs/                       ドキュメント
```

> ## ★ uv のプロジェクトは 2 つあります。共有していません
>
> | | 何のため | venv | どこで動く |
> | --- | --- | --- | --- |
> | `pyproject.toml`（直下） | **ROS 2 開発** | `.venv` | コンテナの中（`/app/.venv`） |
> | `lerobot_examples/pyproject.toml` | lerobot 直叩き | `lerobot_examples/.venv` | ホスト（Mac / Linux） |
>
> 中身のバイナリの OS が違うので、**そもそも共有できません**。
> ROS 2 の実機開発に必要なのは前者だけです。

> ★ `ros2_ws/src/ros2_so_arm` と `ros2_ws/src/sllidar_ros2` は**上流**で、
> `.gitignore` 済みです。`make bootstrap` がイメージから配置します。

---

## ドキュメント一覧（読む順番）

| # | ドキュメント | 内容 |
| --- | --- | --- |
| 1 | **この README** | 起動までの手順、Topic / Service / Action の一覧、ノードの書き方 |
| 2 | [`docs/examples.md`](docs/examples.md) | **動くサンプル。** ★ 最小構成、画像保存、リーチ、キーボード操作 |
| 3 | [`docs/internals.md`](docs/internals.md) | **内部処理の仕組み。** 指令がどこを通るか |
| 4 | [`docker/robot/README.md`](docker/robot/README.md) | 停止・非常停止・異常終了からの復帰 |
| 5 | [`lerobot_examples/README.md`](lerobot_examples/README.md) | ROS 2 を使わない lerobot 直叩き |

---

## Requirements

| 項目 | |
| --- | --- |
| ROS 2 | Jazzy（コンテナ内。ホストへの導入は不要） |
| Docker | 最新安定版。Mac は Docker Desktop |
| ディスク | イメージに約 8GB |
| 実機を繋ぐ側 | **Linux 必須**（macOS の Docker はシリアル/USB を渡せません） |

## 参考リンク

- [ROS 2 Jazzy 公式ドキュメント](https://docs.ros.org/en/jazzy/index.html)
- [Nav2 ドキュメント](https://docs.nav2.org/)
- [ros2_control ドキュメント](https://control.ros.org/jazzy/index.html)
- [LeRobot](https://github.com/huggingface/lerobot)
