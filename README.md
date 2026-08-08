# trail_SO101

**LeKiwi 移動ベースに SO-101 アームを載せた実機ロボット**を、ROS 2 Jazzy と
lerobot で動かすリポジトリです。

## 目次

1. [リポジトリを取得する](#1-リポジトリを取得する)
2. [Docker イメージをビルドする](#2-docker-イメージをビルドする)（初回のみ・時間がかかります）
3. [ワークスペースを初期化する](#3-ワークスペースを初期化する)（初回とパッケージ追加時）
4. [起動方法](#4-起動方法)（★ 安全上の注意）
5. [RViz で動かす](#5-rviz-で動かす)
6. [サブシステム別の Topic / Service / Action](#6-サブシステム別の-topic--service--action)
7. [自分でノードを書く](#7-自分でノードを書く)

- [この機体の構成](#この機体の構成)
- [リポジトリ構成](#リポジトリ構成)
- [ドキュメント一覧（読む順番）](#ドキュメント一覧読む順番)

---

## 1. リポジトリを取得する

リポジトリをForkし、cloneする。
```bash
git clone git@github.com:<ForkしたGitHubユーザー名>/trail_SO101.git
cd trail_SO101
```

---

## 2. Docker イメージをビルドする

**初回のみ。20〜40 分かかります**（約 7.9GB）。

```bash
cd docker/robot
cp .env.example .env
make build
```

**★ `.env` はここで実機に合わせて編集してください（後回しにしない）。**

以下のコマンドでデバイスを確認します。

```bash
getent group dialout        # 出力の3番目の数字が DIALOUT_GID（Ubuntu なら 20）
ls -l /dev/lekiwi /dev/so101_follower /dev/rplidar    # 3つとも見えること
```

3つとも見えて `DIALOUT_GID` が 20 なら、**`.env` は編集不要**です。

### udev ルールを入れる（初回のみ）

デバイスが見えない場合はルールが入っていません。

```bash
sudo cp docker/lekiwi_base_ros2/99-lekiwi.rules /etc/udev/rules.d/
sudo cp docker/so101_ros2/99-so101.rules        /etc/udev/rules.d/
sudo cp docker/rplidar_ros2/99-rplidar.rules    /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

ルールは `SYMLINK+=` で `/dev/<名前>` を作り、`GROUP:="dialout"` を付けます。

| デバイス | 識別のしかた | 現在の値 |
| --- | --- | --- |
| `/dev/lekiwi` | `ATTRS{serial}` | `5A7A017874` |
| `/dev/so101_follower` | `ATTRS{serial}` | `5A7A018080` |
| `/dev/rplidar` | `idVendor:idProduct` | `10c4:ea60`（CP210x） |

> ★ **アームとベースはシリアル番号で識別しています。** 両方 WaveShare の同じ
> 設計で **VID:PID が同一（`1a86:55d3`）**のため、VID:PID で書くと
> `/dev/lekiwi` と `/dev/so101_follower` が**どちらも「最後に認識された方」の
> 同じ基板を指し**、12V のホイール指令が 7.4V のアームサーボへ飛びます。

**基板を交換したらシリアルが変わる**ので、ルールの `ATTRS{serial}` を書き換えます。

```bash
udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT
```

---

## 3. ワークスペースを初期化する

```bash
cd docker/robot
make bootstrap
```

これが 2 つのことをやります。

1. **Python 環境を作る** — `uv sync` がリポジトリ直下に `.venv` を作ります
   （★ 初回だけ約 1.7GB。torch を含むので時間がかかります）
2. **ワークスペースを建てる** — 上流の配置 → `colcon build` → 静的検査

> ★ **`.venv` はホスト側に残ります。** コンテナを作り直してもイメージを
> 作り直しても消えないので、2 回目以降の `make bootstrap` は一瞬で終わります。
>
> ★ Python の依存はリポジトリ直下の `pyproject.toml` に足して、**コンテナ内で
> `cd /app && uv sync` するだけ**です（`uv.lock` も一緒に更新されます）。
> **`make build` も `colcon build` も要りません。**

---

## 4. 起動方法

> ## ★ サーボのトルクの切り替えについて
>
> robot.launchの起動時にトルクON、終了時にトルクOFFとなります。
> 
> トルクがOFFになるとアームは姿勢を保てず落ちることに注意してください。

よく使うコマンドはMakefileにまとめられています。

```bash
make up # コンテナが立ち上がる
make shell # コンテナの中のシェルへ移動
```

コンテナの中で
```bash
ros2 launch lekiwi_so101_bringup robot.launch.py \
    backend:=lerobot robot_id:=my_follower
```

`robot_id` は LeRobot の較正 ID です。実物はここで確認できます。

```bash
ls ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
```

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
make release
```

---

## 5. RViz で動かす

### ツール（上部のツールバー）

| ツール | 出すもの | 何が起きるか |
| --- | --- | --- |
| **2D Goal Pose** | `/goal_pose` | **その姿勢へ走る**（Nav2） |
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
> ゼロなら点群の生成自体をスキップするので、切っている間はコストがゼロです。
> 見たいときだけ ON にしてください。

## 6. サブシステム別の Topic / Service / Action

**よく使うものだけ**を挙げます。全部を見るなら `ros2 topic list -t` /
`ros2 action list -t` / `ros2 service list -t` が確実です。

以降 `$E` は次の前置きです（`docker exec` は ENTRYPOINT を通らないので必須）。

```bash
E="docker compose -f docker/robot/compose.yaml exec robot /entrypoint.sh"
```

### 6-1. アーム（SO-101）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/joint_trajectory_controller/follow_joint_trajectory` | **Action** | 関節を直接動かす（5 関節） |
| `/gripper_controller/gripper_cmd` | **Action** | グリッパ |
| `/so101/stow` | **Service** | **アームを畳む。停止前に必ず** |
| `/joint_states` | Topic `JointState` | 関節角。★ publisher は 2 つ（車輪 / アーム） |
| `/so101/lerobot_bridge/shutdown` | **Service** | トルク OFF して終了 |

```bash
# ★ 🔴 実際に動きます。人が立ち会うこと
$E ros2 action send_goal -f /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [arm_shoulder_pan_joint, arm_shoulder_lift_joint,
     arm_elbow_flex_joint, arm_wrist_flex_joint, arm_wrist_roll_joint],
    points: [{positions: [0.0, 0.0, 0.5, 0.5, 0.0], time_from_start: {sec: 3}}]}}'

# 畳む（★ 停止前に必ず）
$E ros2 service call /so101/stow std_srvs/srv/Trigger '{}'
```

> ★ `map` 上の点へアームを伸ばす「リーチ」は `lekiwi_examples` にあります。
> `robot.launch.py` には含まれません → [`docs/examples.md`](docs/examples.md)

### 6-2. ベース（走行・ナビゲーション）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/goal_pose` | Topic `PoseStamped` | ナビ目標。RViz の "2D Goal Pose" と同じ |
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
$E ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}'

# ★ 車輪を浮かせてから。2 秒流す（watchdog が 0.5 秒なので --once では止まる）
$E ros2 topic pub -r 10 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'

$E ros2 lifecycle get /bt_navigator     # active でなければ経路計画も走りません
```

> ★ **`/cmd_vel` を手打ちすると Nav2 の安全機構を全部飛ばします。**
> 本来は `controller_server → /cmd_vel_nav → velocity_smoother →
> /cmd_vel_smoothed → collision_monitor → /cmd_vel` の 3 段構えで、
> 加速度制限も衝突監視もその途中にあります。

### 6-3. LiDAR（RPLIDAR A1）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/scan` | Topic `LaserScan` | 生スキャン（360°） |
| `/scan_filtered` | Topic `LaserScan` | **前方 ±60° に絞ったもの** |

```bash
$E ros2 topic hz /scan
$E ros2 run tf2_ros tf2_echo base_link laser_link   # 実測 (0.10, 0, 0.03) yaw −7°
```

> ★ **SLAM も costmap も `/scan_filtered` を見ています**（生の `/scan` ではありません）。
> 360° のままだと自分の後輪とボディが障害物として地図に焼き付くためです。
>
> ★ `scan_filter` が起動していないと `/scan_filtered` の publisher が 0 になり、
> **`map → odom` が永遠に出ません。** Nav2 は `Invalid frame ID map` を
> INFO で吐き続けるのでエラーに見えません。

### 6-4. RealSense（手首カメラ D435i）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/wrist_camera/wrist_camera/depth/color/points` | Topic `PointCloud2` | 点群。★ **BEST_EFFORT** |
| `/wrist_camera/wrist_camera/color/image_raw` | Topic `Image` | カラー画像 |

```bash
$E ros2 topic hz /wrist_camera/wrist_camera/depth/color/points
$E ros2 run tf2_ros tf2_echo map wrist_camera_depth_optical_frame
```

> ★ **購読するときは QoS を `BEST_EFFORT` にしてください。** 既定は RELIABLE で、
> そのままだと**1 通も届きません**（エラーも出ません）。
>
> ★ **点群を `map` 上に置くのに較正は要りません。** カメラは URDF で
> `arm_gripper_link` に剛体固定されているので TF から出ます。
> ただし**取付姿勢の既定値は未実測**（D405 + 公式ホルダ前提の幾何計算値）
> なので、絶対位置はまだ信用できません。
>
> ★ **カメラが動くので、点群を使う側は TF を「メッセージのタイムスタンプ」で
> 引くこと。** 最新 TF で解決すると腕の動作中に点群がずれます。

### 6-5. 動いているかを確かめる

```bash
make check
```

`/robot_description` = **1**、`/joint_states` = **2**、コントローラ 3 つが `active`、
`/navigate_to_pose` と `follow_joint_trajectory` が**両方**見えれば正常です。

```bash
$E ros2 run tf2_ros tf2_echo map base_footprint              # 自己位置
$E ros2 run tf2_ros tf2_echo base_link arm_gripper_frame_link
#   ★ 全関節ゼロなら (0.471, 0.000, 0.283) になるはず
```

> ★ **`/robot_description` の publisher は 1 つでなければなりません。**
> 2 つ以上あると RViz に別のロボットが重なって出ます。
>
> ★ `ros2 topic list` は **discovery 待ちで最初は少なく出ます**。
> 足りないように見えても数秒待ってから読み直してください。

---

## 7. 自分でノードを書く

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

### 編集したあと何をすればよいか

| 変えたもの | やること |
| --- | --- |
| Python / YAML / launch / URDF | **何もしなくていい。** launch を上げ直すだけ（`--symlink-install`） |
| ファイルを**追加**した | `colcon build --symlink-install --packages-select <pkg>` |
| パッケージを追加した | `make bootstrap` |
| `pyproject.toml`（Python の依存） | コンテナ内で `cd /app && uv sync` |
| `Dockerfile`（apt パッケージ） | `make build` してから `make bootstrap` |

### つまずきポイント

| 症状 | 原因と対処 |
| --- | --- |
| `ros2: command not found` | `docker exec` は ENTRYPOINT を通りません。`/entrypoint.sh` を前置する |
| `Package '...' not found` | `make bootstrap` を打っていません（`install/` は `.gitignore` 済み） |
| ビルドしたのに反映されない | `source install/setup.bash` を打ち直す |
| `ros2 topic echo` に何も出ない | QoS 不一致。センサ系は `qos_profile_sensor_data`、CLI なら `--qos-reliability best_effort` |
| **画像を扱うノードが Segmentation fault** | **★ `cv_bridge` を使わないこと**（下記） |
| アームが `unconfigured` のまま | 実機の姿勢が URDF の可動域の外です。手で範囲内へ戻す |
| `colcon build` が `can't copy '...'` | `--symlink-install` の壊れたリンク。`make bootstrap` が検出して直す |
| RViz に見覚えのないロボットが出る | `ROS_DOMAIN_ID` の衝突。`docker ps` で他スタックが動いていないか見る |

> ★ **`cv_bridge` は使わないこと。** 画像は `image_saver.py` の `imgmsg_to_np()`
> のように自前で numpy へ整形します（やっているのは `bytes` の reshape だけです）。
>
> `import` も `CvBridge()` の生成も成功し、**`imgmsg_to_cv2()` を呼んだ瞬間に
> SIGSEGV** します。`cv_bridge` の C 拡張は apt の numpy 1.26 向けで、
> このコンテナは lerobot の要求で numpy 2 系を使うためです。
> `cv_bridge/__init__.py` がその `ImportError` を握り潰すので、import では気付けません。

---

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
| **アーム** | Feetech STS3215 × 6（ID 1–6）、**7.4V** | `/dev/so101_follower` |
| **ベース** | Feetech STS3215 × 3（ID 7/8/9）、**12V**、3輪オムニ | `/dev/lekiwi` |
| **LiDAR** | RPLIDAR A1M8 | `/dev/rplidar` |
| **RealSense** | D435i（アームの手首に載せる） | USB |

> ★ **アームとホイールを同じシリアルバスに繋がないこと。**
> どちらも STS3215 の 1 Mbps で ID も分かれているため**物理的には繋がってしまい**、
> 繋いだ瞬間に 12V が 7.4V のアームサーボに掛かって壊れます。

> ★ **実機を繋げるのは Linux だけです。** macOS の Docker はシリアル / USB
> デバイスをコンテナへ渡せません。Mac では `make mock`（実機に触れない構成）
> までしか実行できません。

---

## リポジトリ構成

```
trail_SO101/
├── docker/
│   ├── robot/                  ★ 統合スタック。ふだん使うのはこれだけ
│   │   ├── Dockerfile          1 イメージ（ROS + Nav2 + LeRobot + RealSense）
│   │   ├── compose.yaml        1 コンテナ（実機）
│   │   ├── compose.mock.yaml   1 コンテナ（実機に触れない）
│   │   ├── bootstrap.sh        上流の配置 + colcon build + 静的検査
│   │   └── Makefile            build / bootstrap / run / mock / check / release
│   ├── so101_ros2/             以下は 1 サブシステムだけ切り分けたいとき用
│   ├── lekiwi_base_ros2/
│   ├── rplidar_ros2/
│   └── realsense_ros2/
├── ros2_ws/src/
│   ├── lekiwi_examples/        ロボットの上で動くもの。リーチ、逆運動学、キーボード操作
│   ├── so101_bringup/          アーム。LeRobot ブリッジ、較正（ハードウェアに触る側）
│   ├── lekiwi_base_bringup/    ベース。ドライバ、オドメトリ、スキャン処理
│   ├── lekiwi_so101_bringup/   合成のみ。結合 URDF、robot.launch.py、release_all
│   ├── lekiwi_description/     ベースの URDF
│   ├── rplidar_bringup/        LiDAR
│   └── realsense_bringup/      カメラ
├── examples/                   ROS 2 を使わない lerobot 直叩き（+ SO101 モデル）
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
> | `examples/pyproject.toml` | lerobot 直叩き | `examples/.venv` | ホスト（Mac / Linux） |
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
| 5 | [`examples/README.md`](examples/README.md) | ROS 2 を使わない lerobot 直叩き |

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
