# trail_SO101

**LeKiwi 移動ベースに SO-101 アームを載せた実機ロボット**を、ROS 2 Jazzy と
lerobot で動かすリポジトリです。SLAM で地図を作り、Nav2 で走り、
`map` 上の点へアームを伸ばします。

このドキュメントは**上から順にやれば動く手順書**です。深い話は別ファイルに送ります。

## 目次

1. [リポジトリを取得する](#1-リポジトリを取得する)
2. [Docker イメージをビルドする](#2-docker-イメージをビルドする)（初回のみ・時間がかかります）
3. [ワークスペースを初期化する](#3-ワークスペースを初期化する)（初回とパッケージ追加時）
4. [起動する](#4-起動する)（★ 安全上の注意）
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
git clone git@github.com:<あなたのGitHubユーザー名>/trail_SO101.git
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

以下のコマンドで確認します。

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

## 4. 起動する

> ## ★ サーボのトルクの切り替えについて
>
> robot.launchの起動時にトルクON、終了時にトルクOFFとなります。
> 
> トルクがOFFになるとアームは姿勢を保てず落ちることに注意してください。

```bash
cd docker/robot
make run BACKEND=lerobot ROBOT_ID=my_follower
```

### ★ コンテナは `bash` を起動するだけ

`make run` は 2 段階です。**コンテナ側では何も動きません。launch は
その中で人が叩きます。**

```bash
docker compose up -d                    # ① コンテナが上がる（bash が待つだけ）

docker compose exec -it robot /entrypoint.sh \
  ros2 launch lekiwi_so101_bringup robot.launch.py \
    backend:=lerobot robot_id:=my_follower    # ② launch を前面で走らせる
```

`compose.yaml` の `command:` は `["bash"]` で、launch は書いていません。理由は 2 つです。

1. **`docker compose up -d` でロボットが動き出さないようにするため。**
   launch を `command:` に書くと、**コンテナを上げた瞬間にトルクが入り**、
   ノードが全部走り出します。いつ動かすかは人が決めるべきです。
2. **起動のたびに引数を変えるため。** `backend` / `robot_id` / `sim` /
   `use_saved_map` は毎回違います。`command:` に書くと compose の編集が要ります。

> ## ★★ 代償: `make down` だけでは止まりません
>
> `docker compose down` が SIGTERM を送るのは**コンテナの PID 1 だけ**です。
> `exec` で起動したプロセス（＝ launch）には**届きません**。
> launch は SIGKILL され、**トルクが入ったまま残ります**（アームは凍り、
> ホイールは最後の指令速度で回り続けます）。
>
> ```
> docker compose down で SIGTERM が届くか（実測）
>   PID 1（compose の command:）  → 届く
>   exec したプロセス（launch）   → 届かない。SIGKILL される
> ```
>
> **必ず launch の端末で `Ctrl+C` してから `make down` してください。**
> 順番を逆にしてしまったときの復帰は `make release`（後述）。

引数を変えたいときは ② を直接叩いてください（`make run` が渡すのは
`backend` / `robot_id` / `start_rviz` と 3 つのポートだけです）。
`sim` や `use_saved_map` などは ② で指定します。

`ROBOT_ID` は LeRobot の較正 ID です。実物はここで確認できます。

```bash
ls ~/.cache/huggingface/lerobot/calibration/robots/so_follower/
```

RViz が上がります。別ターミナルで健全性を確認してください。

```bash
cd docker/robot
make check
```

`/robot_description` = **1**、`/joint_states` = **2**、コントローラ 3 つが `active`、
`/navigate_to_pose` と `follow_joint_trajectory` が**両方**見えれば正常です。

> ★ 実機が無い環境（Mac など）では `make mock` で同じ launch を
> `sim:=true backend:=mock` で起動できます。シリアルも USB も開きません。
> ROS グラフ・TF・リーチのソルバ・SLAM/Nav2 はそのまま検証できます。

### 停止手順（★ 順番を守ること）

```bash
# ① 別ターミナル
cd docker/robot && make stow      # アームを低く畳む

# ② 人がアームを支える

# ③ make run の端末で Ctrl+C     ← ここでトルクが切れる

# ④ アームが静止してから手を放す

# ⑤ コンテナを片付ける
make down
```

### 異常終了したとき（launch が落ちた / SIGKILL / OOM）

停止処理が走らなかった場合、**サーボは指令を保持したままです。**
**★ コンテナを落とす必要はありません。** 止まっている必要があるのは launch だけです。

```bash
cd docker/robot
make release-check    # 読むだけ。いまトルクが入っているか確認
make release          # ★ アームもホイールもこれ 1 つで解放（★ アームが落ちます）
make release-wheels   # ホイールだけ止める（アームは落ちない）
```

> ★ launch がまだ生きている場合は、**どのプロセスがポートを掴んでいるかを
> 名指しして中止します**。その場合は先に launch を `Ctrl+C` してください。

詳細は [`docker/robot/README.md`](docker/robot/README.md)。

---

## 5. RViz で動かす

**いちばん簡単な入口です。** `make run` で RViz が一緒に上がります
（設定は `lekiwi_so101_bringup/rviz/reach.rviz`）。

> ★ **Fixed Frame は `map` にしてください。** ツールは Fixed Frame の座標で
> publish するので、`odom` のままだとリーチが `REJECTED_WRONG_FRAME` で弾かれます。
> 既定は `map` です。

### ツール（上部のツールバー）

| ツール | 出すもの | 何が起きるか |
| --- | --- | --- |
| **Publish Point** | `/clicked_point` | **クリックした点へアームを伸ばす**。シングルクリックで発火 |
| **2D Goal Pose** | `/goal_pose` | **その姿勢へ走る**（Nav2） |
| **2D Pose Estimate** | `/initialpose` | AMCL の初期姿勢。★ **保存地図構成（`use_saved_map:=true`）のときだけ**意味があります |

> ★ **"Publish Point" と "2D Goal Pose" は別物です。** 前者は `PointStamped`、
> 後者は `PoseStamped` で、型も宛先も違います。アームを動かすのは前者です。

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
>
> ★ 点群は Fixed Frame が `map` なので**自動的に `map` 上の正しい位置に出ます**。
> カメラは URDF で `arm_gripper_link` に剛体固定されており、外部キャリブレーションは
> 要りません。

> ## ★ リーチは別途起動が必要です
>
> `robot.launch.py` は**ロボットを動かせる状態にするところまで**で、リーチは
> 起動しません。"Publish Point" を使う前に、別ターミナルで:
>
> ```bash
> cd docker/robot && make shell
> ros2 launch lekiwi_examples reach.launch.py
> ```

### リーチの結果を見る

RViz には目標球しか出ません。**理由まで知りたいときは端末で流してください。**

```bash
docker compose -f compose.yaml exec robot /entrypoint.sh \
  ros2 topic echo /so101/reach_status
```

```
ACCEPTED   target=map(0.350,0.050,0.250) ... residual=0.0042
SUCCEEDED  residual_fk=0.0042
REJECTED_OUT_OF_RANGE range=1.963m > max_reach_radius=0.55m
```

状態コードの一覧は [`docs/interfaces.md`](docs/interfaces.md#リーチの状態メッセージ)。

---

## 6. サブシステム別の Topic / Service / Action

**よく使うものだけ**を挙げます。全部の一覧と CLI テストコマンドは
**[`docs/interfaces.md`](docs/interfaces.md)** にあります。

以降 `$E` は次の前置きです（`docker exec` は ENTRYPOINT を通らないので必須）。

```bash
E="docker compose -f docker/robot/compose.yaml exec robot /entrypoint.sh"
```

### 6-1. アーム（SO-101）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/clicked_point` | Topic `PointStamped` | **リーチ目標**。RViz の "Publish Point" と同じ |
| `/so101/reach_target` | Topic `PoseStamped` | リーチ目標（`frame_id: map` 必須） |
| `/so101/reach_status` | Topic `String` | 判定結果 1 行。**まずこれを流しておく** |
| `/so101/reach_markers` | Topic `Marker` | 目標球（緑 = 受理 / 赤 = 棄却） |
| `/joint_trajectory_controller/follow_joint_trajectory` | **Action** | 関節を直接動かす（5 関節） |
| `/gripper_controller/gripper_cmd` | **Action** | グリッパ |
| `/so101/stow` | **Service** | **アームを畳む。停止前に必ず** |
| `/joint_states` | Topic `JointState` | 関節角。★ publisher は 2 つ（車輪 / アーム） |

```bash
# map 上の点へ伸ばす
$E ros2 topic pub --once -w 1 /so101/reach_target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 0.35, y: 0.05, z: 0.25}, orientation: {w: 1.0}}}'

# 畳む（★ 停止前に必ず）
$E ros2 service call /so101/stow std_srvs/srv/Trigger '{}'
```

> ★ **リーチのノードは別途起動が必要です**（`ros2 launch lekiwi_examples reach.launch.py`）。
> `/clicked_point` と `/so101/reach_target` はそのノードが購読します。
>
> ★ **届かない目標は「警告して何もしない」**のが仕様です。ベースは動きません。
> 到達不能かどうかは**指令を出す前にオフラインで判定**しています。

### 6-2. ベース（走行・ナビゲーション）

| 名前 | 種別 | 用途 |
| --- | --- | --- |
| `/goal_pose` | Topic `PoseStamped` | ナビ目標。RViz の "2D Goal Pose" と同じ |
| `/navigate_to_pose` | **Action** | ナビの本筋。**結果とフィードバックが得られる** |
| `/compute_path_to_pose` | **Action** | 経路計画だけ（走らない）。到達可能かの確認 |
| `/cmd_vel` | Topic `Twist` | 速度指令。★ **安全機構より下流**（下記） |
| `/odom` | Topic `Odometry` | 自己位置。★ **指令値の積分**で実測ではない |
| `/map` | Topic `OccupancyGrid` | SLAM が作った地図 |
| `/plan` `/optimal_trajectory` | Topic `Path` | 大域経路 / 局所軌道（MPPI） |
| `/lekiwi_base_driver/recover` | **Service** | 過負荷ラッチ解除 + 速度モード再設定 |

```bash
# ナビ目標（アクション。結果が返る）
$E ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}'

# ★ 車輪を浮かせてから。2 秒流す（watchdog が 0.5 秒なので --once では止まる）
$E ros2 topic pub -r 10 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

> ★ **`/cmd_vel` を手打ちすると Nav2 の安全機構を全部飛ばします。**
> 本来は `controller_server → /cmd_vel_nav → velocity_smoother →
> /cmd_vel_smoothed → collision_monitor → /cmd_vel` の 3 段構えで、
> 加速度制限も衝突監視もその途中にあります。**床に降ろした状態で使わないこと。**

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
> ただし**取付姿勢の既定値は未実測**なので、絶対位置はまだ信用できません
> （[`docs/wrist_camera.md`](docs/wrist_camera.md)）。
>
> ★ **カメラが動くので、点群を使う側は TF を「メッセージのタイムスタンプ」で
> 引くこと。** 最新 TF で解決すると腕の動作中に点群がずれます。

---

## 7. 自分でノードを書く

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

ノードの書き方、QoS の罠、テストの書き方、つまずきポイント集は
**[`docs/development.md`](docs/development.md)** にまとめてあります。

**動くサンプルは [`ros2_ws/src/lekiwi_examples/`](ros2_ws/src/lekiwi_examples/)**
にあります（リーチ / 逆運動学 / キーボード操作）。自分のプログラムもここに
置いてください。

```bash
# コンテナ内。ロボットが起動している状態で
ros2 launch lekiwi_examples reach.launch.py      # map 上の点へリーチ
ros2 run lekiwi_examples teleop_keyboard         # ベース + アームをキーボードで
```

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
│   ├── lekiwi_examples/        ★ ロボットの上で動くもの。リーチ、IK、キーボード操作
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
| 1 | **この README** | 起動までの手順 |
| 2 | [`docs/interfaces.md`](docs/interfaces.md) | **Topic / Service / Action の一覧と CLI テスト** |
| 3 | [`docs/internals.md`](docs/internals.md) | **内部処理の仕組み。** 指令がどこを通るか |
| 4 | [`docs/development.md`](docs/development.md) | ノードの書き方、つまずきポイント集 |
| 5 | [`docker/robot/README.md`](docker/robot/README.md) | 停止・非常停止・異常終了からの復帰 |

必要になったときに読むもの:

| ドキュメント | 内容 |
| --- | --- |
| [`docs/tf_reliability.md`](docs/tf_reliability.md) | **TF のどこが信用できないか。** 精度で悩んだら |
| [`docs/lekiwi_so101_reach.md`](docs/lekiwi_so101_reach.md) | リーチの設計と精度（数 cm ずれる理由） |
| [`docs/wrist_camera.md`](docs/wrist_camera.md) | 手首カメラの取り付けと較正 |
| [`docs/lerobot_examples.md`](docs/lerobot_examples.md) | ROS 2 を使わない lerobot 直叩き |
| [`docs/hardware_agent.md`](docs/hardware_agent.md) | 実機を触る担当者への指示 |

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
