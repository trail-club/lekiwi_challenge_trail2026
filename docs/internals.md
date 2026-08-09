# 内部処理の仕組み

「トピックに投げると、なぜアームが動くのか」を追える資料です。
使い方と名前の一覧は [`../README.md`](../README.md)。

**★ ここに書いてあることは、すべてコードを読むか実際に動かして確認しています。**
未確認のものは「未確認」と明記しています。

---

## 全体像

統合スタックは **1 コンテナ・1 launch・約 24 ノード**です。
`robot.launch.py` が既存の launch を include して組み立てています。

```
robot.launch.py
├─ arm.launch.py            ← アーム側。RSP と RViz は「システム全体で 1 つ」
│   ├─ robot_state_publisher  ← 結合 URDF（ベース + アーム + 手首カメラ）
│   ├─ follower.launch.py     ← lerobot_bridge → ros2_control → spawner ×3
│   └─ rviz2
├─ nav.launch.py              ← ベース側（sim:=true なら sim_nav.launch.py）
│   ├─ base_driver            ← /cmd_vel → サーボ
│   ├─ scan_filter            ← /scan → /scan_filtered
│   ├─ sllidar_node           ← 実機 LiDAR（sim なら fake_scan）
│   ├─ slam_toolbox           ← map → odom の TF と /map
│   └─ Nav2 一式
└─ d435i.launch.py            ← realsense2_camera（sim では起動しない）
```

★ **`robot.launch.py` が起動するのはここまで**です。ロボットを動かせる状態に
するところまでを受け持ち、その上で何をするかはアプリケーション側が決めます。
リーチも逆運動学もアプリケーション側（`lekiwi_examples`）にあり、別ターミナルで
起動します。

```bash
ros2 launch lekiwi_examples reach.launch.py      # map 上の点へリーチ
ros2 run lekiwi_examples teleop_keyboard         # ベース + アームをキーボードで
```

> ★ ベース側の include には `start_robot_state_publisher:=false` と
> `start_rviz:=false` を渡しています。`/robot_description` は
> TRANSIENT_LOCAL / depth 1 なので、**publisher が 2 つあると後から繋いだ購読者が
> どちらを掴むか非決定**になります（RViz に別のロボットが出る症状）。

---

## モータバスの2モード

`robot.launch.py` は `motor_bus_mode:=split|shared` を必須とし、自動判定しません。

| モード | シリアル所有者 | 電圧・較正 |
| --- | --- | --- |
| split | arm bridgeが`/dev/so101_follower`、base_driverが`/dev/lekiwi` | アーム7.4V/車輪12V、`robots/so_follower` |
| shared | arm bridgeだけが`/dev/lekiwi`を開く。base_driverは内部topicへtickを送る | 全モータ12V、`robots/lekiwi`の9モーターJSON |

sharedの起動順は、全ID接続確認、全トルクOFF、アーム現在位置のラッチ、
車輪ゼロ、ID 1〜6を位置モード、ID 7〜9を速度モード、全トルクONです。

## アームはどう動くのか

いちばん分かりにくいところです。**指令はトピックを 2 回経由します。**

```
① ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory
        │
        ▼
② joint_trajectory_controller  … 経由点を時間で補間する
        │  ros2_control の command interface（プロセス内）
        ▼
③ JointStateTopicSystem        … ros2_control の「ハードウェア」プラグイン
        │  /so101/hardware_commands  (sensor_msgs/JointState)
        ▼
④ lerobot_bridge               … rad → deg、gripper を 0-100% に変換
        │  LeRobot SO101Follower.send_action()
        ▼
⑤ split: /dev/so101_follower / shared: /dev/lekiwiのID 1〜6
```

戻りは逆向きです。

```
⑤ シリアル → ④ lerobot_bridge（deg → rad）
        │  /so101/hardware_states  (BEST_EFFORT)
        ▼
③ JointStateTopicSystem → ② joint_state_broadcaster
        │  /joint_states
        ▼
   robot_state_publisher → /tf
```

実測（モック構成、`ros2 topic info --verbose`）:

```
/so101/hardware_commands : so_arm101（ros2_control）→ lerobot_bridge
/so101/hardware_states   : lerobot_bridge → so_arm101
```

### ★ なぜこんな遠回りをするのか

ふつう ros2_control のハードウェアは **C++ のプラグイン**として書きます。
ここではそうせず、間にトピックを挟んでいます。理由は 2 つです。

1. **モータ設定・較正・単位変換をすべて LeRobot に任せたいから。**
   LeRobot は Python で、しかも `torch` を引き込みます。
   `ros2_control_node`（C++、リアルタイム寄り）と同じプロセスには入れられません。
2. **プロセスを分けられるから。** ブリッジがシリアル異常で落ちても
   `robot_state_publisher` は生き残り、ベースの SLAM / Nav2 は測位を失いません
   （`shutdown_on_bridge_exit:=false`）。

代償は、**指令が届くまでに DDS を 1 往復挟むこと**です。ros2_control の
`update_rate` とブリッジの `update_rate`（50Hz）が別々に回ります。

### 使っているプラグイン

`so101_bringup/control/so101_follower.ros2_control.xacro`:

```xml
<plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
<param name="joint_commands_topic">/so101/hardware_commands</param>
<param name="joint_states_topic">/so101/hardware_states</param>
<param name="trigger_joint_command_threshold">-1</param>
```

`trigger_joint_command_threshold: -1` は「変化が無くても毎周期送る」の意味です。
ブリッジの watchdog が**静止中でも生存を確認できる**ようにするためです。

---

## ベースはどう動くのか

```
/cmd_vel (geometry_msgs/Twist)
    │
    ▼
base_driver（運動学・odom・TFは両モード共通）
    │  3輪オムニの逆運動学（kinematics.py）
    │  vx, vy, wz  →  車輪3個の角速度  →  tick
    ▼
split: StsBus.sync_write_velocity()
shared: /lekiwi/hardware_wheel_commands → lerobot_bridge
    │
    ▼
シリアル /dev/lekiwi   … STS3215 × 3（ID 7/8/9）、12V、速度モード
```

戻りは **`/odom` と `/joint_states`（車輪 3 関節）と TF `odom → base_footprint`**。

### ★ `/odom` は実測ではありません

`base_driver.py` の冒頭にこう書いてあります。

> オドメトリは **送った指令値の積分** であって実測ではない。飽和と整数丸めは
> 反映されるが、スリップも外乱も検出できない。自己位置は後段の LiDAR/SLAM が
> 担当する前提。

つまり:

- 車輪が滑っても `/odom` は動いたことにします
- **アームを振った反動でずれても `/odom` には出ません**
- 自己位置の真の担い手は `slam_toolbox` です

### 安全に関わる作り

| 仕掛け | 何のため |
| --- | --- |
| `cmd_vel_timeout_s`（既定 0.5 秒） | 無指令が続いたら速度をゼロにする。**送信元が落ちても止まる** |
| `finally` の `safe_stop()` | 終了時に「ゼロ送信 → トルク OFF」を必ず実行 |
| `dry_run:=true` | シリアルを開かずに全経路を検証（`sim:=true` がこれを使う） |

★ ただし **`finally` に到達できないとき**（SIGKILL）はどれも走りません。
その話は後述の「安全論理」へ。

---

## `/cmd_vel` に届くまで — 安全機構の順序

Nav2 は `/cmd_vel` を直接publish していません。**3 段構えです。**

```
controller_server / behavior_server
    │  /cmd_vel_nav          （publisher 6、subscriber 1）
    ▼
velocity_smoother            … 加減速を丸める
    │  /cmd_vel_smoothed
    ▼
collision_monitor            … フットプリントに障害物が入ったら止める
    │  /cmd_vel              （publisher 2、subscriber 1 = base_driver）
    ▼
base_driver
```

実測（`ros2 topic info /cmd_vel --verbose`）:

```
Publisher  : collision_monitor, docking_server
Subscriber : lekiwi_base_driver
```

> ★ **`ros2 topic pub /cmd_vel ...` と手で打つと、この 3 段を全部飛ばします。**
> `velocity_smoother` の加速度制限も `collision_monitor` の停止も効きません。
> 動作確認には便利ですが、**床に降ろした状態で使わないこと。**

---

## SLAM と地図

```
/scan            … sllidar_node（実機） or fake_scan（sim）
    │
    ▼
scan_filter      … 前方 ±60° 以外を inf に置き換える
    │  /scan_filtered   （publisher 1、subscriber 4）
    ├──────────────► slam_toolbox   → /map と TF map→odom
    └──────────────► costmap（local / global）
```

### ★ なぜ前方 ±60° に絞るのか

RPLIDAR は 360° を測ります。そのままだと**自分の後輪とボディが障害物として
地図に焼き付きます**。`scan_filter` が右車輪（−60°）から左車輪（+60°）の
アークだけを残します。

`slam_toolbox.yaml` の `scan_topic` も `nav2.yaml` の costmap も
`/scan_filtered` を見ています（生の `/scan` ではありません）。

> ★ **`scan_filter` が起動していないと何も動きません。** `/scan_filtered` が
> publisher 0 / subscriber 4 になり、`map → odom` が永遠に出ず、Nav2 は
> `Invalid frame ID map` を INFO で吐き続けます（エラーではないので気付きにくい）。

---

## TF ツリー — 誰がどの辺を出すか

```
map
 │  slam_toolbox（sim でも実機でも同じ）
 └─ odom
     │  base_driver（指令値の積分）
     └─ base_footprint
         │  robot_state_publisher（URDF の固定ジョイント）
         └─ base_link
             ├─ laser_link              ← 実測 (0.10, 0, 0.03) yaw −7°
             └─ arm_mount_link          ← 実測 (0.08, 0.00, 0.057)
                 └─ arm_base_link
                     │  robot_state_publisher（/joint_states から）
                     └─ … arm_gripper_link
                         ├─ arm_gripper_frame_link       ← リーチの手先
                         └─ wrist_camera_mount_link
                             └─ wrist_camera_link
                                 │  realsense2_camera が /tf_static へ
                                 └─ wrist_camera_depth_optical_frame
```


### ★ 手首カメラに外部キャリブレーションが要らない理由

カメラは URDF で `arm_gripper_link` に**剛体固定**されています。だから
`map → カメラ` は TF を辿るだけで出ます。`realsense2_camera` は
`wrist_camera_link` を**根**として光学フレームを生やすだけで、
`wrist_camera_link` を**子にする TF は出しません**。だから URDF 側が親を与えても
二重定義になりません（`laser_link` と同じパターン）。

---

## `/joint_states` の publisher が 2 つでよい理由

実測すると 2 つです。

```
joint_state_broadcaster   … アーム 6 関節
lekiwi_base_driver        … 車輪 3 関節
```

ふつう「1 トピックに publisher 2 つ」は設計ミスを疑いますが、ここでは正しい形です。

**`robot_state_publisher` は状態を保持しません。** 受け取ったメッセージに
含まれる関節ぶんだけ TF を出します。木が繋がるのは **tf2 のバッファが
(親, 子) の組ごとに最新値を保つ**ためです。

したがって:

- 車輪 3 関節とアーム 6 関節が別メッセージで来ても問題ありません
- ★ **`/joint_states` を購読する側は、複数メッセージにまたがって蓄積する必要があります。**
  1 通目だけ見ると関節が足りません（`lekiwi_examples` の `cartesian_jog.py` と `reach_to_point.py` は
  どちらも辞書に `update()` していく作りです）
- ★ **publisher が死ぬと TF は消えずに「凍り」ます。**
  TF を使う側は `header.stamp` を見て鮮度を判断してください
  （リーチノードの例は [`examples.md`](examples.md)）

---

## 安全論理 — アームとベースで向きが逆

**このリポジトリでいちばん重要な性質です。**

| | 正常終了（`Ctrl+C`） | SIGKILL |
| --- | --- | --- |
| **アーム** | トルク OFF → **落ちる** | トルク ON のまま → **凍る** |
| **ベース** | 速度ゼロ + トルク OFF → 安全 | **最後の指令速度で回り続ける** |

分岐しているのは **「Python の `finally` に到達できるか」だけ**です。

- アーム: `disable_torque_on_disconnect=True` で接続し、`disconnect()` を
  `main()` の `finally` から呼ぶ（`lerobot_backend.py`）
- ベース: `safe_stop()` が `bus.stop()` → `disable_torque()`。これも `finally`
  （`base_driver.py`）
- **STS3215 にコマンドウォッチドッグはありません。** プロセスが消えてもサーボは
  最後に受け取った `Goal_Velocity` を保持し続けます

> ★ だから **`docker kill` を非常停止に使えません。** 1 コンテナなので
> ベースのドライバも道連れになり、ホイールが回り続けます。
> **非常停止は物理スイッチだけ**です。
>
> 異常終了からの復帰は `make release BUS_MODE=split|shared`。
> **アームもホイールもこれ 1 つ**で解放する。
>
> ★ **コンテナを落とす必要はない。** 止まっている必要があるのは launch だけで、
> いちばん多い「launch だけ落ちてコンテナは生きている」場合はそのまま叩ける。
> バスを触ってよいかは `release_all` が `/proc` を見て**ポートごとに**判定し、
> 掴んでいるプロセスが居ればその名前を出して中止する。
>
> 詳細は [`../docker/robot/README.md`](../docker/robot/README.md)。

### ★ アームの起動時にも一瞬トルクが抜けます

`lerobot_backend.py` の接続処理は
`disable_torque()` → 現在位置を `Goal_Position` に書く → 再投入、の順です。
古い `Goal_Position` が残っていてアームが飛ぶのを防ぐためですが、
**その瞬間だけ脱力します**。起動時も人が支える必要があります。

---

## なぜ 1 コンテナなのか

アーム・ベース・LiDAR・カメラを 1 コンテナに入れています。分けると次が起きます。

- `ROS_DOMAIN_ID` の食い違いで `/tf` と `/robot_description` が混信する
- `nav2_msgs` が片方のイメージにしか無く、もう片方から action を叩くと
  `The passed action type is invalid`
- discovery の遅れ、`/robot_description` の二重 latch

運用は **launch を前面で走らせて `Ctrl+C` する**の一本です。

> ★ **最後の 1 つは統合しても消えていません。** `docker compose down` が
> SIGTERM を送るのは**コンテナの PID 1 だけ**で、`exec` したプロセスには
> 届かず SIGKILL されます（実測で確認）。`command:` に launch を書けば
> `down` でも綺麗に止まりますが、そうすると**コンテナを上げた瞬間に
> ロボットが動き出す**ので採っていません。
> **`Ctrl+C` してから `down` する**のが正しい順序です。

停止と復帰の詳細は [`../docker/robot/README.md`](../docker/robot/README.md)。

---

## ワークスペースはどう組み立てられるか

| 段階 | どこで | 何を |
| --- | --- | --- |
| イメージ build | `Dockerfile` | apt / pip の**実行依存だけ** + 上流を `/opt/upstream` へ `vcs import` |
| `make bootstrap` | コンテナ内 | `/opt/upstream` → `src/` へコピー、`colcon build`、静的検査 |
| 実行 | コンテナ内 | ホストの `ros2_ws` を `/ros2_ws` に bind mount |

### ★ なぜ上流を `/opt/upstream` に置くのか

`compose.yaml` が `../..` を `/app` へ bind mount するので（`/ros2_ws` は
`/app/ros2_ws` への symlink）、
**Dockerfile が `/ros2_ws` に置いたものは実行時に完全に隠れます**（実測確認済み）。
マウントの外に置いてからコピーする必要があります。

この方式だと bootstrap が**オフラインで完結**し、`so101_upstream.repos` の
SHA とワークスペースの中身が**ずれようがありません**。

### ★ `--symlink-install` の意味

`install/` の中身が `src/` へのシンボリックリンクになります。だから

- **Python を編集したら launch を上げ直すだけで反映されます**（`colcon build` 不要）
- ファイルを**追加**したときだけ `colcon build` が要ります
- ★ ブランチを切り替えると、消えたファイルを指すリンクが `build/` に残り
  `error: can't copy '...': doesn't exist` で失敗します。
  `bootstrap.sh` が壊れたリンクを持つパッケージを検出して作り直します

---

## 関連

| 知りたいこと | どこ |
| --- | --- |
| 使い方の手順・ノードの書き方 | [`../README.md`](../README.md) |
| リーチとキーボード操作 | [`examples.md`](examples.md) |
| 停止・非常停止・復帰 | [`../docker/robot/README.md`](../docker/robot/README.md) |
