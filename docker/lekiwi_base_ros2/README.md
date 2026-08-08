# LeKiwi ベース + ROS 2 Jazzy + Docker

> **★ 通常の運用は [`../robot/`](../robot/) を使ってください（1 イメージ 1 コンテナの統合スタック）。**
> このディレクトリはベース単体の動作確認・デバッグ用です。
> 統合スタックと同時に起動しないこと（`ROS_DOMAIN_ID` が衝突して `/tf` が混信します）。

LeKiwi の3輪オムニ移動ベース（Feetech STS3215 × 3、モータ ID 7/8/9）を
ROS 2 の `/cmd_vel` で駆動する構成です。オドメトリと TF を出すので、
RPLIDAR や RealSense と同じ TF ツリー上に載せられます。

オドメトリは**送った指令値の積分**であって実測ではありません。スリップも
外乱も検出しないため、自己位置は後段の LiDAR/SLAM が担当する前提です。

## 安全上の注意

**必ず先に読んでください。**

- **12V 給電中にアーム（モータ ID 1〜6）をバスへ繋がないこと。**
  この機体は7.4V版のアームと12V版のホイールが混在しており、
  7.4V版サーボの上限は 8.0V です。12V を掛けると破損します。
- **初回の通電は必ず車体を台に載せ、ホイールを浮かせた状態で行うこと。**
  回転方向の確定（Phase D）まで床に降ろさないでください。
- **電源スイッチを手の届く場所に置くこと。**
  `docker kill` や SIGKILL で停止した場合、**ホイールは最後の指令速度で
  回り続けます**。STS3215 にはコマンドウォッチドッグが無く、ソフト側の
  回避策は存在しません。正常な停止経路は `Ctrl+C` と `docker compose down`
  （どちらも SIGINT が届き、速度ゼロ送信 → トルク OFF が走ります）。
- 有線テザーなので、ケーブル長が行動半径の上限です。

## 前提

- Linux ホスト（X11 または XWayland を利用できるデスクトップ環境）
- Docker Engine と Docker Compose v2（`docker compose` コマンド）
- LeKiwi ベース本体、WaveShare サーボコントローラ、12V 電源

> macOS では Docker にシリアルデバイスを渡せないため、この構成は動きません。
> Mac から直接叩く場合は `lerobot_examples/lekiwi_base_keyboard.py` を使ってください。

以下のコマンドは、この README があるディレクトリで実行します。

```bash
cd docker/lekiwi_base_ros2
```

## 1. USBデバイスを確認する

サーボコントローラを接続する前後で次を実行します。

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
dmesg --follow
```

通常は `/dev/ttyACM0` です（CDC-ACM として認識されます）。
VID/PID を確認します。

```bash
udevadm info --attribute-walk --name=/dev/ttyACM0 | grep -m1 -E 'idVendor|idProduct'
```

確認した値を `99-lekiwi.rules` の `XXXX` に反映してからコピーします。

```bash
sudo cp 99-lekiwi.rules /etc/udev/rules.d/99-lekiwi.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
# 反映されない場合はUSBを抜き差しする
ls -l /dev/lekiwi
```

ルールを入れない場合も `/dev/ttyACM0` のまま使用できますが、
**ModemManager 対策**が入らないため接続直後に通信が不安定になることがあります
（詳細は `99-lekiwi.rules` のコメント参照）。

現在ユーザーを `dialout` グループへ追加します。追加後はログアウト・ログインが必要です。

```bash
sudo usermod -aG dialout "$USER"
```

一時的に反映する場合（現在のシェルのみ）
```bash
newgrp dialout
```

## 2. 環境ファイルを作る

```bash
cp .env.example .env
sed -i "s/^DIALOUT_GID=.*/DIALOUT_GID=$(getent group dialout | cut -d: -f3)/" .env
```

udevルールを使う場合は `.env` を次のように変更します。

```dotenv
LEKIWI_DEVICE=/dev/lekiwi
```

複数のROS 2システムが同じLANにある場合は、衝突しない `ROS_DOMAIN_ID`
（0～232）に変更してください。

## 3. RViz用のX11アクセスを許可する

ホスト上で、ローカルのrootユーザー（コンテナ内ユーザー）だけを許可します。

```bash
xhost +si:localuser:root
```

## 4. イメージをビルドして起動する

```bash
docker compose build
```

**まずサーボに通電しない状態でドライランしてください。**
シリアルを開かずに運動学・オドメトリ・TF・RViz の全経路を検証できます。

サーボコントローラを **USB 接続したまま通電しない** 場合（`/dev/lekiwi` は見える）:

```bash
DRY_RUN=true docker compose up
```

**実機がまだ手元に無い場合**は `compose.dryrun.yaml` を使います。

```bash
docker compose -f compose.dryrun.yaml build
docker compose -f compose.dryrun.yaml up
```

> `compose.yaml` は `devices:` でシリアルデバイスを無条件にマップするため、
> デバイスノードが存在しないホストでは `DRY_RUN=true` を付けても
> `error gathering device information ... no such file or directory` で
> **起動できません**（コンテナ生成前に daemon が出すエラーなので環境変数では
> 回避できず、Compose のリストマージでは `devices` を除去できません）。
> `compose.dryrun.yaml` は `devices` / `group_add` を持たず `dry_run:=true` を
> 固定した専用ファイルで、ビルド対象は本番と同一イメージです。

RVizに車体モデルが表示され、TF が
`odom → base_footprint → base_link → base_*_wheel_link` で繋がっていれば
ソフト側は正常です。マゼンタの小さい箱は **取付位置が未実測**のフレーム
（`laser_link` と `camera_mount_link`）の目印です。

### 単体テスト（実機も ROS 2 も不要）

`kinematics.py` と `raycast.py` は numpy しか import しないため、Docker を立てずに
単体で回せます。
Phase D の前に「運動学は正しい」と言い切れる状態を作っておくと、実機で向きが
おかしかったときに配線・符号の問題へ切り分けられます。

```bash
cd ../../ros2_ws/src/lekiwi_base_bringup
uv run --no-project --with pytest --with numpy pytest test/ -v
```

運動学側は飽和時の3輪比例縮小、往復の一致、README 6.2 の Phase D が前提にしている
符号の規約、`max_ticks` から逆算される到達可能速度を固定しています。
レイキャスト側（`fake_scan` の距離計算）は壁までの距離、最近傍の優先、後方の
レイを拾わないこと、NaN を出さないことを固定しています。全 70 件。
コンテナ内から `colcon test` でも実行できます。

### 実機なしで SLAM / Nav2 を回す

`fake_scan`（仮想 2D LiDAR）を使うと、シリアルも LiDAR も無い状態で
「走らせる → 地図ができる → ゴールを与えると経路を追従する」まで閉ループで
検証できます。

```bash
docker compose -f compose.dryrun.yaml run --rm --service-ports lekiwi-base-dryrun \
  ros2 launch lekiwi_base_bringup sim_nav.launch.py
```

RViz が開いたら "2D Goal Pose" でゴールを与えます。コマンドからも送れます。

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.2}, orientation: {w: 1.0}}}'
```

構成は次のとおりで、`/scan` の供給元が実機の LiDAR ではなく `fake_scan` に
なっているだけです。

```
base_driver (dry_run) ←─ /cmd_vel ←─ Nav2 (MPPI, Omni)
      │ odom, TF                          ↑
      ↓                                   │ /map, TF (map→odom)
fake_scan ──/scan──→ slam_toolbox ────────┘
```

既定の仮想世界は 5m x 4m の部屋に箱を 2 つ置いたもの
（`config/fake_scan.yaml` で変更できます）。

**既定のままでは SLAM のロバスト性は検証できません。** `dry_run` の
オドメトリは指令値の積分で誤差ゼロなので、スキャンマッチングが自明に成功します。
意味のある検証にするには odom へ系統誤差を入れてください。

```bash
# odom が実際より 3% 多く報告する状態（車輪半径の見積もり誤りに相当）
ros2 launch lekiwi_base_bringup sim_nav.launch.py odom_trans_scale:=1.03
```

RViz でオレンジの線（`/fake_scan/world` = 真の壁）と SLAM の地図を重ねると、
地図の歪みが見えます。

#### ここで検証できないこと

- **Phase D**（車輪の回転方向・前方向・鏡像の確定）は代替できません。未確定の
  ままだと Nav2 が正しい `/cmd_vel` を出しても機体は違う方向へ走ります。
- `laser_link` の取付位置は URDF の仮値（z=0.09）です。実機では実測値へ。
- スリップ、床面摩擦、サーボの追従遅れ、実際の地図品質。

問題なければ実機を接続して起動します。

```bash
docker compose up
```

停止は `Ctrl+C`、バックグラウンド起動と停止は次のとおりです。

```bash
docker compose up -d
docker compose logs -f
docker compose down
```

## 5. トピックとTFを確認する

別ターミナルからコンテナ内で確認できます。

> **`/entrypoint.sh` を挟むのは必須です。** `docker compose exec` はイメージの
> ENTRYPOINT を通らないため、`/opt/ros/jazzy/setup.bash` が source されず
> `exec: "ros2": executable file not found in $PATH` になります。
> `entrypoint.sh` は source して `exec "$@"` するだけなので、これで解決します。
> （`docker compose run` は ENTRYPOINT を通るので、そちらは不要です。）
>
> `compose.dryrun.yaml` を使っている場合はサービス名も読み替えてください:
> `docker compose -f compose.dryrun.yaml exec lekiwi-base-dryrun /entrypoint.sh ros2 ...`

```bash
docker compose exec lekiwi-base /entrypoint.sh ros2 topic list
docker compose exec lekiwi-base /entrypoint.sh ros2 topic hz /odom
docker compose exec lekiwi-base /entrypoint.sh ros2 topic echo /joint_states --once
docker compose exec lekiwi-base /entrypoint.sh ros2 run tf2_tools view_frames
docker compose exec lekiwi-base /entrypoint.sh ros2 run tf2_ros tf2_echo odom base_footprint
```

期待する主なトピックは次のとおりです。

- `/cmd_vel`: 速度指令の入力（`geometry_msgs/Twist`）
- `/odom`: オープンループのオドメトリ
- `/joint_states`: 3輪の回転角と角速度
- `/tf`: `odom → base_footprint`

ドライランのまま `/cmd_vel` を流せば、ハードウェア無しで運動学からTFまでを検証できます。

```bash
docker compose exec lekiwi-base /entrypoint.sh \
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

`/odom` の x が 0.1 m/s で増え、RVizのモデルが動けば正常です。
publish を止めて0.5秒後に速度がゼロに戻れば、ウォッチドッグも動いています。

## 6. 動作確認（車体を浮かせた状態で）

> ホイールを床から離し、アームのコネクタが抜けていることを目視確認してください。

### 6.1 1輪ずつ確認する

`test_wheel_ticks` は `/cmd_vel` と運動学を完全にバイパスして生の速度値を
書き込む診断用パラメータです。1輪ずつ切り分けられます。

```bash
docker compose exec lekiwi-base /entrypoint.sh ros2 param set /lekiwi_base_driver test_wheel_ticks "[300,0,0]"   # 左輪のみ
docker compose exec lekiwi-base /entrypoint.sh ros2 param set /lekiwi_base_driver test_wheel_ticks "[0,300,0]"   # 後輪のみ
docker compose exec lekiwi-base /entrypoint.sh ros2 param set /lekiwi_base_driver test_wheel_ticks "[0,0,300]"   # 右輪のみ
docker compose exec lekiwi-base /entrypoint.sh ros2 param set /lekiwi_base_driver test_wheel_ticks "[0,0,0]"     # 通常動作へ戻す
```

**3輪それぞれについて、外側から見たリムの回転方向を記録してから**次へ進んでください。

過負荷でサーボが停止した場合は、復帰サービスでラッチを解除できます。

```bash
docker compose exec lekiwi-base /entrypoint.sh ros2 service call /lekiwi_base_driver/recover std_srvs/srv/Trigger
```

### 6.2 向きと符号を確定する（3つのノブを順に）

3つの調整項目は独立しているので、**必ずこの順序で**1つずつ確定します。
確定した値は `ros2_ws/src/lekiwi_base_bringup/config/base.yaml` に書いてください。
**これがこの機体のキャリブレーション値です。**

#### D1: 各輪の回転方向 → `wheel_direction_signs`

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'
```

純回転では運動学行列の第3列が3輪とも `+base_radius` なので、
**3輪とも同じ速度値**になります。したがって**3輪のリムは同じ向きに回るはず**です。
逆を向く輪があれば、その要素を `-1.0` にします（順序は left, back, right）。

```yaml
wheel_direction_signs: [-1.0, 1.0, 1.0]   # 左輪が逆だった場合
```

#### D2: 機体の前方向 → `wheel_angle_offset_deg`

D1が通ってから実施します。

```
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

**後輪（ID 8）の反対側**へ進むのが正しい向きです。

| 症状 | `wheel_angle_offsebasht_deg` |
| --- | --- |
| 正しい | `-90.0`（既定） |
| 90°ずれる | `0.0` または `-180.0` |
| 逆向き（180°） | `90.0` |

> このパラメータは `lekiwi_description` の車輪方位角
> （left=60°, back=180°, right=300°）と同じ規約から導かれています。
> **URDF側だけを変えないでください。**

#### D3: 左右の鏡像 → `motor_ids` の順序

D2が通ってから実施します。

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.1}}'
```

**左**へ平行移動するのが正しい動きです。前進と回転が正しいのに横だけ逆なら、
左右の行が入れ替わっています。

```yaml
motor_ids: [9, 8, 7]
```

> 符号反転では鏡像は直せません。`wheel_direction_signs` で直そうとしないでください。

#### D4: 総合確認

すべて正しければ、teleopで約1m四方を走らせ、RVizの `/odom` 軌跡が
概ね四角くなることを確認します。オープンループなのでドリフトは想定内で、
**見るのは形と向きだけ**です。

RViz上で車輪が逆回転して見える場合は表示だけの問題で、走行には影響しません。

## 7. キーボードで走らせる

`teleop_twist_keyboard` は `ros2 launch` 配下では標準入力を取れないため、
別ターミナルから起動します。

```bash
docker compose up -d
docker compose exec -it lekiwi-base /entrypoint.sh \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.1 -p turn:=0.5
```

```
   u    i    o
   j    k    l        k: 停止
   m    ,    .

速度変更: q（上げる） / z（下げる）
横移動:   Shift を押しながら（ホロノミックモード）
```

> ## ★ `j` / `l` は**旋回**です（横移動ではありません）
>
> `teleop_twist_keyboard` では小文字 `j`/`l` が `angular.z`、
> **横移動は大文字 `J`/`L`（Shift）**です。オムニだからと `j` を押すと
> その場で回ります。**機体の故障ではありません。**
>
> 統合スタック（`docker/robot`）には**横移動を `j`/`l` に置いた自作ノード**が
> あります。そちらのほうがこのベースには自然です。
>
> ```bash
> ros2 run lekiwi_examples teleop_keyboard   # j/l = strafe, [/] = 旋回
> ```
>
> → [`docs/examples.md`](../../docs/examples.md)

**既定の `speed:=0.5` から始めないでください。** 0.1 → 0.2 と徐々に上げます。

### 速度の上限について

`max_ticks: 3000` がサーボ側の上限を決めており、ここから逆算される
**実際に到達可能な速度**は次のとおりです。

| 方向 | 上限 |
| --- | --- |
| 前後（x） | 約 0.27 m/s |
| 横（y） | 約 0.23 m/s |
| 旋回（z） | 約 105 deg/s（1.84 rad/s） |

これを超える指令を出しても**3輪が比例縮小されるだけ**で進行方向は保たれますが、
指令値と実速度が乖離して分かりにくくなります。起動ログに実際の上限が出ます。

## トラブルシュート

### `No such file or directory: /dev/ttyACM0`

USBを抜き差しして `ls -l /dev/ttyACM*` を確認し、実際のデバイスを
`LEKIWI_DEVICE` に設定します。コンテナ起動後に接続した場合は、Composeの
device mappingを作り直すため `docker compose down && docker compose up` が必要です。

### `Permission denied`

```bash
ls -ln /dev/ttyACM0
getent group dialout
grep DIALOUT_GID .env
```

デバイスのグループIDと `.env` の `DIALOUT_GID` が一致することを確認します。
ホストユーザーのグループ追加後に再ログインしていない場合も反映されません。

### `応答しないモータ: [...]`

- 12V電源が入っているか確認する（ホイールは12V版で、9V未満では駆動段が動きません）
- サーボのデイジーチェーンのコネクタを確認する
- `docker compose run --rm lekiwi-base ros2 run lekiwi_base_bringup sts_bus --port /dev/lekiwi --ping`
  で ID ごとの応答を確認する（`model 777` が正常）

### `Incorrect status packet!` が散発的に出る

- **ModemManager** が接続直後にポートを探っている可能性が最も高いです。
  `99-lekiwi.rules` を導入したか、`systemctl status ModemManager` を確認します
- 他のプロセスがポートを使っていないか `lsof /dev/lekiwi` で確認する
- USBハブを外し、直接接続する

### ホイールが回らない（通信は成功している）

```bash
docker compose exec lekiwi-base /entrypoint.sh \
  ros2 run lekiwi_base_bringup sts_bus --port /dev/lekiwi --diagnostics
```

- 電圧が 9V 未満 → 12V電源を確認（ホイールは12V版）
- `load` が 1000 前後で張り付く → 過負荷。`recover` サービスを呼ぶ
- 温度が高い → 冷ましてから再開

### 進む向きがおかしい

「6.2 向きと符号を確定する」を D1 → D2 → D3 の順に実施してください。
順序を飛ばすと結果が解釈できなくなります。

### RVizが開かない／QtまたはGLXエラー

```bash
echo "$DISPLAY"
ls -l /tmp/.X11-unix
xhost
```

`xhost +si:localuser:root` をデスクトップへログインしているユーザーの端末から
再実行してください。設定変更後は次のようにコンテナを再作成します。

```bash
docker compose down
docker compose up --force-recreate
```

### ロボットが暴走したら

**電源スイッチを切ってください。** `docker kill` ではホイールが止まりません
（SIGKILL では停止処理が走らないため）。正常な停止は `Ctrl+C` または
`docker compose down` です。

## ファイル構成

- `Dockerfile`: ROS 2 Jazzy、feetech-servo-sdk、RViz、ドライバのビルド
- `compose.yaml`: シリアルデバイス、ホストネットワーク、X11をコンテナへ渡す設定
- `compose.dryrun.yaml`: 実機なし検証用（`devices` を持たず `dry_run:=true` 固定）
- `99-lekiwi.rules`: 安定した `/dev/lekiwi` 名、dialout権限、ModemManager除外
- `../../ros2_ws/src/lekiwi_base_bringup`: ドライバ本体、launch、設定、RViz
- `../../ros2_ws/src/lekiwi_base_bringup/test`: 運動学とレイキャストの単体テスト（ROS 2 不要）
- `../../ros2_ws/src/lekiwi_base_bringup/config/nav2.yaml`: Nav2 設定（オムニ向け差分入り）
- `../../ros2_ws/src/lekiwi_base_bringup/config/fake_scan.yaml`: 仮想 LiDAR と仮想世界
- `../../ros2_ws/src/lekiwi_description`: URDF/xacro（基本図形。`use_mesh:=true` で拡張可）

LeKiwi 本体: https://github.com/SIGRobotics-UIUC/LeKiwi
ROS 2 Jazzy Dockerガイド: https://docs.ros.org/en/jazzy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html
