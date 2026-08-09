# LeKiwi + SO-101 — 統合スタック（1 イメージ 1 コンテナ）

LiDAR / SLAM / Nav2 / SO-101 アーム / 手首カメラ（RealSense）を **1 つのイメージ、
1 つのコンテナ、1 つの launch** で動かす。

旧4コンテナ構成を置き換える正規入口です。サブシステム単体のDocker構成は
切り分け用途に残しています。

## なぜ 1 つにまとめたか

分けていた理由は「アームとベースで安全論理が逆で、compose の `stop_signal` は
サービス単位にしか設定できないから」だった。だが実際に踏んだ問題の多くは、
**コンテナが分かれていること自体**が原因だった。

| 分割していたときの問題 | 統合後 |
| --- | --- |
| `ROS_DOMAIN_ID` の食い違い（realsense 単体は既定 0、統合スタックは 7） | **起こりえない** |
| `nav2_msgs` がベースのイメージにしか無く、アーム側から action を叩くと `The passed action type is invalid` | **消える**（全部同じイメージ） |
| DDS discovery の遅れで `ros2 topic list` が不完全 | 大幅に軽減 |
| `/robot_description` の publisher を 1 つに保つための調整 | launch が 1 つなので自明 |
| `docker compose down` が `exec` した launch に SIGINT を届けない | ★ **これは統合しても消えない。** launch を前面で `Ctrl+C` する運用で回避する（下記） |
| compose の `camera_name` と URDF の `wrist_camera_name` の不一致 | launch が同じ値を両方へ渡す |

代償:

- **`privileged: true` が全体に波及する。** RealSense は USB の再接続でノード番号が
  変わるので `devices:` では掴めない。アームとホイールを駆動するプロセスも
  特権で動くことになる
- **イメージが大きい。** 旧 so101 4.96GB / lekiwi-base 4.75GB。共通レイヤがあるので
  単純合計にはならない
- **停止方針が 1 つになる。** それを補うのが `make release BUS_MODE=...`（後述）

## 構成

```
        map (slam_toolbox / amcl)
         └ odom (base_driver のオドメトリ積分)
            └ base_footprint → base_link
                               ├ laser_link      ← RPLIDAR
                               └ arm_mount_link
                                  └ arm_base_link … arm_gripper_link
                                       ├ arm_gripper_frame_link  ← リーチの手先
                                       └ wrist_camera_mount_link
                                          └ wrist_camera_link  ← 手首カメラ
```

| コンテナ | イメージ | 中身 |
| --- | --- | --- |
| `robot` | `local/lekiwi-so101:jazzy` | ロボット本体（RSP / base_driver / sllidar / slam / Nav2 / LeRobotブリッジ / ros2_control / RealSense / RViz）。リーチ等のアプリは別起動 |

## 環境構築

```bash
cd docker/robot
cp .env.example .env      # ★ 先に実機に合わせて編集する
make build
make bootstrap            # ★ 初回とパッケージ追加時。上流取得 + colcon build + 静的検査
```

`make bootstrap` は `ros2_ws` をホストからマウントしたまま `colcon build
--symlink-install` する。成果物はホスト側の `ros2_ws/build`・`install` に残るので、
イメージを作り直しても消えない。

## 動かし方

```bash
make run-split SO101_ROBOT_ID=my_follower
make run-shared LEKIWI_ROBOT_ID=my_lekiwi
make mock-split
make mock-shared
```

splitは `robots/so_follower/<SO101_ROBOT_ID>.json`、sharedは
`robots/lekiwi/<LEKIWI_ROBOT_ID>.json` を使います。shared較正は既存の6軸JSONを
変換せず、全モータ12V・ID 1〜9を接続した状態で次のように新規作成します。

```bash
cd ../../lerobot_examples
uv sync  # 初回のみ
uv run lerobot-calibrate \
  --robot.type=lekiwi --robot.id=my_lekiwi --robot.port=/dev/lekiwi
```

sharedの固定配置は `7=left, 8=back, 9=right` です。異なるIDの較正JSONは起動時に
拒否されます。

`make run-shared` の実体（splitは2ポートをmount）:

```bash
docker compose -f compose.yaml -f compose.shared.yaml up -d
docker compose exec -it robot /entrypoint.sh \
  ros2 launch lekiwi_so101_bringup robot.launch.py \
    motor_bus_mode:=shared backend:=lerobot robot_id:=my_lekiwi
```

**コンテナは bash を起動するだけで、launch は人が手で叩く。** 理由は 2 つ。

1. `docker compose up -d` でロボットが動き出さないようにするため
   （`command:` に launch を書くと**上げた瞬間にトルクが入る**）
2. 起動のたびに引数が変わるため（`backend` / `robot_id` / `sim` / `use_saved_map`）

> ★★ **代償: `make down` だけでは止まらない。**
> `docker compose down` が SIGTERM を送るのは **PID 1 だけ**で、
> `exec` した launch には**届かず SIGKILL される**（実測で確認）。
> つまり **トルクが入ったまま残る**。
> 必ず launch を `Ctrl+C` してから `make down` すること。復帰は
> `make release BUS_MODE=split|shared`。

ros2 コマンドを使うときは別端末で:

```bash
make shell        # docker compose exec -it robot bash
```

### `robot.launch.py` の主な引数

| 引数 | 既定 | 意味 |
| --- | --- | --- |
| `motor_bus_mode` | **既定なし・必須** | `split`（2ポート）/ `shared`（1ポート、全12V、ID 1〜9） |
| `sim` | `false` | `true` でシリアルも LiDAR も開かない（`base_driver` は dry_run、スキャンは `fake_scan`） |
| `backend` | `mock` | `lerobot` で実機のアーム。`sim` とは独立 |
| `robot_id` | （空） | `backend:=lerobot` では必須の LeRobot 較正 ID |
| `use_saved_map` + `map_file` | `false` | `true` で slam_toolbox の代わりに map_server + AMCL |
| `start_base` / `start_camera` / `start_lidar` / `start_rviz` | `true` | 部分起動 |
| `mock_wrist_camera_optical` | `false` | カメラ実機なしで光学フレームだけ出す（`sim` 用） |

## 停止手順

**★ 順番を守ること。正常終了でトルクが切れてアームが落ちる。**

```bash
# 1. reach.launch.pyを起動している場合、アームを低く畳む
make stow
# 2. アームを支え、make run-split/shared の端末で Ctrl+C
make down      # 3. コンテナを片付ける
```

`robot.launch.py` 単独では `/so101/stow` を提供しません。`make stow` はサービスが
無ければ直ちにエラーにします。その場合は使用中のアプリケーションまたは
ros2_controlでアームを安全な低い姿勢へ移してから停止してください。

## ★ 非常停止は物理スイッチだけ

**`docker kill` を非常停止に使わないこと。**

旧 4 コンテナ構成では「アームのコンテナだけ SIGKILL すれば姿勢が凍る」という
手が使えた。1 コンテナではベースのドライバも道連れになり、
**ホイールが最後の指令速度で回り続ける**。

| やりたいこと | 正しい手段 |
| --- | --- |
| いますぐ全部止めたい | **物理スイッチ（電源）を切る** |
| ソフト的に安全に止めたい | アームを低くする（reach起動中なら `make stow`）→ launchを `Ctrl+C` |
| 走り出したホイールだけ止めたい | **`make release-wheels BUS_MODE=split|shared`** |

## ★ 異常終了したとき何が起きるか

停止処理はすべて Python の `finally` にある。分岐しているのは
**「`finally` に到達できるか」だけ**（コードで確認済み）。

| 経路 | アームのトルク | ホイール |
| --- | --- | --- |
| `Ctrl+C` / SIGTERM | **OFF → 支えが無ければ落ちる** | 速度ゼロ + トルク OFF |
| shared bridgeのPython例外 | **OFF → 支えが無ければ落ちる** | 速度ゼロ → 全IDトルクOFF |
| **SIGKILL**（`docker kill` / OOM / コンテナ強制削除） | **ON のまま → 凍る** | **最後の指令速度で回り続ける** |

- アーム: `disable_torque_on_disconnect=True` で接続し、`disconnect()` を
  `main()` の `finally` から呼ぶ（`so101_bringup/lerobot_backend.py`）
- splitのベース: `safe_stop()` が `bus.stop()` → `disable_torque()`。これも `finally`
- sharedのベース: bridgeへゼロ指令を送り、最終停止と全IDのトルクOFFは
  共有bridgeの `disconnect()` が保証する
- **STS3215 にコマンドウォッチドッグは無い。** プロセスが消えてもサーボは
  最後に受け取った `Goal_Velocity` を保持し続ける

### 復帰コマンド

**★ コンテナは落とさなくてよい。** 止まっている必要があるのは launch だけ。

```bash
make release-check BUS_MODE=shared
make release BUS_MODE=shared
make release-wheels BUS_MODE=shared
```

いちばん多い故障は「**launch だけが落ちて、コンテナは生きている**」で、
そのときシリアルポートは空いている。`make release BUS_MODE=...` はコンテナが生きていれば
その中で `exec`、死んでいれば使い捨てコンテナで実行するので、
**どちらの状態でも同じ 1 コマンドで通る**。

> ★ バスを触ってよいかの判定は `release_all` 自身が持っている。
> `/proc` を見て「そのポートを開いているプロセスが居るか」を**ポートごとに**
> 確認し、居ればそのプロセス名を出して中止する。
>
> ```
> ★ /dev/so101_follower は PID 90 (so101_lerobot_bridge) が開いています。
>    Feetech のバスはマスタが 1 つだけ。ここで触ると混線し、
>    ブリッジが通信異常と判定してトルクを切ります（★ アームがその場で落ちます）。
>    先に launch を止めてから実行すること（コンテナは落とさなくて構いません）。
> ```
>
> splitではポートごとなので、**アーム側が掴まれていてもホイールだけは解放できる**。
> sharedでは1ポートをbridgeだけが所有するため、launchを止めてから実行する。

実体は ROS を一切使わず、シリアルポートを直接開く:

```bash
ros2 run lekiwi_so101_bringup release_all --bus-mode split|shared \
  [--only both|wheels|arm] [--yes] [--dry-run]
```

`--dry-run` は `Torque_Enable` を**読むだけで一切書き込まない**。
「いまトルクが入っているのか」を確かめたいときに使う（アームは落ちない）。
解放の前後で比べると、実際に効いたことが確認できる。

```bash
make release-check BUS_MODE=shared  # = release_all --bus-mode shared --dry-run
```

| 対象 | ポート / ID | やること |
| --- | --- | --- |
| ホイール | `/dev/lekiwi`、7/8/9 | `stop()`（Goal_Velocity=0）→ `disable_torque()` |
| アーム | split: `/dev/so101_follower` / shared: `/dev/lekiwi`、1〜6 | **`disable_torque()` のみ** |

> ★ **アームに `stop()` を使わない。** `stop()` が書く `Goal_Velocity` は、速度モード
> では速度指令だが、**位置モードでは速度上限**であって意味が違う。0 を書いたときの
> 挙動がファームウェア依存になるので、トルクを切るだけにする。
>
> ★ **アームはトルクを切ると落ちる。** 凍ったアームを解放するのが目的なので
> それが正しい挙動。`--yes` が無ければ確認を求める。**人が支えてから実行すること。**
>
> ★ 実行後に `Torque_Enable` を読み戻して表示する。`disable_torque()` は ID ごとの
> 失敗を握り潰すので、「呼べた」ことは「切れた」ことを意味しない。
> **読み戻して 0 だった ID だけ成功**として扱う。

## 個々のノードが落ちたとき何が起きるか

1 コンテナ・1 launch なので、**ノードが 1 つ落ちても他は生き続けます。**
launch 全体が落ちた場合（SIGKILL）は前節を参照。

| 落ちたもの | 影響 | 復帰 |
| --- | --- | --- |
| **アームのブリッジ**（シリアル異常 / watchdog期限切れ） | splitはアームだけトルクOFF。sharedは車輪を止めて全IDをトルクOFF。RSPは生き残る | launchを上げ直す |
| `base_driver` | `/odom` が止まる。splitは速度ゼロ+トルクOFF、sharedはbridgeのwheel watchdogが速度ゼロにする | 同上 |
| `slam_toolbox` | `map → odom` が**凍る**（消えない）。リーチは `REJECTED_STALE_TF` で拒否し、**黙って古い座標で解かない** | 同上 |
| `scan_filter` | `/scan_filtered` の publisher が 0 になり `map → odom` が出なくなる。★ Nav2 は `Invalid frame ID map` を **INFO で**吐き続けるのでエラーに見えない | 同上 |
| `realsense2_camera` | 点群が止まるだけ。TF もリーチもナビも影響を受けない | 同上 |

> ★ splitでは**アームの故障をアームだけに閉じ込めています。** sharedでは同じ
> シリアルバスを所有するbridgeの障害なので、安全のため車輪停止後に全IDを解放します。
> `robot.launch.py` は `follower.launch.py` を `shutdown_on_bridge_exit:=false`
> で include している。既定の `true` だと、ブリッジが落ちた瞬間に launch service
> 全体が止まり、**同居している唯一の `robot_state_publisher` も道連れ**になって
> `base_footprint → laser_link` の TF が消え、SLAM と Nav2 が測位を失う。

## 健全性チェック

```bash
make check
```

- `/robot_description` の publisher = **1**（2 だと RViz に別のロボットが出る）
- `/joint_states` の publisher = **2**（車輪 3 関節 / アーム 6 関節）
- コントローラ 3 つが `active`
- **`ros2 action list` に `navigate_to_pose` と `follow_joint_trajectory` が同時に見える**
  ← 分割時は別コンテナだったのでこれができなかった
- `map → arm_gripper_frame_link` と `map → wrist_camera_depth_optical_frame` の TF

## インターフェース・CLI テストコマンド

**→ [`../../README.md`](../../README.md)**

トピック / サービス / アクションの一覧と CLI テストコマンドは
ルートの README にまとめてある。

このディレクトリの README は**起動と停止**に絞ってある。

## 検証状況

### Mac（実機なし）で確認済み

| # | 内容 | 結果 |
| --- | --- | --- |
| M-1 | イメージが 1 つビルドできる | OK。`local/lekiwi-so101:jazzy` **7.85GB**（旧 so101 4.96GB + lekiwi-base 4.75GB。上流を焼き込んで +0.04GB） |
| M-2 | 全パッケージが1ワークスペースでビルドできる | OK（shared用interfaceとアプリケーション分離後も `make bootstrap` で一括ビルド） |
| M-3 | pip の numpy 2.2.6 でベース側のテストが通る | OK。`lekiwi_base_bringup` **70 件**を含む全 **136 件** pass |
| M-4 | コンテナが 1 つ | OK。`docker ps` が 1 行 |
| M-5 | `map → arm_gripper_frame_link` / `map → wrist_camera_depth_optical_frame` | OK。`(0.471, -0.000, 0.315)` / `(0.394, -0.071, 0.323)` |
| M-6 | `/robot_description`=1、`/joint_states`=2、コントローラ 3 つ active | OK |
| M-7 | **`ros2 action list` に nav2 と ros2_control が同時に見える** | OK。`/navigate_to_pose` と `/joint_trajectory_controller/follow_joint_trajectory` |
| M-8 | リーチとナビが動く | OK。`ACCEPTED`→`SUCCEEDED` (residual 0.0042)、`REJECTED_OUT_OF_RANGE`、Nav2 ゴール受理後にオドメトリが目標へ動いた |
| M-9 | **`Ctrl+C` で正常終了する** | OK。**24 プロセスすべて cleanly、died 0、トレースバック 0、生き残り無し** |
| M-10 | `release_all` がポート不在で正しく失敗する | OK。診断メッセージ + exit 1。非対話端末では `--yes` を促して中止 |

> M-9 の過程で、`so101_lerobot_bridge` と `scan_filter` が `Ctrl+C` のたびに
> `KeyboardInterrupt` のトレースバックを出して `exit code -2` で死んでいた
> （**既存の挙動**。トルク OFF は `finally` が担うので安全性には影響しない）。
> 統合後は停止経路が `Ctrl+C` 一本になるため、正常な停止でログが赤くなると
> 本物の異常を見落とす。`base_driver.py` と同じ扱いに揃えて解消した。

### ★ 実機（Linux PC）でしか確認できないこと

| # | 内容 |
| --- | --- |
| H-1 | **`release_all` が実際にトルクを落とすか。** `Torque_Enable` の読み戻しで確認 ← **最重要** |
| H-2 | 意図的に `docker kill` → ホイールが回り続けることを確認 → `release_all` で止まるか（**★ 車輪を浮かせて、人が立ち会う**） |
| H-3 | `privileged` で splitの3デバイス / sharedの2デバイス + RealSenseが見えるか |
| H-4 | 起動時間（1 プロセスに 24 ノードが乗る） |
| H-5 | `Ctrl+C` で実機のトルク OFF とホイール停止が両方走るか |

**H-1 が通らなければ 1 コンテナ化は安全側に成立しない。** 最初に確認すること。

## 旧構成との対応

| 旧 | 統合後 |
| --- | --- |
| `docker/lekiwi_base_ros2` | このスタックに吸収（`base_driver` / `scan_filter` / SLAM / Nav2） |
| `docker/rplidar_ros2` | このスタックに吸収（`sllidar_node`）。`sllidar_ros2` は `so101_upstream.repos` へ **SHA 固定で移した**（以前は `--branch main` で未固定） |
| `docker/so101_ros2` | このスタックに吸収 |
| `docker/realsense_ros2` | このスタックに吸収 |
| `docker/lekiwi_so101_bringup`（4 コンテナ） | このスタックが置き換える |
| `arm.launch.py` | `robot.launch.py` が include している（アーム側だけ動かしたいときは今も使える） |

サブシステム単体の4ディレクトリは切り分け用に残しています。旧統合4コンテナ構成は
削除済みです。単体構成とこの構成を同時に起動しないでください（guardが停止します）。
