# lekiwi_examples

**ロボットの上で動かすサンプルプログラム。**

`robot.launch.py` が**起動していることが前提**です。このパッケージのコードは
ハードウェアに直接触らず、ROS のインターフェース（トピック / アクション / TF）
だけを使います。

```
┌──────────────────────────────────────────────┐
│ robot.launch.py                              │  ← ロボット本体
│   ドライバ / ros2_control / SLAM / Nav2 / カメラ │    （別ターミナル）
└──────────────────────────────────────────────┘
                    ▲ ROS のインターフェースだけ
┌──────────────────────────────────────────────┐
│ lekiwi_examples                              │  ← ここ
│   リーチ / 逆運動学 / キーボード操作             │
└──────────────────────────────────────────────┘
```

**★ 自分のプログラムもここに置いてください。** 書き方は
[`../README.md`](../README.md)。

---

## 使い方

まず別ターミナルでロボットを起動しておきます。

```bash
# コンテナ内で
ros2 launch lekiwi_so101_bringup robot.launch.py \
    backend:=lerobot robot_id:=my_follower
```

### 1. 最小構成の例 — まずこれを読む

**上から下へ一直線に動かす例**と、**呼ばれた瞬間に保存するサービス**の
2 本に分かれています。役割が違うので分けてあります。

| | いつ動くか | 形 |
| --- | --- | --- |
| `example_sequence` | 起動から終了まで一度だけ | 手順を上から順に実行して終わる |
| `image_saver` | **呼ばれたとき** | 常駐してサービスを待つ |

#### 1-1. `example_sequence` — アームとnavigationの最小構成

```bash
# コンテナ内
ros2 run lekiwi_examples example_sequence
```

1. アームを stow（収納）へ
2. アームを上げる
3. アームを stow へ戻す
4. 前方 50cm へナビゲーション

> ★ `nav2.yaml` の `xy_goal_tolerance: 0.12` があるので、
> **50cm ちょうどには止まりません**（モック実測で 0.39m）。

#### 1-2. `image_saver` — 呼ばれた瞬間に手首カメラの画像を保存する

```bash
# 端末 A（コンテナ内）: 常駐させる
ros2 run lekiwi_examples image_saver

# 端末 B（コンテナ内）: 保存したいタイミングで叩く
ros2 service call /image_saver/save std_srvs/srv/Trigger
```

保存先は `captured_images/example_rgb.png` と `example_depth.jpg`
（`docker/robot/compose.yaml` が `/captured_images` にマウントしています）。
depth は 16UC1 [mm] を 0-255 へ正規化したグレースケールです。

cv_bridgeは使えないため、自前でimgmsgからnpへの変換を実装しています。

> ★ depth は `align_depth` の既定が `false` なので **RGB と画角がずれます**。
> 揃えたいなら realsense を `align_depth:=true` で起動し、
> `DEPTH_TOPIC` を `aligned_depth_to_color/image_raw` に書き換えてください。

### 2. リーチ — `map` 上の点へアームを伸ばす

```bash
# コンテナ内
ros2 launch lekiwi_examples reach.launch.py
```

**RViz の "Publish Point" でクリックする**のがいちばん簡単です
（★ Fixed Frame を `map` にすること）。トピックからも与えられます。

```bash
ros2 topic pub --once -w 1 /so101/reach_target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 0.35, y: 0.05, z: 0.25}, orientation: {w: 1.0}}}'
```

結果は `/so101/reach_status` に 1 行ずつ出ます。

```
ACCEPTED   target=map(0.350,0.050,0.250) iters=9 residual=0.0044
SUCCEEDED  residual_fk=0.0045
```

#### どう解いているか

```
/clicked_point (PointStamped) または /so101/reach_target (PoseStamped)
    │
    ▼ ① frame_id が map か確認     → 違えば REJECTED_WRONG_FRAME
    ▼ ② TF で map → arm_base_link  → 引けなければ REJECTED_NO_TF
    ▼ ③ TF の鮮度を見る            → 古ければ REJECTED_STALE_TF
    ▼ ④ 距離の足切り               → 遠すぎれば REJECTED_OUT_OF_RANGE
    ▼ ⑤ ★ オフラインで IK を収束させる（reach_solver.py）
    │      届かなければ REJECTED_UNREACHABLE  ← ここまでアームは動かない
    ▼ ⑥ FollowJointTrajectory を 1 本送る
    ▼ ⑦ 実行中もベースの動きを監視 → 動いたら ABORTED_BASE_MOVED
```

**★ ⑤ が設計の要です。**

`cartesian_math.damped_least_squares` は**速度レベル**のソルバです。
そのまま指令に使うと「動かしてみて届かなかった」になります。

`reach_solver.py` はこれを**指令を出す前にオフラインで反復**します。

```
for i in range(200):
    tip, J = chain.position_and_jacobian(q, controlled)   # 順運動学とヤコビアン
    err = target - tip
    if |err| < 0.005:  return SOLVED
    if 改善が止まった:  return STALLED        # ← 到達不能
    dq = damped_least_squares(J, step, ...)
    q  = arm_target(q, dq, ...)               # 関節上下限でクランプ
```

これで**「届かないなら警告して何もしない」が成立します**。
`arm_target` が関節上下限でクランプするので、ワークスペース外では残差が
改善しなくなります（＝ `STALLED`）。**どの関節が上限に張り付いたかも報告する**ので、
「遠すぎる」のか「ベースを回せば届く」のかが区別できます。

**★ ベースは絶対に動かしません。**

「届かなければベースを動かして近づく」ことはしません。これは
**リーチノードが `/cmd_vel` の publisher を一切作らない**ことで
構造的に保証していて、AST を走査する単体テストで固定してあります
(`test_reach_node_contract.py`)。

**★ 鮮度チェックが要る理由**

`lookup_transform` を最新時刻で引くと、**古い TF でも成功します**。
slam_toolbox が死んでも凍った `map → odom` を返し続けるので、
**黙って過去の世界で解いてしまいます**。返ってきた `header.stamp` を
現在時刻と比べ、`tf_max_age` を超えたら `REJECTED_STALE_TF` にします。

### 3. キーボード操作 — ベースとアームを同時に

```bash
# コンテナ内
ros2 run lekiwi_examples teleop_keyboard
```

**ベース**（★ オムニなので真横にも動けます）

```
  u  i  o        i / ,   前後
  j  k  l        j / l   左右（strafe）
  m  ,  .        u/o/m/. 斜め
                 k       停止
                 [ / ]   その場で旋回
```

**アーム**（上段が +、下段が −。1 キーで 0.05 rad）

```
  1 / q   shoulder_pan      2 / w   shoulder_lift
  3 / e   elbow_flex        4 / r   wrist_flex
  5 / t   wrist_roll        6 / y   gripper（開 / 閉）

  Space   アームを現在姿勢で保持（目標を実測値へ同期し直す）
  ?       ヘルプ
  Ctrl+C  終了
```

### 4. デカルト座標でのジョグ（手先を XYZ で動かす）　** 未テスト **

```bash
# コンテナ内
ros2 launch lekiwi_examples cartesian_teleop.launch.py
```

キー: `w`/`s` = ±x、`a`/`d` = ±y、`r`/`f` = ±z。
逆運動学（DLS）で関節軌道に変換します。**関節ごとに動かしたいときは 3 を**
使ってください。

---

## 中身

| ファイル | 内容 |
| --- | --- |
| `cartesian_math.py` | 順運動学・ヤコビアン・**減衰最小二乗（DLS）**。ROS に依存しない |
| `reach_solver.py` | DLS を**指令の前にオフラインで収束**させる。到達不能の判定 |
| `reach_to_point.py` | リーチのノード。TF の鮮度チェック、ベース移動の監視、`/so101/stow` |
| `example_sequence.py` | **最小構成の例。** アームを動かして前へ進む（一直線に実行して終わる） |
| `image_saver.py` | **最小構成の例。** 手首カメラの画像をサービス呼び出しで保存（常駐） |
| `teleop_keyboard.py` | ベース + アームのキーボード操作 |
| `cartesian_jog.py` + `keyboard_input.py` | デカルト座標のジョグ（2 ノードで 1 組） |

**★ `cartesian_math.py` と `reach_solver.py` は ROS を import しません。**
だから実機もコンテナも無しで単体テストできます。

```bash
# コンテナ内 /ros2_ws/src/
python3 -m pytest lekiwi_examples -q
```

---

## なぜロボット本体と分けているか

`robot.launch.py` は**ロボットを「動かせる状態」にするところまで**を担当し、
その上で何をするかはこちらの責任にしています。

- ロボットを起動しただけでは**アームは動きません**。何を動かすかは明示的に選ぶ
- サンプルを差し替えても**ロボット側は再起動不要**
- ハードウェアに触るコード（`so101_bringup` / `lekiwi_base_bringup`）と、
  その上のロジックが混ざらない

## 既知の未解決事項

- **保持力が弱い。** 全関節 `P=16`（STS3215 の工場出荷値 32 の半分。lerobot が
  振動回避で下げた値）。位置制御のトルクは概ね `P × 位置偏差`で、目標に追い付いた
  関節は偏差ゼロ = トルクゼロになり重力で下がります。
  **直す場所は ROS 側ではなく LeRobot の設定**です
- **電源電圧が定格より低い。** サーボは 7.4V 定格ですが静止時の実測は 4.9V。
  上記の一因である可能性がありますが**未検証**です。
  **★ 8.0V を超えるとサーボが壊れます。** 電源の変更は人間の判断が必要です
- **干渉チェックが一切ありません。** 単一 waypoint なので JTC が関節空間で補間し、
  肘が天板や LiDAR を通り抜ける経路を取りえます
- **`nav2.yaml` の `robot_radius: 0.17` は収納状態の前提です。** 伸ばしたまま走ると
  通れない隙間を計画します。**走行前に stow してください**


## 関連

| 知りたいこと | どこ |
| --- | --- |
| ロボットの起動手順 | [`../../../README.md`](../README.md) |
| 名前と型の一覧・仕組み | [`../../../docs/internals.md`](internals.md) |
| 自分でノードを書く | [`../README.md`](../README.md) |
