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
[`../../../docs/development.md`](../../../docs/development.md)。

---

## 使い方

まず別ターミナルでロボットを起動しておきます。

```bash
# Linux PC（実機）
cd docker/robot && make run BACKEND=lerobot ROBOT_ID=my_follower

# 実機なし（Mac 可）
cd docker/robot && make mock
```

以降はコンテナに入って叩きます。

```bash
cd docker/robot && make shell
```

### 1. 最小構成の例 — まずこれを読む

```bash
# コンテナ内
ros2 run lekiwi_examples example_sequence
```

アーム・ナビゲーション・カメラを**ひととおり 1 ファイルで**触ります。

1. アームを stow（収納）へ
2. アームを上げる
3. アームを stow へ戻す
4. 前方 50cm へナビゲーション
5. 手首カメラの画像を保存

保存先は `captured_images/example_rgb.png` と `example_depth.jpg`
（`docker/robot/compose.yaml` が `/captured_images` にマウントしています）。

> ★ **車輪を浮かせるか、前方 1m を空けてください。** 4 で実際に走ります。
> アームは可動域の内側ですが**干渉チェックはありません**。

読みどころは 3 つです。

| 論点 | 中身 |
| --- | --- |
| **スレッドを使わない** | アクションは `send_goal_async()` + `spin_until_future_complete()` で待ちます。手順が上から下へ一直線に読め、どこで待っているかがコードと一致します |
| **`cv_bridge` を使わない** | numpy 2 で `imgmsg_to_cv2()` が SIGSEGV します（import は通るので気付きにくい）。`imgmsg_to_np()` を自前で持っています → [`docs/development.md`](../../../docs/development.md) |
| **画像は購読しっぱなし** | 最新の 1 枚だけ持ち、変換と保存は最後にまとめてやります。撮りたい瞬間に同期を取らずに済みます |
| **ナビ目標は固定フレームで自分で計算** | `frame_id: base_link` に「前へ 0.5m」と書くと**目標が自分に付いて回り**、Nav2 が収束しません。モック実測で ABORTED（224 秒で 0.123m）。`map` で計算すれば SUCCEEDED |

> ## ★ スレッドを使わないなら、ブロックする API を呼んではいけない
>
> spin していない間は**何も受信しません**。購読も TF バッファの更新も
> spin の中でしか進まないので、次の 2 つは**別スレッドが spin していないと
> 永久に待ちます**。
>
> | 呼んではいけない | 代わりに |
> | --- | --- |
> | `ActionClient.send_goal()`（同期版） | `send_goal_async()` + `spin_until_future_complete()` |
> | `Buffer.lookup_transform(..., timeout=…)` | `can_transform()` になるまで `spin_once()` で回してから、timeout 無しで引く |
>
> `ActionClient.wait_for_server()` だけは例外で、グラフを直接見るので
> spin が要りません。

> ★ `nav2.yaml` の `xy_goal_tolerance: 0.12` があるので、
> **50cm ちょうどには止まりません**（モック実測で 0.39m）。

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

> ★ **届かない目標では警告して何もしません。** ベースは動かしません
> （`/cmd_vel` の publisher を一切作らないことで構造的に保証しています）。
>
> ★ **精度は数 cm です。** 理由は
> [`../../../docs/lekiwi_so101_reach.md`](../../../docs/lekiwi_so101_reach.md)。

### 3. キーボード操作 — ベースとアームを同時に

```bash
# コンテナ内
ros2 run lekiwi_examples teleop_keyboard
```

> ## ★ 先に車輪を浮かせてください
>
> `/cmd_vel` は Nav2 の `collision_monitor` より**下流**なので、
> **衝突監視も加速度制限も効きません。**
>
> アームは可動域の内側でクランプしますが、**機体との干渉は見ていません**。
> LiDAR やプレートに当たりえます。

**ベース**（★ オムニなので真横にも動けます）

```
  u  i  o        i / ,   前後
  j  k  l        j / l   左右（strafe）
  m  ,  .        u/o/m/. 斜め
                 k       停止
                 [ / ]   その場で旋回
```

> ## ★ `teleop_twist_keyboard` とは `j` / `l` の意味が違います
>
> | キー | `teleop_twist_keyboard` | このノード |
> | --- | --- | --- |
> | `j` / `l` | **旋回** | **左右（strafe）** |
> | `J` / `L`（Shift） | 左右（strafe） | 未割当 |
> | `[` / `]` | 未割当 | 旋回 |
>
> このベースはオムニで真横に動けるので、Shift の要らない押しやすいキーを
> strafe に割り当てています。
>
> **`teleop_twist_keyboard` を動かしていると「`j`/`l` で旋回する」ことになりますが、
> それは仕様で、機体の故障ではありません。** 実際に一度取り違えました
> （2026-08-08。車輪の較正を疑って 48 通り総当たりする羽目になりました）。
> `docker/lekiwi_base_ros2/README.md` や `base.launch.py` が
> `teleop_twist_keyboard` を案内しているので、混ざりやすいです。

**アーム**（上段が +、下段が −。1 キーで 0.05 rad）

```
  1 / q   shoulder_pan      2 / w   shoulder_lift
  3 / e   elbow_flex        4 / r   wrist_flex
  5 / t   wrist_roll        6 / y   gripper（開 / 閉）

  Space   アームを現在姿勢で保持（目標を実測値へ同期し直す）
  ?       ヘルプ
  Ctrl+C  終了
```

> ★ **キーを離すとベースは止まります**（`base_driver` の watchdog が 0.5 秒で
> 速度ゼロにする）。**アームは止まらず、その姿勢で保持します。**

> ## ★ アームの目標と実測はずれます
>
> ステータス行には**目標と実測の両方**が出ます。
>
> ```
> arm_shoulder_lift_joint 目標 -0.400 (実測 -0.520) rad
> ```
>
> 保持力が弱い（全関節 `P=16`。[既知の未解決事項](../../../docs/lekiwi_so101_reach.md#既知の未解決事項)）
> ため、目標に追い付いた関節は偏差ゼロ = トルクゼロになり、重力で下がります。
>
> このノードは**自分が送った目標を覚えていて、そこに加算します**。実測値を
> 読み直すと下がった値が次の目標に焼き込まれ、**どのキーを押しても
> shoulder_lift が下がり続けます**（実際にそうなっていました）。
>
> ずれが大きくなったら **Space** で現在姿勢へ同期し直してください。

押しっぱなしにすると、`arm_speed`（既定 0.5 rad/s）で連続して動きます。
キーを離してから止まるまでに最大 `arm_max_lead`（既定 0.15 rad）+ 追従の
遅れぶん動きます（モック実測で +0.195 rad）。

> ★ **キーを押した回数ではなく時間で決まります。** キーのオートリピート速度は
> 端末と OS まかせなので、そこに指令を連動させると機械の挙動が環境で変わって
> しまいます。1 打あたりの `arm_step`（0.05 rad）は行き先を進めるだけで、
> 実際に送る目標は 20Hz のタイマーが `arm_speed` で寄せていきます。

### 4. デカルト座標でのジョグ（手先を XYZ で動かす）

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
| `example_sequence.py` | **最小構成の例。** アーム -> ナビゲーション -> 画像の保存 |
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

## 関連

| 知りたいこと | どこ |
| --- | --- |
| ロボットの起動手順 | [`../../../README.md`](../../../README.md) |
| Topic / Service / Action の一覧 | [`../../../docs/interfaces.md`](../../../docs/interfaces.md) |
| リーチの仕組みと精度 | [`../../../docs/internals.md`](../../../docs/internals.md) |
| 自分でノードを書く | [`../../../docs/development.md`](../../../docs/development.md) |
