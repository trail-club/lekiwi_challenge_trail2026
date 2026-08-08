#!/usr/bin/env bash
# ホストからマウントした ros2_ws の初期化。統合イメージ用。
#
#   cd docker/robot && make bootstrap
#   （実体は docker compose run --rm robot bash /bootstrap.sh）
#
# 上流の取得と colcon build をここに集約する。
# 成果物はホスト側の ros2_ws/build・install に残るので、イメージの
# 作り直しやコンテナの作り直しで消えない。
#
# ★ 旧 docker/so101_ros2/bootstrap.sh との違い
#   ビルド対象を全パッケージへ広げた。以前はアームのイメージに nav2 も
#   laser_geometry も realsense2_camera も入っていなかったので、
#   lekiwi_base_bringup / rplidar_bringup / realsense_bringup を除外していた。
#   統合イメージには全部入っているので除外理由が消えた。
set -eo pipefail

# ROS の setup.bash は未定義変数を参照するので set -u とは併用できない
# (AMENT_TRACE_SETUP_FILES: unbound variable になる)。
source /opt/ros/jazzy/setup.bash

# ── Python の環境 (uv) ────────────────────────────────────────────────
# ★ venv はイメージに焼かない。compose がリポジトリを /app へ bind mount
#   するので、イメージ側に作っても実行時に隠れる（CLAUDE.md の地雷）。
#   ここで作るとホスト側 (<リポジトリ>/.venv) に残り、コンテナを作り直しても
#   イメージを作り直しても消えない。
#
# ★ 初回は約 1.7GB のダウンロードとインストールがある（torch だけで 574MB）。
#   二回目以降は uv が「変更なし」と判断して一瞬で終わる。
#
# ★ 「.venv があるか」で分岐しない。`uv venv` だけ成功して `uv sync` が
#   失敗すると**空の venv が残り**、次回そこを「作成済み」と誤判定して
#   `No module named colcon` で落ちる（実際に踏んだ）。
#   `uv sync` は venv が無ければ自分で作るので、毎回これだけ呼べばよい。
[ -d /app/.venv ] || echo "== Python 環境を作ります (初回のみ。約 1.7GB) =="
cd /app
uv sync --frozen --no-install-project
# ★ colcon を venv の側で動かす。apt の /usr/bin/colcon で建てると
#   console_script の shebang が /usr/bin/python3 になり、`ros2 run` した
#   ノードが venv を見ずに ModuleNotFoundError で落ちる（実測）。
source /app/.venv/bin/activate

cd /ros2_ws

# ★ colcon の作業ディレクトリを先に作っておく。
#   build / install / log は .gitignore 済みなので **clone 直後は存在しない**。
#   colcon は自分で作るが、上流を cp -a した直後 (macOS の virtiofs 越しに
#   16MB / 数千ファイル) に走らせると、log/ の作成が
#     FileNotFoundError: 'log/build_YYYY-MM-DD_hh-mm-ss'
#   で落ちることがあった (実際に一度踏んだ)。単純な colcon build では再現せず
#   タイミング依存なので、先に作って競合の余地を無くす。
mkdir -p build install log

# ── 上流をイメージから配置する ────────────────────────────────────────
# ros2_so_arm (アームの description) と sllidar_ros2 (LiDAR ドライバ)。
# **取得はイメージのビルド時に済んでいる** (docker/robot/Dockerfile が
# /opt/upstream へ vcs import 済み)。ここではコピーするだけで、
# ネットワークを一切使わない。
#
# ★ なぜコピーが要るのか
#   compose.yaml が ../.. を /app へ bind mount するため、
#   イメージが /app（= /ros2_ws の実体）に置いたものは実行時に完全に隠れる。
#   マウントの外 (/opt/upstream) からマウントの中へ運ぶ必要がある。
#
# ★ ホスト側にも実体を置く（symlink にしない）理由
#   ホストのエディタや grep から上流ソースを読めるようにするため。
#   URDF の関節可動域を確認する、といった作業が頻繁に発生する。
UPSTREAM=/opt/upstream
if [ ! -d "$UPSTREAM" ]; then
  echo "ERROR: $UPSTREAM がありません。イメージが古い可能性があります。" >&2
  echo "       make build からやり直してください。" >&2
  exit 1
fi

for repo in "$UPSTREAM"/*/; do
  name="$(basename "$repo")"
  if [ -d "src/$name" ]; then
    continue
  fi
  echo "== 上流を配置します: $name =="
  cp -a "$repo" "src/$name"
done

# ★ 何が入っているかを毎回表示する。イメージを作り直さずに src/ だけ残っていると
#   古い上流を使い続けることになるので、SHA を目に見えるところへ出す。
echo "== 上流のバージョン =="
for repo in "$UPSTREAM"/*/; do
  name="$(basename "$repo")"
  image_sha="$(git -C "$repo" rev-parse --short HEAD 2>/dev/null || echo '?')"
  ws_sha="$(git -C "src/$name" rev-parse --short HEAD 2>/dev/null || echo '?')"
  if [ "$image_sha" = "$ws_sha" ]; then
    echo "   $name: $ws_sha"
  else
    echo "   ★ $name: ワークスペース $ws_sha / イメージ $image_sha （食い違っています）"
    echo "     イメージのものを使うなら: rm -rf src/$name してから再実行"
  fi
done

# CMakeCache.txt は CMake の絶対パスを保持する。
#
# 以前のイメージでビルドした成果物をホスト側に残したまま UV/venv の
# イメージへ切り替えると、キャッシュ内の CMAKE_COMMAND が例えば
# /usr/local/lib/python3.12/dist-packages/cmake/data/bin/cmake のような、
# 現在のコンテナに存在しないパスを指すことがある。すると colcon が
# 再ビルド時にその古いパスを Makefile から呼び出して停止する。
# そのパッケージだけ build/install を捨てて、現在の cmake で生成し直す。
while IFS= read -r -d '' cache; do
  pkg_build="${cache%/CMakeCache.txt}"
  pkg="$(basename "$pkg_build")"
  cmake_command="$(sed -n 's/^CMAKE_COMMAND:INTERNAL=//p' "$cache" | head -n 1)"
  if [[ -n "$cmake_command" && ! -x "$cmake_command" ]]; then
    echo
    echo "== $pkg の古い CMake キャッシュを作り直します =="
    echo "   使用できない CMAKE_COMMAND: $cmake_command"
    rm -rf "$pkg_build" "install/$pkg"
  fi
done < <(find build -mindepth 2 -maxdepth 2 -name CMakeCache.txt -print0)

# --symlink-install の成果物には src を指すシンボリックリンクが残る。
# データファイルを消したりブランチを切り替えたりすると、リンク先が消えて
# 「壊れたリンク」になり、setuptools が存在しないファイルをコピーしようとして
#   error: can't copy '...': doesn't exist or not a regular file
# で失敗する。該当パッケージの生成物だけを消して作り直す。
#
# ★ ブランチ切り替えで頻繁に起きる (例: feat/env-camera にしか無い
#   env_camera.launch.py のリンクが build に残ったまま別ブランチへ戻る)。
#   パッケージ名を決め打ちせず、壊れたリンクを持つものを全部拾う。
for pkg_build in build/*/; do
  pkg="$(basename "$pkg_build")"
  if find "$pkg_build" -xtype l -print -quit 2>/dev/null | grep -q .; then
    echo
    echo "== $pkg に壊れたシンボリックリンクがあるので作り直します =="
    find "$pkg_build" -xtype l -printf '   %p -> %l\n' 2>/dev/null | head -5
    rm -rf "build/$pkg" "install/$pkg"
  fi
done

echo
echo "== ビルドします =="
# ★ --packages-ignore を使う（--packages-select ではない）。
#   select だと自作パッケージを src/ に置いてもビルド対象に入らず、
#   「作ったのに ros2 run で見つからない」という分かりにくい詰まり方をする。
#   ignore なら **src/ に置いたものは自動でビルドされる**。
#
#   除外するのは上流 ros2_so_arm に同梱されている SO-ARM100 系だけ。
#   so_arm_gz は Gazebo、so_arm100_moveit_config は MoveIt を要求するが、
#   どちらもこのイメージに入っていない (この機体は SO-101 なので使わない)。
# ★ `colcon` ではなく `python3 -m colcon` で呼ぶこと。
#   PATH 上の colcon が apt 版 (/usr/bin/colcon) だと sys.executable が
#   /usr/bin/python3 になり、console_script の shebang がそこを指す。
#   すると `ros2 run` したノードが venv を見ず、scservo_sdk や lerobot が
#   import できずに落ちる (実測)。python3 は venv のものなので、
#   -m で呼べば shebang は /app/.venv/bin/python3 になる。
python3 -m colcon build --symlink-install \
  --packages-ignore so_arm_gz so_arm100_description so_arm100_moveit_config

# ★ Dockerfile にあった静的スモークテストの移設先。
#   ワークスペースをマウントする方式ではビルド時に検査できないので、
#   ここで毎回走らせる。壊れたまま実機に持っていくのを防ぐのが目的。
echo
echo "== 静的検査 =="
source install/setup.bash

# so101_bringup はハードウェアに触る側だけ (ブリッジと較正)
python3 -c "from so101_bringup import bridge_core, calibration_limits, \
    lerobot_backend, lerobot_bridge"
# lekiwi_examples はその上で動くアプリケーション側 (逆運動学・リーチ・操作)
python3 -c "from lekiwi_examples import cartesian_jog, cartesian_math, keyboard_input, \
    reach_solver, reach_to_point, teleop_keyboard"
python3 -c "import lerobot; from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig"
# ★ ベース側。統合により numpy が dpkg 版から pip の 2.2.6 に変わったので、
#   import が通ることをここで毎回確かめる。
python3 -c "import scservo_sdk, serial, numpy"
python3 -c "from lekiwi_base_bringup import base_driver, kinematics, sts_bus"
python3 -c "from lekiwi_so101_bringup import release_all"
echo "  Python import: OK"

# ★ 統合イメージにだけ必要な検査。以前は「アームのイメージに nav2 が無い」
#   ことが原因で /navigate_to_pose を叩くと action type invalid になっていた。
#   1 コンテナ化の目的そのものなので、ここで実在を確かめる。
python3 -c "import nav2_msgs.action, slam_toolbox.srv"
ros2 pkg prefix nav2_bringup > /dev/null
ros2 pkg prefix slam_toolbox > /dev/null
ros2 pkg prefix realsense2_camera > /dev/null
ros2 pkg prefix sllidar_ros2 > /dev/null
echo "  ナビ・LiDAR・カメラのパッケージ: OK"

# アーム単体の URDF
D="$(ros2 pkg prefix so_arm101_description)/share/so_arm101_description"
B="$(ros2 pkg prefix so101_bringup)/share/so101_bringup"
xacro "$D/urdf/so_arm101.urdf.xacro" \
  ros2_control_hardware_type:=real \
  ros2_control_file:="$B/control/so101_follower.ros2_control.xacro" > /tmp/arm.urdf
check_urdf /tmp/arm.urdf > /dev/null
grep -q 'joint_state_topic_hardware_interface/JointStateTopicSystem' /tmp/arm.urdf
echo "  アーム単体 URDF: OK"

# ベース単体の URDF (lekiwi_base_bringup の nav.launch.py が使う)
L="$(ros2 pkg prefix lekiwi_description)/share/lekiwi_description"
xacro "$L/urdf/lekiwi_base.urdf.xacro" use_mesh:=false > /dev/null
echo "  ベース単体 URDF: OK"

# 結合 URDF。ルートが base_footprint 1 つで、リンク名が衝突せず、
# controllers 設定の関節名が実在することを確認する。
C="$(ros2 pkg prefix lekiwi_so101_bringup)/share/lekiwi_so101_bringup"
xacro "$C/urdf/lekiwi_so101.urdf.xacro" > /tmp/combined.urdf
check_urdf /tmp/combined.urdf | grep -q '^root Link: base_footprint'
python3 - "$C/config/ros2_controllers.yaml" <<'PY'
import sys, yaml, xml.etree.ElementTree as ET
root = ET.parse('/tmp/combined.urdf').getroot()
links = [e.get('name') for e in root.findall('link')]
dup = sorted({n for n in links if links.count(n) > 1})
if dup:
    sys.exit(f'リンク名が重複: {dup}')
joints = {e.get('name') for e in root.findall('joint')}
cfg = yaml.safe_load(open(sys.argv[1]))
want = set(cfg['joint_trajectory_controller']['ros__parameters']['joints']) \
       | {cfg['gripper_controller']['ros__parameters']['joint']}
if want - joints:
    sys.exit(f'controllers が存在しない関節を参照: {sorted(want - joints)}')
need = {'arm_base_link', 'arm_gripper_frame_link', 'base_footprint', 'laser_link'}
if not need <= set(links):
    sys.exit(f'必要なリンクが無い: {sorted(need - set(links))}')
# ★ 手首カメラ: <name>_link を子とする joint が**ちょうど 1 個**であること。
#   realsense2_camera は <name>_link を子にする TF を出さない前提なので、
#   URDF 側が 1 個だけ親を与えるのが正しい。0 個なら孤立フレームになり
#   map -> 点群 が解けず、2 個なら tf2 が後着勝ちで非決定になる。
cam = [j for j in root.findall('joint')
       if j.find('child').get('link') == 'wrist_camera_link']
if 'wrist_camera_link' in links and len(cam) != 1:
    sys.exit(f'wrist_camera_link を子とする joint が {len(cam)} 個 (1 であること)')
PY
echo "  結合 URDF: OK"

# ★ launch ファイルが読み込めること。--show-args は generate_launch_description() を
#   実行して引数を列挙するだけで、ノードは 1 つも起動しない。
#   include 先のパッケージが見つからない・引数名を打ち間違えた、といった
#   「起動して初めて分かる」種類の壊れをここで拾う。
ros2 launch lekiwi_so101_bringup robot.launch.py --show-args > /dev/null
ros2 launch lekiwi_so101_bringup arm.launch.py --show-args > /dev/null
ros2 launch lekiwi_examples reach.launch.py --show-args > /dev/null
ros2 launch lekiwi_examples cartesian_teleop.launch.py --show-args > /dev/null
echo "  launch の読み込み: OK"

echo
echo "== 完了 =="
echo "以降は次で起動できます:"
echo "  docker compose up -d"
echo "  docker compose exec -it robot bash"
echo "  ros2 launch lekiwi_so101_bringup robot.launch.py backend:=lerobot robot_id:=my_follower"
echo
echo "設定や Python コードを編集したら launch を上げ直すだけで反映されます"
echo "(--symlink-install なので colcon build も要りません)。"
echo
echo "ファイルやパッケージを**追加**した場合だけ、コンテナ内で:"
echo "  colcon build --symlink-install --packages-select <パッケージ名>"
echo "  source install/setup.bash"
