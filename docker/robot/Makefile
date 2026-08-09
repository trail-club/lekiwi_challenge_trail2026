# LeKiwi + SO-101 統合構成 (1 イメージ 1 コンテナ) の操作窓口。
#
# 使い方:
#   make build       イメージをビルド
#   make bootstrap   ★ 初回とパッケージ追加時。上流取得 + colcon build + 静的検査
#   make install-udev BUS_MODE=shared|split  .env からホストudevルールを生成
#   make up-split / make up-shared  コンテナだけ起動
#   make run-split SO101_ROBOT_ID=my_follower
#   make run-shared LEKIWI_ROBOT_ID=my_lekiwi
#   make mock-split / make mock-shared
#   make shell       コンテナに入る
#   make stow        reach.launch.py起動中にアームを低く畳む
#   make save-map    現在の地図を保存
#   make check       ROS グラフの不変条件を確認
#   make release-check BUS_MODE=shared ★ 読むだけ
#   make release BUS_MODE=shared       ★ 異常終了後に全解放
#   make down        コンテナを停止・削除
#
# ■ 停止
#   通常  : アームを安全な低い姿勢にしてからrun端末でCtrl+C
#   非常時: **物理スイッチを切る。** docker kill は使わない
#           (SIGKILL では停止処理が走らず、ホイールは最後の指令速度で回り続ける)
#   異常終了からの復帰: launchを止めて make release BUS_MODE=split|shared
#                         (コンテナは停止しなくてよい)

-include .env

MAP_NAME   ?= my_room
SVC        := robot
SVC_MOCK   := robot-mock
E          := /entrypoint.sh
BACKEND    ?= lerobot
SO101_ROBOT_ID ?= my_follower
LEKIWI_ROBOT_ID ?= my_lekiwi
START_RVIZ ?= true
MOCK_START_RVIZ ?= false

# ★ `-include .env` は make 変数を作るだけで、レシピのシェルへは export されない。
#   レシピ内で `$$LEKIWI_DEVICE` と書いても**必ず空**になる (以前これで
#   .env の設定が黙って無視されていた)。make 変数として `?=` で既定値を与え、
#   `$(...)` で参照すること。
#   compose.yaml 側の `${LEKIWI_DEVICE:-...}` は docker compose が .env を
#   自分で読むので別経路。
LEKIWI_DEVICE  ?= /dev/lekiwi
SO101_DEVICE   ?= /dev/so101_follower
RPLIDAR_DEVICE ?= /dev/rplidar
MAP_DIR        ?= $(HOME)/maps

# USBシリアルは機体固有なので追跡ファイルへ書かない。.env からudev生成処理へ渡す。
LEKIWI_SERIAL ?=
SO101_SERIAL  ?=
export LEKIWI_SERIAL SO101_SERIAL

# ★ 別スタックのコンテナが同じ ROS_DOMAIN_ID に居ると /robot_description が
#   二重に latch され、/tf も混信する (CLAUDE.md の「RViz に別のロボットが出る」)。
#   統合したのでスタック内には 1 コンテナしか無い。止めたいのは他スタックのもの。
#
#   単体デバッグ用に残してあるスタックのコンテナ:
LEGACY := so101-follower lekiwi-base lekiwi-base-dryrun rplidar-a1 realsense-d435i
#   ★ 削除済みの 4 コンテナ構成 (docker/lekiwi_bringup, lekiwi_so101_bringup)。
#     compose ファイルはもう無いが、**実機に古いコンテナが残っている可能性がある**
#     ので検出は続ける。しばらく経って実機から消えたら外してよい。
LEGACY += lekiwi-nav lekiwi-nav-mock lekiwi-so101-arm lekiwi-so101-arm-mock \
          lekiwi-wrist-camera
CONFLICTS_FOR_REAL := $(LEGACY) $(SVC_MOCK)
CONFLICTS_FOR_MOCK := $(LEGACY) $(SVC)

.PHONY: build bootstrap install-udev udev-dry-run up-split up-shared run-split run-shared \
        mock-split mock-shared shell stow save-map check release \
        release-wheels release-check down logs guard guard-mock require-bus-mode \
        preflight-shared-devices preflight-split-devices

build:
	docker compose -f compose.yaml build

# ★ ros2_ws をホストからマウントする方式なので、イメージをビルドしただけでは
#   install/ が空で launch できない。初回と、パッケージを追加したときに実行する。
#   成果物はホストの ros2_ws/build・install に残る。
#   --no-deps は不要 (サービスが 1 つしか無い)。デバイスを掴まない mock 側で走らせる。
bootstrap:
	docker compose -f compose.mock.yaml run --rm $(SVC_MOCK) bash /bootstrap.sh

# 追跡対象の.rulesはテンプレート。機体固有値を.envから埋めてホストへ入れる。
# dry-runはsudoも書き込みも行わず、生成されるルールだけを表示する。
install-udev: require-bus-mode
	bash ./install_udev.sh $(BUS_MODE)

udev-dry-run: require-bus-mode
	bash ./install_udev.sh --dry-run $(BUS_MODE)

# $(1) に渡した名前のコンテナが動いていたら止めさせる。
define guard_against
	@for name in $(1); do \
	  cids=$$(docker ps -aq -f name=^$$name$$) || { echo "ERROR: Docker状態を確認できない。" >&2; exit 1; }; \
	  for cid in $$cids; do \
	    state=$$(docker inspect -f '{{.State.Status}}' $$cid) || { echo "ERROR: $$name の状態を確認できない。" >&2; exit 1; }; \
	    case "$$state" in \
	      exited|dead|'') ;; \
	      *) \
	        echo "ERROR: $$name が $$state 状態。ROS_DOMAIN_ID が衝突して" >&2; \
	        echo "       /robot_description が二重に latch され、/tf も混信する。" >&2; \
	        echo "       先に停止すること: docker stop $$name" >&2; \
	        exit 1; \
	        ;; \
	    esac; \
	  done; \
	done
endef

guard:
	$(call guard_against,$(CONFLICTS_FOR_REAL))

guard-mock:
	$(call guard_against,$(CONFLICTS_FOR_MOCK))

preflight-shared-devices:
	@test -c "$(LEKIWI_DEVICE)" || { \
	  echo "ERROR: $(LEKIWI_DEVICE) が文字デバイスではない。make install-udev BUS_MODE=shared とUSB接続を確認すること" >&2; exit 1; }
	@test -c "$(RPLIDAR_DEVICE)" || { \
	  echo "ERROR: $(RPLIDAR_DEVICE) が文字デバイスではない。udevとLiDAR接続を確認すること" >&2; exit 1; }

preflight-split-devices: preflight-shared-devices
	@test -c "$(SO101_DEVICE)" || { \
	  echo "ERROR: $(SO101_DEVICE) が文字デバイスではない。make install-udev BUS_MODE=split とUSB接続を確認すること" >&2; exit 1; }

up-split: guard preflight-split-devices
	@test -f .env || { echo "ERROR: .env が無い。cp .env.example .env して編集すること" >&2; exit 1; }
	mkdir -p $(MAP_DIR)
	docker compose -f compose.yaml -f compose.split.yaml up -d

up-shared: guard preflight-shared-devices
	@test -f .env || { echo "ERROR: .env が無い。cp .env.example .env して編集すること" >&2; exit 1; }
	mkdir -p $(MAP_DIR)
	docker compose -f compose.yaml -f compose.shared.yaml up -d

# ★ 前面で走らせること。Ctrl+C が launch に届いてはじめて停止処理
#   (アームのトルク OFF + ホイールの速度ゼロ) が走る。
#   docker compose down では届かない。
run-split: up-split
	docker compose -f compose.yaml exec -it $(SVC) $(E) \
		ros2 launch lekiwi_so101_bringup robot.launch.py \
		motor_bus_mode:=split backend:=$(BACKEND) \
		robot_id:=$(SO101_ROBOT_ID) start_rviz:=$(START_RVIZ) \
		lekiwi_port:=$(LEKIWI_DEVICE) \
		usb_port:=$(SO101_DEVICE) \
		lidar_port:=$(RPLIDAR_DEVICE)

run-shared: up-shared
	docker compose -f compose.yaml exec -it $(SVC) $(E) \
		ros2 launch lekiwi_so101_bringup robot.launch.py \
		motor_bus_mode:=shared backend:=$(BACKEND) \
		robot_id:=$(LEKIWI_ROBOT_ID) start_rviz:=$(START_RVIZ) \
		lekiwi_port:=$(LEKIWI_DEVICE) lidar_port:=$(RPLIDAR_DEVICE)

mock-split: guard-mock
	docker compose -f compose.mock.yaml up -d
	docker compose -f compose.mock.yaml exec -it $(SVC_MOCK) $(E) \
		ros2 launch lekiwi_so101_bringup robot.launch.py \
		motor_bus_mode:=split sim:=true backend:=mock \
		start_rviz:=$(MOCK_START_RVIZ) mock_wrist_camera_optical:=true

mock-shared: guard-mock
	docker compose -f compose.mock.yaml up -d
	docker compose -f compose.mock.yaml exec -it $(SVC_MOCK) $(E) \
		ros2 launch lekiwi_so101_bringup robot.launch.py \
		motor_bus_mode:=shared sim:=true backend:=mock \
		start_rviz:=$(MOCK_START_RVIZ) \
		mock_wrist_camera_optical:=true

shell:
	docker compose -f compose.yaml exec -it $(SVC) bash

# ★ アームを低く畳む。launch を止める前に必ず実行すること
#   (正常終了でトルクが切れてアームが落ちるため)。
stow:
	docker compose -f compose.yaml exec $(SVC) $(E) bash -lc '\
		if ! ros2 service list | grep -qx /so101/stow; then \
		  echo "ERROR: /so101/stow が無い。lekiwi_examples の reach.launch.py を起動するか、アームを別の安全な方法で低くしてから停止すること" >&2; \
		  exit 1; \
		fi; \
		ros2 service call /so101/stow std_srvs/srv/Trigger "{}"'

save-map:
	docker compose -f compose.yaml exec $(SVC) $(E) \
		ros2 run lekiwi_base_bringup save_map $(MAP_NAME)

# TF を 1 行表示する。$(1)=サービス, $(2)=親, $(3)=子, $(4)=出なかったときの助言。
#
# ★ `... | head -1 || echo fallback` は**絶対に発火しない**。パイプラインの
#   終了ステータスは head のもので、入力が空でも 0 になるため。その結果
#   「TF が無い」と「検査が空振りした」を区別できなかった。
#   変数に取ってから中身の有無で分岐する。
# ★ timeout は 15 秒。tf2_echo は discovery が終わるまで何も出さないので、
#   短いと TF はあるのに空になりうる。
define echo_tf
out=$$(docker exec $(1) $(E) timeout 15 ros2 run tf2_ros tf2_echo $(2) $(3) 2>/dev/null | grep -a Translation | head -1); if [ -n "$$out" ]; then echo "$$out"; else echo "  ★ $(2) -> $(3) が引けない ($(4))"; fi
endef

# ROS グラフの不変条件。モックでも実機でも同じ結果になること。
#   robot_description の publisher が 1 でなければ RViz に別のロボットが出る。
#   joint_states の publisher は 2 (ベースの車輪 3 関節 + アームの 6 関節)。
#
# ★ 起動直後は DDS の discovery が終わっておらず、publisher 数が実際より
#   少なく出る。数が合うまで最大 60 秒待ってから表示する。
check:
	@C=$$(docker ps -q -f name=^$(SVC)$$ -f name=^$(SVC_MOCK)$$ | head -1); \
	test -n "$$C" || { echo "コンテナが動いていない" >&2; exit 1; }; \
	echo "discovery を待っています..."; \
	for i in $$(seq 1 30); do \
	  n=$$(docker exec $$C $(E) ros2 topic info /joint_states 2>/dev/null \
	       | sed -n 's/.*Publisher count: //p'); \
	  [ "$$n" = "2" ] && break; sleep 2; \
	done; \
	echo "--- /robot_description (期待: 1) ---"; \
	docker exec $$C $(E) ros2 topic info /robot_description | grep 'Publisher count'; \
	echo "--- /joint_states (期待: 2) ---"; \
	docker exec $$C $(E) ros2 topic info /joint_states | grep 'Publisher count'; \
	echo "--- controllers (期待: active 3 つ) ---"; \
	docker exec $$C $(E) ros2 control list_controllers; \
	echo "--- ★ nav2 と ros2_control の action が同時に見えること ---"; \
	docker exec $$C $(E) ros2 action list | grep -E 'navigate_to_pose|follow_joint_trajectory' || \
	  echo "  ★ 片方しか見えない。分割時の症状が残っている"; \
	echo "--- map -> wrist_camera_depth_optical_frame ---"; \
	$(call echo_tf,$$C,map,wrist_camera_depth_optical_frame,realsense が未起動か camera_name 不一致); \
	echo "--- map -> arm_gripper_frame_link ---"; \
	$(call echo_tf,$$C,map,arm_gripper_frame_link,RSP かコントローラが未起動)

# ★★ 復帰系は「コンテナが生きていてもいなくても、同じ 1 コマンドで通る」ようにする。
#
#   いちばん多い故障は「**launch だけが落ちて、コンテナは生きている**」。
#   このときシリアルポートは空いているので、**コンテナを落とす必要はない**。
#   コンテナが生きていれば exec で入り、死んでいれば使い捨てコンテナを立てる。
#
#   ★ バスを触ってよいかの判定は release_all 自身が持っている。
#     /proc を見て「そのポートを開いているプロセスが居るか」を**ポートごとに**
#     確認し、居ればそのプロセス名を出して中止する。
#     以前ここに「コンテナが動いていたら中止」という guard を置いていたが、
#     **launch が落ちてコンテナだけ生きている場合を弾いてしまう**ので廃止した。
define release_all_cmd
	$(if $(filter shared,$(BUS_MODE)),$(eval RELEASE_COMPOSE=compose.shared.yaml),$(eval RELEASE_COMPOSE=compose.split.yaml))
	@if [ -n "$$(docker ps -q -f name=^$(SVC)$$)" ]; then \
	  echo "コンテナ $(SVC) は動いています。その中で実行します。"; \
	  docker compose -f compose.yaml exec $(1) $(SVC) $(E) \
	    ros2 run lekiwi_so101_bringup release_all --bus-mode $(BUS_MODE) \
	      --shared-port $(LEKIWI_DEVICE) --wheel-port $(LEKIWI_DEVICE) \
	      --arm-port $(SO101_DEVICE) $(2); \
	else \
	  echo "コンテナ $(SVC) は動いていません。使い捨てコンテナで実行します。"; \
	  docker compose -f compose.yaml -f $(RELEASE_COMPOSE) run --rm $(1) $(SVC) $(E) \
	    ros2 run lekiwi_so101_bringup release_all --bus-mode $(BUS_MODE) \
	      --shared-port $(LEKIWI_DEVICE) --wheel-port $(LEKIWI_DEVICE) \
	      --arm-port $(SO101_DEVICE) $(2); \
	fi
endef

require-bus-mode:
	@test "$(BUS_MODE)" = split -o "$(BUS_MODE)" = shared || { \
	  echo "ERROR: BUS_MODE=split または BUS_MODE=shared を必ず指定すること" >&2; \
	  exit 2; \
	}

# ★ 異常終了からの復帰。**アームもホイールもこれ 1 つ**で解放する。
#   ROS は使わずシリアルポートを直接開く。
#   ★ アームは支えが無ければ落ちる。人が支えてから実行すること。
#   ★ コンテナは落とさなくてよい。launch さえ止まっていれば通る。
release: require-bus-mode
	$(call release_all_cmd,-it,)

# ホイールだけ止める (アームは落ちない)。走り出したときの一次対応。
release-wheels: require-bus-mode
	$(call release_all_cmd,,--only wheels --yes)

# ★ 読むだけ。何も書き込まないのでアームは落ちない。
#   「いまトルクが入っているのか」の確認。release の前後で比べる。
#   ★ exit 1 は「トルクが入っている / 読めない」の意味で、異常とは限らない。
#     make は `Error 1` と表示するが、解放前に叩けばそれが期待どおり。
release-check: require-bus-mode
	$(call release_all_cmd,,--dry-run)

down:
	docker compose -f compose.yaml down
	docker compose -f compose.mock.yaml down

logs:
	docker compose -f compose.yaml logs -f
