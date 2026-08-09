# リポジトリ直下の操作窓口。**実体は docker/robot/Makefile**。
#
#   make build       イメージをビルド
#   make bootstrap   ★ 初回とパッケージ追加時。上流取得 + colcon build + 静的検査
#   make up          コンテナを**バックグラウンドで**起動する (compose up -d)。
#                    bash が上がるだけで、ROS はまだ何も動いていない
#   make shell       コンテナに入る
#   make release     ★ 異常終了後にホイールとアームを解放する
#   make down        コンテナを停止・削除
#
#   make help        委譲先の全ターゲットを表示する
#
# ■ ここに処理を書かないこと
#   docker/robot/Makefile には実際に踏んだ問題への対処が入っている。
#     * guard      … 別スタックのコンテナが生きていると ROS_DOMAIN_ID が衝突し、
#                    /robot_description が二重に latch されて /tf も混信する
#     * .env 検査  … 無いまま up するとデバイスのパスが既定値で動いてしまう
#     * release    … コンテナが生きていても死んでいても同じ 1 コマンドで通す分岐
#   ここへ書き写すと二重管理になり、必ず片方が古くなる。**丸ごと委譲する。**
#
# ■ 停止 (詳細は docker/robot/README.md)
#   通常  : make run の端末で Ctrl+C。その前に make stow
#   非常時: **物理スイッチを切る。** docker kill は使わない
#           (SIGKILL では停止処理が走らず、ホイールは最後の指令速度で回り続ける)
#   異常終了からの復帰: make release

# ★ 相対パスで書かないこと。`make -f /path/to/trail_SO101/Makefile build` のように
#   別のディレクトリから呼ばれても壊れないよう、この Makefile 自身の場所から解く。
ROBOT_DIR := $(dir $(realpath $(firstword $(MAKEFILE_LIST))))docker/robot

# docker/robot/Makefile が持つターゲットを**全部**そのまま通す。
#
# ★ 一覧を手で書き写さない。あちらにターゲットを足したときにここへ写し忘れ、
#   「docker/robot では動くのに直下からは動かない」というズレが起きる。
#   ファイルから抜き出せば、足した瞬間から直下でも使える。
#
# ★ ワイルドカード (%:) にしないのは、打ち間違えたターゲットが黙って
#   委譲先へ流れないようにするため。実在するものだけを列挙するので、
#   `make buld` はここで止まる。
FORWARD := $(shell grep -oE '^[a-z][a-z0-9_-]*:' $(ROBOT_DIR)/Makefile 2>/dev/null \
                   | tr -d ':' | sort -u)

ifeq ($(strip $(FORWARD)),)
$(error $(ROBOT_DIR)/Makefile が読めません。リポジトリの構成を確認してください)
endif

.PHONY: help $(FORWARD)

# ★ 既定のターゲットは help。bare の `make` で何かが動き出さないようにする
#   (docker/robot/Makefile の既定は build なので、そこだけ挙動が違う)。
help:
	@echo 'リポジトリ直下の操作窓口。実体は docker/robot/Makefile。'
	@echo ''
	@echo '  make build       イメージをビルド'
	@echo '  make bootstrap   ★ 初回とパッケージ追加時 (上流取得 + colcon build)'
	@echo '  make up          コンテナをバックグラウンドで起動 (compose up -d)'
	@echo '  make run         ★ 実機で launch を起動する (前面。Ctrl+C で止める)'
	@echo '                   ★ up と違い前面。Ctrl+C が届かないと停止処理が走らない'
	@echo '  make mock        実機に触れないモック構成で起動 (Mac 可)'
	@echo '  make shell       コンテナに入る'
	@echo '  make stow        アームを低く畳む (★ 停止前に必ず実行する)'
	@echo '  make check       ROS グラフの不変条件を確認'
	@echo '  make save-map    現在の地図を保存'
	@echo '  make release     ★ 異常終了後にホイールとアームを解放する'
	@echo '  make release-check  トルクが入っているかを読むだけ (何も書かない)'
	@echo '  make down        コンテナを停止・削除'
	@echo '  make logs        ログを追う'
	@echo ''
	@echo '使えるターゲット (docker/robot/Makefile から取得):'
	@echo '  $(FORWARD)'
	@echo ''
	@echo '★ 非常停止は物理スイッチだけ。docker kill は使わないこと。'

# コマンドラインで渡した変数 (make run ROBOT_ID=foo) は MAKEFLAGS 経由で
# 委譲先にもそのまま届く。
#
# ★ --no-print-directory を付ける。付けないと 1 コマンドごとに
#   "Entering/Leaving directory" が 2 行出て、release や check の
#   本当に読みたい出力が埋もれる。
$(FORWARD):
	@$(MAKE) --no-print-directory -C $(ROBOT_DIR) $@
