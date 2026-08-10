# Navigation引き継ぎメモ

このブランチが、LeKiwi + SO-101実機で使うNavigation作業ブランチである。

ブランチ:

```text
dev-navigation
```

入っているはずのファイル:

```text
docs/navigation.md
ros2_ws/src/lekiwi_examples/lekiwi_examples/navigation.py
ros2_ws/src/lekiwi_examples/lekiwi_examples/navigation_demo.py
ros2_ws/src/lekiwi_examples/lekiwi_examples/navigation_types.py
ros2_ws/src/lekiwi_examples/test/test_navigation_types.py
```

確認コマンド:

```bash
git branch --show-current
git log --oneline -1
find . -name navigation_demo.py
```

`make build` / `make bootstrap` 後、さらにロボットスタックが起動済みの状態で実行する:

```bash
make shell
source /app/ros2_ws/install/setup.bash
ros2 run lekiwi_examples navigation_demo --ros-args \
  -p target_x:=0.2 -p target_y:=0.0 -p target_yaw:=0.0
```

安全メモ:

- 実機を動かす前に、必ず`shared`か`split`かを確認する。
- アーム較正とバス構成が確認できるまでは、まず`BACKEND=mock`で起動する。
- 最初は`target_x:=0.2`のような小さい目標で試す。
- 初回の移動テスト中は、すぐ物理電源を切れる状態にしておく。
