# Navigation担当メモ

## 境界

Navigationは既存のNav2を利用する。アプリ側は`/cmd_vel`をpublishしない。

```
CompetitionNavigator
  └ NavigateToPose (Nav2)
       └ controller_server (MPPI / Omni)
            └ velocity_smoother → collision_monitor → /cmd_vel
                 └ lekiwi_base_driver
```

`lekiwi_examples/navigation.py`の`CompetitionNavigator.navigate_to()`が、Task
Orchestratorから利用する最小APIである。Nav2結果を成功・拒否・失敗・タイムアウトへ
正規化し、既定で合計3回まで試行する。

## object / drop と approach pose

物体・Drop Zoneの座標はランドマークであり、ロボット中心の目標ではない。把持・配置の
ための停止姿勢は別に作る。

```python
from lekiwi_examples.navigation_types import PlanarPose, approach_pose_from_landmark

object_pose = PlanarPose(x=2.0, y=3.0, yaw=0.0)
pick_approach = approach_pose_from_landmark(
    object_pose,
    standoff_m=0.45,
    final_yaw=0.0,
)
# -> (1.55, 3.0, 0.0): +Xを物体へ向け、0.45m手前で停止
```

`standoff_m`と`final_yaw`は、Pick/Place担当がアームの到達性を実測して決める値である。
大会の0.5m採点半径は停止精度ではないので、到達可能なアーム姿勢と衝突余裕で決める。

実機でこの変換込みの移動を試す最小入口は`navigation_approach_demo`である。

```bash
ros2 run lekiwi_examples navigation_approach_demo --ros-args \
  -p landmark_x:=1.0 -p landmark_y:=0.0 -p landmark_yaw:=0.0 \
  -p approach_yaw:=0.0 -p standoff_m:=0.45
```

この例は「ランドマーク `(1.0, 0.0)` の0.45m手前」にapproach poseを作り、Nav2へ送る。
`approach_yaw`が最終停止時のロボット向きである。

## Goal Checker

`lekiwi_base_bringup/config/nav2.yaml`で既に以下が有効である。

```yaml
controller_server:
  general_goal_checker:
    plugin: "nav2_controller::SimpleGoalChecker"
    xy_goal_tolerance: 0.12
    yaw_goal_tolerance: 0.2
```

これは実機の再現性を測るまで変更しない。調整する場合は同ファイルの値を変え、
`make bootstrap`後にlaunchを上げ直す。調整候補は各approach poseについて、最終TF誤差・
Pick成功率・再試行回数を記録して決める。

## モック確認

`make mock-shared`を起動した別ターミナルで、任意姿勢へのAPI呼び出しを確認できる。

その前に`/bt_navigator`と`/planner_server`が`active`であることを確認する。
Action名が見えてもLifecycle Nodeが`inactive`なら、`NavigateToPose`はgoalを拒否する。
`make check`は`planner_server`と`bt_navigator`がともに`active`であることも確認する。

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /planner_server

ros2 run lekiwi_examples navigation_demo --ros-args \
  -p target_x:=1.0 -p target_y:=0.0 -p target_yaw:=0.0
```

長時間起動したモックでLifecycleが`inactive`になった場合は、前面で動かしている
`make mock-shared`を`Ctrl+C`で終了してから、同じコマンドをもう一度起動する。
既存のActionが残っているだけの状態を「Nav2が使える」と判断しない。

## opennav_docking の扱い

`opennav_docking`はDockerイメージに含まれ、`docking_server`と`/dock_robot`
(`nav2_msgs/action/DockRobot`)も起動済みである。`SimpleNonChargingDock`も導入済みで、
非充電の物体・棚への最終接近という用途に概念上は適合する。

ただし精密制御には`/detected_dock_pose`を連続配信する認識器と、カメラ座標からdock座標への
外部補正が必要である。現構成にはその認識器・補正の実測値が無いため、本番経路にはまだ使わない。
先に`NavigateToPose`でPick/Place成功率を測り、必要なときだけ別launch/configでDocking PoCを
追加する。既存の`simple_charging_dock`設定は変更しない。

## Pick/Placeとの次の契約

Task Orchestratorは次の順で`navigate_to()`を呼び、成功時だけ担当間イベントを進める。

```
NAV_TO_OBJECT → ARRIVED_OBJECT → WAIT_PICK
  → NAV_TO_DROP → ARRIVED_DROP → WAIT_PLACE → DONE
```

ROS Action/Service名とPick/Place完了通知のメッセージ型は、Pick/Place担当と合意してから追加する。
今の段階では汎用`String`トピックを勝手に固定しない。
