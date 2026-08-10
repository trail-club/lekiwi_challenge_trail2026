# Navigation Handoff

This branch is the active Navigation branch for the LeKiwi + SO-101 real robot.

Branch:

```text
dev-navigation
```

Expected files:

```text
docs/navigation.md
ros2_ws/src/lekiwi_examples/lekiwi_examples/navigation.py
ros2_ws/src/lekiwi_examples/lekiwi_examples/navigation_demo.py
ros2_ws/src/lekiwi_examples/lekiwi_examples/navigation_types.py
ros2_ws/src/lekiwi_examples/test/test_navigation_types.py
```

Quick check:

```bash
git branch --show-current
git log --oneline -1
find . -name navigation_demo.py
```

Run after build/bootstrap and after the robot stack is already running:

```bash
make shell
source /app/ros2_ws/install/setup.bash
ros2 run lekiwi_examples navigation_demo --ros-args \
  -p target_x:=0.2 -p target_y:=0.0 -p target_yaw:=0.0
```

Safety notes:

- Confirm `shared` or `split` before any real hardware run.
- Start with `BACKEND=mock` unless the arm calibration and bus mode are confirmed.
- Use a small target first, such as `target_x:=0.2`.
- Keep access to the physical power switch during the first motion test.
