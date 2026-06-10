# P4.5 Nav2 Controller API Notes

## 1. 目标

P4.5 的目标是把前面 P4.1 到 P4.4 的 MPC 数据流稳定接到 Nav2 controller 插件接口：

```cpp
setPlan()
computeVelocityCommands()
setSpeedLimit()
```

验收重点不是继续搬 ORU 的 `qpOASES`，而是确认同一个 `ForkliftMpcController` 能被 Nav2 的两条入口调用：

- 手动 `/follow_path`
- `NavigateToPose` 经过 global planner 输出的 path

## 2. 当前接口行为

`ForkliftMpcController::setPlan()`：

- 保存 Nav2 下发的 `nav_msgs/Path`
- 同步构造内部 `MpcTrajectory`

`ForkliftMpcController::computeVelocityCommands()`：

- 把 global plan 转到当前 pose frame
- 构造 `MpcTrajectory`
- 从当前 `MpcState(x, y, theta, phi)` 截取 preview window
- 调用最小 MPC solver 生成 `v + steering_rate`
- 转成 `v + steering_angle`
- 用现有 footprint collision/path scoring 验证
- 返回 Nav2 必须的 `geometry_msgs/msg/TwistStamped`
- 当 `publish_control_cmd=true` 时，同步发布 `/forklift/control_cmd`

`ForkliftMpcController::setSpeedLimit()`：

- 支持 Nav2 绝对限速和百分比限速
- 实际输出仍会被 `ForkliftVehicleModel` 的速度上限夹紧

## 3. 输出链路

仿真 bridge 模式：

```text
Nav2 controller_server
  -> computeVelocityCommands()
  -> Nav2 internal cmd_vel chain

ForkliftMpcController
  -> /forklift/control_cmd
  -> sim_command_bridge
  -> /forklift/sim_cmd_vel
  -> Gazebo diff_drive
  -> /odom + TF
```

注意：

- bridge 模式下 Gazebo 订阅 `/forklift/sim_cmd_vel`。
- `sim_command_bridge` 不发布 `/odom` 或 TF。
- `/odom` 和 TF 仍由 Gazebo 输出。

真车模式：

```text
ForkliftMpcController
  -> /forklift/control_cmd
  -> curtis_vehicle_interface
  -> Curtis CAN command

Curtis CAN feedback
  -> curtis_vehicle_interface
  -> /odom + TF + /forklift/vehicle_state + /forklift/fault_state
```

## 4. 启动命令

所有 ROS 终端先执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

构建：

```bash
colcon build \
  --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

单测：

```bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select forklift_nav2_plugins --event-handlers console_cohesion+
```

headless bridge 链路：

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/install/forklift_nav2_demo/share/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_command_bridge:=true \
  gazebo_gui:=false \
  use_rviz:=false
```

AMCL 初始位姿：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"
```

## 5. 验收结果

日期：2026-06-10

构建：

```text
Summary: 4 packages finished
```

单测：

```text
test_forklift_mpc_types: 6 passed
test_forklift_mpc_trajectory: 7 passed
test_forklift_mpc_preview_window: 5 passed
test_forklift_mpc_solver: 5 passed
Total: 23 passed
```

FollowPath 短直线：

```text
/follow_path accepted: True
/follow_path status: 4 SUCCEEDED
/forklift/control_cmd samples: 21
/forklift/control_cmd max velocity_mps: 0.3942857142857143
/forklift/control_cmd last: brake=True forward=False reverse=False velocity_mps=0.0 steering_angle_rad=0.0
/forklift/sim_cmd_vel samples: 50
/forklift/sim_cmd_vel max linear.x: 0.3942857142857143
/odom final x=-1.5112646005374553 y=-0.5033982399458335
```

NavigateToPose：

```text
/navigate_to_pose accepted: True
/navigate_to_pose status: 4 SUCCEEDED
/navigate_to_pose feedback count: 376
/forklift/control_cmd samples: 38
/forklift/control_cmd max velocity_mps: 0.32142857142857145
/forklift/control_cmd last: brake=True forward=False reverse=False velocity_mps=0.0 steering_angle_rad=-0.41000000000000003
/forklift/sim_cmd_vel samples: 85
/forklift/sim_cmd_vel max linear.x: 0.32142857142857145
/forklift/sim_cmd_vel last: linear.x=0.0 angular.z=0.0
/odom final x=-1.113917087004493 y=-0.5037984941175595
```

controller / bridge 关键日志：

```text
Configured FollowPath as ForkliftMpcController ... use_mpc_solver=true ... publish_control_cmd=true
MPC preview window: start=0 end=2 points=3 length=0.700
MPC solver seed accepted: v=0.450 w=0.000 steer=0.000 solver_score=4.209
Reached the goal!
Begin navigating from current location (-1.49, -0.38) to (-0.90, -0.50)
MPC preview window: start=0 end=9 points=10 length=0.508
MPC solver seed accepted: v=0.257 w=0.000 steer=0.000 solver_score=3.251
Goal succeeded
Stopping: brake command.
Stopping: command timeout.
```

bridge shutdown check：

```text
sim_command_bridge SIGINT exit code: 0
```

日志目录：

```text
log/p4_5_smoke/launch.log
log/p4_5_smoke/initialpose.log
log/p4_5_smoke/follow_path.log
log/p4_5_smoke/navigate_to_pose.log
log/p4_5_smoke/sim_bridge_sigint.log
```

## 6. P4 完成状态

P4.1 到 P4.5 已完成：

- MPC state/control types
- Path to internal trajectory
- Preview window
- Minimal constrained MPC solver
- Nav2 Controller API integration and FollowPath/NavigateToPose acceptance

下一步进入 P5：轨迹处理和平滑。
