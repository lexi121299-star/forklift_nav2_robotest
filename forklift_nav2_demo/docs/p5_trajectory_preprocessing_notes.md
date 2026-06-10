# P5 Trajectory Preprocessing Notes

## 1. 目标

P5 的目标是在 `ForkliftMpcController` 内部先处理 Nav2 给来的
`nav_msgs/Path`，再交给 preview window 和最小 MPC 求解器。

当前采用文档里的方案 B：

```text
Nav2 FollowPath
  -> ForkliftMpcController
  -> path preprocessing
  -> MpcTrajectory
  -> preview window
  -> minimal MPC solver
  -> ForkliftControlCommand / TwistStamped
```

这一步不接 ORU 的 qpOASES 求解器，也不改变仿真 bridge 的职责。
仿真仍然是：

```text
/forklift/control_cmd -> sim_command_bridge -> /forklift/sim_cmd_vel -> Gazebo
Gazebo -> /odom + TF
```

## 2. 本次实现

代码位置：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_trajectory.hpp
forklift_nav2_plugins/src/forklift_mpc_trajectory.cpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
forklift_nav2_plugins/test/test_forklift_mpc_trajectory.cpp
forklift_nav2_demo/config/forklift_nav2_oru_test.yaml
```

新增能力：

- 去掉重复或过近 path 点。
- 对稀疏 path 做等距插密。
- 对急转点做轻量 corner-cut smoothing。
- 重新估计每个轨迹点的 `theta`、曲率和参考转角。
- 根据最小转弯半径检查曲率是否超过车辆能力。
- 根据曲率给 preview window 限速。
- controller 打分和 preview window 都使用处理后的 path/trajectory，避免稀疏原始 path 与平滑 trajectory 互相拉扯。

`MpcTrajectoryPoint` 现在包含：

```text
state          x/y/theta/phi
distance       沿轨迹累计距离
curvature      有符号曲率
steering_angle 由曲率换算出的参考转角
speed_limit    该点建议速度上限
```

## 3. Controller 参数

仿真配置已写入：

```yaml
preprocess_path: true
trajectory_resample_spacing: 0.10
trajectory_smoothing_iterations: 1
trajectory_smoothing_corner_cut_ratio: 0.25
sharp_turn_warning_angle: 0.7853981633974483
minimum_turning_radius: 0.0
curvature_slowdown_enabled: true
curvature_slowdown_lateral_accel: 0.12
min_curvature_speed: 0.08
```

说明：

- `trajectory_resample_spacing`：插密后的目标点间距，单位 m。
- `trajectory_smoothing_iterations`：轻量平滑迭代次数，0 表示不平滑。
- `trajectory_smoothing_corner_cut_ratio`：急转点切角比例。
- `minimum_turning_radius: 0.0`：自动由 `wheel_base / tan(max_steering_angle)` 计算。
- `curvature_slowdown_lateral_accel`：曲率限速使用的横向加速度上限。
- `min_curvature_speed`：高曲率段最低速度上限。

## 4. 关键日志

收到 path 后会输出：

```text
P5 path preprocessing: input=<n> filtered=<n> smoothed=<n> resampled=<n> trajectory=<n> sharp_turns=<n> max_curvature=<k> allowed=<k> min_speed=<v>
```

如果检测到急转：

```text
P5 detected <n> sharp path turn(s), max heading change <rad> rad; smoothing/resampling is enabled
```

如果曲率超过最小转弯半径能力：

```text
P5 trajectory curvature <k> exceeds allowed <k>; controller will clamp steering and slow down
```

如果曲率限速生效：

```text
P5 curvature speed limit active: max_v <old> -> <new>
```

## 5. 验收命令

所有 ROS 终端先设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/pl/robotest/install/setup.bash
```

构建：

```bash
colcon build --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

单测：

```bash
colcon test --packages-select forklift_nav2_plugins --event-handlers console_cohesion+
```

headless 仿真启动：

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/install/forklift_nav2_demo/share/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_command_bridge:=true \
  gazebo_gui:=false \
  use_rviz:=false
```

验收重点：

- 短直线 FollowPath 仍然 `SUCCEEDED`。
- 大圆弧 FollowPath 能输出密集 trajectory 和连续 preview window。
- 90 度稀疏 path 能打印 sharp turn 诊断，并被插密/平滑。
- bridge 仍只发布 `/forklift/sim_cmd_vel`，不发布 `/odom` 或 TF。

## 6. 当前结果

已通过：

```text
colcon build --packages-select forklift_nav2_plugins --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon build --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select forklift_nav2_plugins --event-handlers console_cohesion+
```

测试数量：

```text
test_forklift_mpc_types: 6 passed
test_forklift_mpc_trajectory: 11 passed
test_forklift_mpc_preview_window: 5 passed
test_forklift_mpc_solver: 5 passed
```

headless bridge 仿真验收：

```text
retry_short_straight: accepted=True points=2
retry_short_straight: status=4 SUCCEEDED
retry_short_straight: control_samples=16 max_velocity_mps=0.321
retry_short_straight: sim_cmd_samples=76 max_linear_x=0.321
retry_short_straight: odom_final x=-1.682 y=-0.504

retry_gentle_large_arc: accepted=True points=9
retry_gentle_large_arc: status=4 SUCCEEDED
retry_gentle_large_arc: control_samples=26 max_velocity_mps=0.450
retry_gentle_large_arc: sim_cmd_samples=48 max_linear_x=0.450
retry_gentle_large_arc: odom_final x=-1.042 y=-0.374
```

对应 controller 日志：

```text
Configured FollowPath as ForkliftMpcController ... preprocess_path=true resample=0.100 smooth_iter=1 ...

P5 path preprocessing: input=2 filtered=2 smoothed=2 resampled=7 trajectory=7 sharp_turns=0 max_curvature=0.000 allowed=0.511 min_speed=0.450
MPC preview window: start=0 end=6 points=7 length=0.550
Reached the goal!

P5 path preprocessing: input=9 filtered=9 smoothed=16 resampled=11 trajectory=11 sharp_turns=0 max_curvature=0.405 allowed=0.511 min_speed=0.450
MPC preview window: start=1 end=10 points=10 length=0.810
Reached the goal!
```

90 度稀疏 path 验收：

```text
sparse_90_turn: accepted=True points=3
sparse_90_turn: status=4 SUCCEEDED
sparse_90_turn: control_samples=430 max_velocity_mps=0.321
sparse_90_turn: sim_cmd_samples=858 max_linear_x=0.321
sparse_90_turn: odom_final x=0.627 y=1.804
```

对应 controller 日志：

```text
P5 path preprocessing: input=3 filtered=3 smoothed=4 resampled=15 trajectory=15 sharp_turns=1 max_curvature=5.854 allowed=0.511 min_speed=0.080
P5 detected 1 sharp path turn(s), max heading change 1.571 rad; smoothing/resampling is enabled
P5 trajectory curvature 5.854 exceeds allowed 0.511 from min turning radius 1.957 m
P5 curvature speed limit active: max_v 0.450 -> 0.080
Reached the goal!
```

另外保留了一条过急圆弧的负向样本：

```text
large_arc: accepted=True points=7
large_arc: status=6
large_arc: control_samples=448 max_velocity_mps=0.080
```

这条 path 的曲率 `1.215` 已超过允许曲率 `0.511`，controller 正确打印最小转弯半径警告并把速度压到 `0.080 m/s`，随后 Nav2 progress checker 因进展不足 abort。它不是 P5 正向验收路径，但可以作为后续调参和 P6 footprint/path feasibility 的参考。

日志文件：

```text
log/p5_smoke/follow_path_acceptance.log
log/p5_smoke/retry_follow_path_acceptance.log
log/p5_smoke/retry_launch.log
```
