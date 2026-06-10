# P4.2 Path to Trajectory Notes

日期：2026-06-10

## 1. 本轮目标

P4.2 先把 Nav2 的 `nav_msgs/Path` 转成 controller 内部 trajectory，不直接进入 preview window 或 QP 求解。

新增文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_trajectory.hpp
forklift_nav2_plugins/src/forklift_mpc_trajectory.cpp
forklift_nav2_plugins/test/test_forklift_mpc_trajectory.cpp
```

修改文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
forklift_nav2_plugins/CMakeLists.txt
```

## 2. Trajectory 点定义

`MpcTrajectoryPoint` 包含：

```text
state.x
state.y
state.theta
state.phi
distance
curvature
steering_angle
```

含义：

```text
state.x / state.y      轨迹点位置
state.theta            轨迹切向方向，按 ROS yaw 约定相对 +x 轴
state.phi              参考转向角，目前等于 steering_angle
distance               从轨迹起点累计的路径长度
curvature              三点估计得到的有符号曲率
steering_angle         atan(wheel_base * curvature)，并按最大转角限幅
```

## 3. Path 转换规则

当前转换函数：

```text
pathToMpcTrajectory()
nearestTrajectoryIndex()
```

处理方式：

```text
空 path -> 空 trajectory
单点 path -> theta 使用 pose quaternion yaw
多点 path -> theta 由几何切向推导
重复点 -> 按 min_point_spacing 过滤
三点及以上 -> 用前中后三点估计 signed curvature
```

多点 path 默认不用输入 pose 的 orientation 推导 `theta`，而是从路径几何方向推导。这样手画 `/follow_path`、`NavigateToPose` 生成的 path、未来 task_manager 固定路线 path 都走同一套逻辑。

## 4. Controller 接入方式

`ForkliftMpcController::setPlan()` 现在会保存：

```text
global_plan_
global_trajectory_
```

`computeVelocityCommands()` 会把当前 frame 下的 transformed plan 转成 transformed trajectory，并在转换失败时直接报错。

当前 P4.2 仍保留 P3/P4.1 的实际控制打分逻辑，避免一次性改变跟踪行为。下一步 P4.3 preview window 会基于 `MpcTrajectory` 截取未来 N 个点。

## 5. 验收命令

所有终端继续先执行：

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

构建：

```bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
colcon build --packages-select forklift_nav2_plugins \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

测试：

```bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select forklift_nav2_plugins --event-handlers console_cohesion+
```

单元测试覆盖：

```text
空 path
单点 path 使用 pose yaw
短直线 path 输出 theta=0、curvature=0、累计 distance
竖直 path 输出 theta=pi/2
左转 path 输出正曲率和限幅后的正 steering angle
重复点过滤
nearestTrajectoryIndex 从指定 start index 后搜索
```

## 6. 短直线 FollowPath 冒烟

P4.2 接入 controller 后，已用 bridge 模式复跑短直线：

```text
map frame path:
(-2.0, -0.5) -> (-1.7, -0.5) -> (-1.3, -0.5)
```

结果：

```text
/follow_path accepted: true
/follow_path status: SUCCEEDED
/forklift/control_cmd samples: 20
/forklift/control_cmd max velocity_mps: 0.3942857142857143
/forklift/control_cmd final velocity_mps: 0.0
/forklift/control_cmd final brake: true
/forklift/sim_cmd_vel samples: 68
/forklift/sim_cmd_vel max linear.x: 0.3942857142857143
/odom final sample: x=-1.508140, y=-0.503681, linear.x near 0
```

日志：

```text
log/p4_2_smoke/nav2_bridge_launch.log
log/p4_2_smoke/follow_path_direct_client.log
```

## 7. 下一步

P4.3：基于 `MpcTrajectory` 增加 preview window。

建议先做：

```text
MpcPreviewWindow:
  vector<MpcTrajectoryPoint>
  start_index
  end_index
  length
```

验收：

```text
给定当前车辆 state 和 trajectory，能截取未来 N 个轨迹点
日志能显示 preview window 起点、终点、长度
不改变现有 FollowPath 输出链路
```
