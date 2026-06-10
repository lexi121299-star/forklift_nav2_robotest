# P4.1 MPC State and Control Notes

日期：2026-06-10

## 1. 本轮做了什么

P4.1 的目标是先把 ORU MPC 里的基础状态和控制量移进当前 Nav2 controller 插件，不直接开始 QP 求解。

新增文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_types.hpp
forklift_nav2_plugins/src/forklift_mpc_types.cpp
forklift_nav2_plugins/test/test_forklift_mpc_types.cpp
```

修改文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
forklift_nav2_plugins/CMakeLists.txt
forklift_nav2_plugins/package.xml
```

## 2. 状态和控制定义

`MpcState`：

```text
x      车辆参考点 x 位置，单位 m
y      车辆参考点 y 位置，单位 m
theta  车身 yaw，按 ROS 约定相对 +x 轴，单位 rad
phi    当前转向角，单位 rad
```

`MpcControl`：

```text
v  参考点切向速度，单位 m/s
w  转向角速度，也就是 phi_dot，单位 rad/s
```

这里的 `w` 对齐 ORU `Control::w()` 的含义：不是车身 yaw rate，而是转向角速度。车身 yaw rate 仍由车辆模型计算：

```text
theta_dot = v * tan(phi) / wheel_base
```

## 3. 当前落地方式

`forklift_mpc_types` 现在提供这些基础函数：

```text
makeMpcState()
makeMpcStateFromPose()
makeMpcControl()
makeMpcControlToSteeringTarget()
commandFromMpcControl()
predictMpcState()
```

它们负责：

```text
theta 归一化到 [-pi, pi]
phi 限制到最大转角范围内
v 限制到最大速度范围内
w 限制到最大转向角速度范围内
从 ROS pose quaternion 取 yaw 作为 theta
用 v + w + 当前 phi 预测下一步 x/y/theta/phi
```

`ForkliftMpcController` 现在不再只保存 `x/y/yaw`，而是用 `MpcState x/y/theta/phi` 做候选轨迹评分。仿真阶段暂时没有真实转向角反馈，所以 `phi` 使用上一帧输出的 steering angle 估计；真车阶段后续应从 Curtis feedback / `ForkliftVehicleState` 接入真实 `phi`。

输出仍保持不变：

```text
Nav2 API 返回 geometry_msgs/TwistStamped
publish_control_cmd=true 时同步发布 /forklift/control_cmd
```

## 4. 本轮验收命令

所有终端继续统一使用：

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

单元测试：

```bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select forklift_nav2_plugins --event-handlers console_cohesion+
```

已经通过的测试点：

```text
MpcState theta 归一化
MpcState phi 转角限幅
ROS pose yaw -> theta
MpcControl v/w 限幅
w 按 dt 推进 steering angle
predictMpcState 推进 x/y/theta/phi
负 dt 不改变状态
```

## 5. 短直线 FollowPath 冒烟

已使用 P3.1 bridge 链路跑过一条短直线：

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/install/forklift_nav2_demo/share/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_command_bridge:=true \
  gazebo_gui:=false \
  use_rviz:=false
```

初始位姿：

```text
map frame: x=-2.0, y=-0.5, yaw=0.0
```

FollowPath path：

```text
(-2.0, -0.5) -> (-1.7, -0.5) -> (-1.3, -0.5)
```

结果：

```text
/follow_path goal accepted
/follow_path status: SUCCEEDED
controller_server: Received a goal, begin computing control effort.
controller_server: Reached the goal!
```

采到的控制链路：

```text
/forklift/control_cmd velocity_mps: 0.394 -> 0.321 -> 0.193 -> 0.0
/forklift/control_cmd brake: true at stop
/forklift/sim_cmd_vel received nonzero Twist and then zero Twist
```

最终 odom / TF 采样：

```text
/odom position: x=-1.481881, y=-0.504622
/odom twist linear.x near 0
TF odom -> base_link translation: x=-1.481881, y=-0.504622, z=0.240000
```

日志位置：

```text
log/p4_1_smoke/nav2_bridge_launch.log
log/p4_1_smoke/follow_path_action_2.log
log/p4_1_smoke/control_cmd_echo_2.log
log/p4_1_smoke/sim_cmd_vel_echo_2.log
log/p4_1_smoke/direct_ros_collect.log
```

## 6. 下一步

P4.2：把 Nav2 `nav_msgs/Path` 转成 controller 内部 trajectory。

建议下一步先做：

```text
MpcTrajectoryPoint:
  x, y, theta, phi_ref, curvature, distance

Nav2 Path -> vector<MpcTrajectoryPoint>
```

验收标准：

```text
同一条短直线 path 能输出方向稳定的 trajectory
带转弯的 path 能输出连续 theta
后续 preview window 可以直接按 trajectory index 截取
```
