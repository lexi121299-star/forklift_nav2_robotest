# P3.1 Vehicle Model and Bridge Mode Notes

日期：2026-06-10

## 1. 本轮做了什么

本轮在 P1/P2 统一车辆接口基础上，继续完成两件事：

1. 让 `sim_command_bridge` 更容易一键启动。
2. 开始 P3.1 `forklift_vehicle_model`，并让 `ForkliftMpcController` 可以输出 `ForkliftControlCommand`。

关键文件：

```text
forklift_vehicle_interface/launch/sim_command_bridge.launch.py
forklift_vehicle_interface/forklift_vehicle_interface/sim_command_bridge.py

forklift_nav2_demo/launch/forklift_gazebo.launch.py
forklift_nav2_demo/launch/forklift_navigation.launch.py
forklift_nav2_demo/urdf/forklift_diff_drive.urdf.xacro

forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_vehicle_model.hpp
forklift_nav2_plugins/src/forklift_vehicle_model.cpp
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
```

## 2. 当前两种控制链路

### 2.1 默认 Nav2/Gazebo 链路

默认模式不改变 P0/P1/P2 基线：

```text
Nav2 controller / velocity_smoother
  -> /cmd_vel
  -> Gazebo diff_drive plugin
  -> /odom + TF
```

适合继续验证原来的 DWB/Nav2 仿真。

### 2.2 ForkliftControlCommand bridge 链路

打开 `use_sim_command_bridge:=true` 后，Gazebo 不再听 `/cmd_vel`，而是听 `/forklift/sim_cmd_vel`：

```text
ForkliftMpcController
  -> /forklift/control_cmd

sim_command_bridge
  -> /forklift/sim_cmd_vel

Gazebo diff_drive plugin
  -> /odom + TF
```

这样即使 Nav2 内部仍然发布 `/cmd_vel`，也不会直接驱动 Gazebo 车辆。真正驱动车的是 bridge 输出的 `/forklift/sim_cmd_vel`。

注意：`sim_command_bridge` 仍然不发布 `/odom` 或 TF；仿真阶段 `/odom` 和 TF 仍由 Gazebo 输出。

## 3. remap / topic 隔离方式

这次没有直接修改 Nav2 官方 `nav2_bringup` launch。处理方式是让 Gazebo diff-drive 插件的 ROS remap 可配置：

```xml
<xacro:arg name="gazebo_cmd_vel_topic" default="cmd_vel"/>
<ros>
  <remapping>cmd_vel:=$(arg gazebo_cmd_vel_topic)</remapping>
</ros>
```

默认：

```text
gazebo_cmd_vel_topic = cmd_vel
```

bridge 模式：

```text
gazebo_cmd_vel_topic = /forklift/sim_cmd_vel
sim_command_bridge cmd_vel_topic = /forklift/sim_cmd_vel
```

这比直接改 Nav2 bringup 更稳，因为默认 Nav2 行为保持不变，bridge 模式只改变 Gazebo 接收命令的入口。

## 4. P3.1 车辆模型

新增 `ForkliftVehicleModel`，统一放这些车辆参数：

```text
wheel_base
max_steering_angle
max_steering_angle_velocity
max_velocity
max_acceleration
max_angular_velocity
```

当前模型支持：

```text
v + steering_angle -> Twist
omega = v * tan(steering_angle) / wheel_base
短时间预测 x, y, theta
```

`ForkliftMpcController` 现在内部使用这个模型做预测和 `TwistStamped` 输出。打开参数后，它会同步发布 `/forklift/control_cmd`：

```yaml
publish_control_cmd: true
control_cmd_topic: "/forklift/control_cmd"
```

`forklift_nav2_demo/config/forklift_nav2_oru_test.yaml` 已经打开 `publish_control_cmd: true`，用于测试统一命令链路。

## 5. 每个终端的固定环境

所有 ROS 终端先执行：

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

如果遇到 conda 抢 Python 导致 `No module named em`，构建时用系统 Python：

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 6. 启动命令

### 6.1 单独启动 bridge

```bash
ros2 launch forklift_vehicle_interface sim_command_bridge.launch.py
```

单独测试时默认输出 `/cmd_vel`。

指定 bridge 输出到仿真隔离 topic：

```bash
ros2 launch forklift_vehicle_interface sim_command_bridge.launch.py \
  cmd_vel_topic:=/forklift/sim_cmd_vel
```

### 6.2 只启动 Gazebo，并启用 bridge 模式

```bash
ros2 launch forklift_nav2_demo forklift_gazebo.launch.py \
  use_sim_command_bridge:=true
```

此时：

```text
/forklift/control_cmd -> sim_command_bridge -> /forklift/sim_cmd_vel -> Gazebo
```

### 6.3 启动默认 Nav2/Gazebo 基线

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml
```

默认不启用 bridge，Gazebo 仍听 `/cmd_vel`。

### 6.4 启动 ForkliftMpcController + bridge 链路

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/install/forklift_nav2_demo/share/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_command_bridge:=true
```

headless 版本：

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/install/forklift_nav2_demo/share/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_command_bridge:=true \
  gazebo_gui:=false \
  use_rviz:=false
```

## 7. 手动验收命令

### 7.1 bridge 输出速度

启动 Gazebo bridge 模式后，发布统一控制命令：

```bash
ros2 topic pub --once /forklift/control_cmd forklift_msgs/msg/ForkliftControlCommand "{
  enable: true,
  brake: false,
  forward: true,
  reverse: false,
  velocity_mps: 0.2,
  steering_angle_rad: 0.0
}"
```

检查 bridge 输出：

```bash
ros2 topic echo /forklift/sim_cmd_vel --once
```

期望：

```text
linear.x: 0.2
angular.z: 0.0
```

检查 Gazebo odom：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

### 7.2 确认 topic 隔离

bridge 模式下，Gazebo 应该听 `/forklift/sim_cmd_vel`，而不是直接被 Nav2 的 `/cmd_vel` 驱动。

可用这些命令看发布者/订阅者：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /forklift/sim_cmd_vel -v
ros2 topic info /forklift/control_cmd -v
```

预期：

```text
/forklift/control_cmd 由 ForkliftMpcController 发布
/forklift/sim_cmd_vel 由 sim_command_bridge 发布，并被 Gazebo 订阅
/cmd_vel 即使存在，也不是 Gazebo bridge 模式的真实驱动车入口
```

## 8. 本轮验收记录

已经验证：

```bash
colcon build --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo --symlink-install
colcon test --packages-select forklift_vehicle_interface
colcon test --packages-select forklift_nav2_plugins
ros2 launch forklift_vehicle_interface sim_command_bridge.launch.py --show-args
ros2 launch forklift_nav2_demo forklift_gazebo.launch.py --show-args
ros2 launch forklift_nav2_demo forklift_navigation.launch.py --show-args
```

bridge launch smoke test 已采到：

```text
/forklift/sim_cmd_vel.linear.x = 0.2
```

## 9. 下一步建议

1. 用 `forklift_nav2_oru_test.yaml` 跑一条短直线 FollowPath，确认 `/forklift/control_cmd` 连续输出。
2. 确认 bridge 模式下 `/odom` 只来自 Gazebo，TF 仍为 Gazebo 输出。
3. 再进入 P4.1，把 ORU State / Control 概念继续移入 `ForkliftMpcController`。
