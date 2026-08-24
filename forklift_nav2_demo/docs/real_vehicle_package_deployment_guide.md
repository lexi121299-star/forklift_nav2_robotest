# 真车部署 Package 编译清单

本文档说明车载工控机上需要放哪些 ROS2 package、哪些 package 不需要上车、推荐编译命令，以及启动前需要确认的外部依赖。

适用场景：

- ROS2 Foxy
- 真车导航
- 外部定位系统提供定位
- 0.05m 工厂地图
- 当前 forklift Nav2 / Task Manager / Safety / Vehicle Interface 架构

## 1. 车上必须编译的 package

最小真车导航栈需要以下 package：

```text
forklift_msgs
forklift_oru_planner
forklift_nav2_plugins
forklift_vehicle_interface
forklift_safety
forklift_nav2_demo
```

如果车上还要接调度、站点任务、路线任务、两段式取托盘流程，则还需要：

```text
forklift_task_manager
```

## 2. 各 package 职责

| Package | 是否必须 | 作用 |
| --- | --- | --- |
| `forklift_msgs` | 必须 | 自定义 msg / srv / action，所有自定义接口的公共定义 |
| `forklift_oru_planner` | 必须 | lattice planner 核心算法库 |
| `forklift_nav2_plugins` | 必须 | Nav2 自定义全局规划器和局部控制器 |
| `forklift_vehicle_interface` | 必须 | 真车底盘接口、CAN codec、车辆状态、故障、里程计、叉臂 adapter |
| `forklift_safety` | 必须 | 最终安全闸，拦截并限幅所有运动命令 |
| `forklift_nav2_demo` | 必须 | 真车 launch、URDF、Nav2 yaml、BT xml、配置文件 |
| `forklift_task_manager` | 可选但推荐 | 任务编排、站点/路线任务、两段式取托盘、任务状态上报 |

## 3. 最小导航编译命令

只测试 Nav2、RViz 发目标、车辆底盘运动时，编译：

```bash
colcon build --packages-select \
  forklift_msgs \
  forklift_oru_planner \
  forklift_nav2_plugins \
  forklift_vehicle_interface \
  forklift_safety \
  forklift_nav2_demo
```

编译后加载环境：

```bash
source install/setup.bash
```

## 4. 带任务管理的编译命令

如果需要调度平台、站点任务、路线任务、取托盘任务：

```bash
colcon build --packages-select \
  forklift_msgs \
  forklift_oru_planner \
  forklift_nav2_plugins \
  forklift_vehicle_interface \
  forklift_safety \
  forklift_nav2_demo \
  forklift_task_manager
```

编译后加载环境：

```bash
source install/setup.bash
```

## 5. 第一版不建议上车编译的 package

以下 package 主要用于仿真、历史 ORU 对比、实验或旧流程，第一版真车部署不需要：

```text
forklift_sim
forklift_warehouse_sim
iliad/
navigation_oru-release/
gazebo_oru/
```

说明：

- `forklift_sim`：轻量仿真，不是真车必需。
- `forklift_warehouse_sim`：仓库仿真 demo，不是真车必需。
- `iliad/`：ILIAD 相关实验包，不进入当前真车主链路。
- `navigation_oru-release/`：原 ORU 源码/对比参考，不是当前 Nav2 插件运行依赖。
- `gazebo_oru/`：Gazebo 仿真资源，真车不需要。

## 6. 车上还必须准备的非 package 文件

地图文件需要放到车上，例如：

```text
map5.yaml
map5.pgm
```

当前真车推荐 Nav2 参数：

```text
forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml
```

该参数文件使用：

```text
resolution: 0.05
global/local footprint: [[0.843, 0.58], [0.843, -0.58], [-2.043, -0.58], [-2.043, 0.58]]
lattice_step_distance: 0.20
amcl.tf_broadcast: false
```

## 7. 外部定位系统需要提供什么

真车模式假设定位系统负责发布：

```text
map -> odom
```

车辆接口或里程计节点负责发布：

```text
odom -> base_link
```

URDF / `robot_state_publisher` 负责发布：

```text
base_footprint -> base_link -> base_scan
```

最终 TF 链路应为：

```text
map -> odom -> base_link -> base_scan
```

如果实际使用 `base_footprint`，则应保持：

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

注意：

- 如果定位系统已经发布 `map -> odom`，AMCL 不能再发布同一条 TF。
- 因此真车参数里设置 `amcl.tf_broadcast: false`。
- 如果定位系统直接发布 `/amcl_pose` 但不发布 `map -> odom`，Nav2 仍然不能正常工作，需要定位侧补 TF 或增加 adapter。

### 7.1 方案 B：定位只提供 map 下的 base_link 位姿

如果定位系统不能直接发布 `map -> odom`，但能提供全局位姿，则让定位侧只发布
Odometry topic，不发布 TF：

```text
/fusion/localization
type: nav_msgs/msg/Odometry
header.frame_id: map
child_frame_id: base_link
pose: base_link 在 map 坐标系下的全局位姿
```

导航侧启动 `map_odom_localization_adapter`，它会读取：

```text
定位侧: /fusion/localization 里的 map -> base_link
底盘侧: TF odom -> base_link
```

并计算发布：

```text
map -> odom = map -> base_link * inverse(odom -> base_link)
```

注意：

- 定位侧不要发布 `odom -> base_link`，这条由 `forklift_vehicle_interface` 发布。
- 定位侧也不要发布 `map -> base_link` TF，避免和 `map -> odom -> base_link` 形成两条 TF 路径。
- 如果定位侧以后改成直接发布 `map -> odom`，必须关闭 adapter。
- 当前 adapter 按 2D 导航处理，只使用 `x/y/yaw`，忽略 z、roll、pitch。

## 8. 真车启动命令

先使用 dry-run 模式，不真正发 CAN：

```bash
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py \
  map:=/workspace/map5.yaml \
  nav2_params_file:=/workspace/forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml \
  vehicle_dry_run:=true \
  can_interface:=can0 \
  use_rviz:=true
```

如果定位侧采用 §7.1 的方案 B，再加：

```bash
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py \
  map:=/workspace/map5.yaml \
  nav2_params_file:=/workspace/forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml \
  vehicle_dry_run:=true \
  can_interface:=can0 \
  use_rviz:=true \
  use_localization_adapter:=true \
  localization_topic:=/fusion/localization
```

确认定位、TF、规划和安全链路正常后，再切到真实 CAN：

```bash
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py \
  map:=/workspace/map5.yaml \
  nav2_params_file:=/workspace/forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml \
  vehicle_dry_run:=false \
  can_interface:=can0 \
  use_rviz:=true
```

## 9. Task Manager 启动命令

如果需要站点任务、路线任务、取托盘流程，主导航栈启动后另开终端：

```bash
source install/setup.bash

ros2 launch forklift_task_manager task_manager.launch.py
```

Task Manager 不直接控制底盘，不绕过 safety。运动链路仍然是：

```text
Task Manager -> Nav2 -> forklift_nav2_plugins controller
-> /forklift/control_cmd_raw
-> forklift_safety
-> /forklift/control_cmd
-> forklift_vehicle_interface
-> CAN
```

## 10. 上车前检查

加载环境后检查 package 是否存在：

```bash
ros2 pkg list | grep -E "forklift_msgs|forklift_oru_planner|forklift_nav2_plugins|forklift_vehicle_interface|forklift_safety|forklift_nav2_demo|forklift_task_manager"
```

检查自定义接口是否生成：

```bash
ros2 interface show forklift_msgs/msg/ForkliftControlCommand
ros2 interface show forklift_msgs/msg/ForkliftVehicleState
ros2 interface show forklift_msgs/msg/ForkliftFaultState
ros2 interface show forklift_msgs/action/ForkMoveTo
ros2 interface show forklift_msgs/action/ExecutePalletPickup
```

检查 Nav2 插件是否安装：

```bash
ros2 pkg prefix forklift_nav2_plugins
```

检查 launch 是否可见：

```bash
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py --show-args
```

## 11. 启动后检查

启动后检查关键节点：

```bash
ros2 node list
```

必须看到：

```text
/map_server
/planner_server
/controller_server
/bt_navigator
/safety_command_gate
```

如果使用当前 `forklift_real_navigation.launch.py`，还应看到：

```text
/robot_state_publisher
/curtis_vehicle_interface
```

检查 lifecycle：

```bash
for n in map_server planner_server controller_server bt_navigator recoveries_server waypoint_follower; do
  echo ===$n===
  ros2 lifecycle get /$n || true
done
```

正常应为：

```text
active [3]
```

检查 TF：

```bash
timeout 3 ros2 topic echo /tf | grep -E "frame_id: map|child_frame_id: odom|frame_id: odom|child_frame_id: base_footprint|child_frame_id: base_link"
```

必须能看到：

```text
map -> odom
odom -> base_link
```

或：

```text
map -> odom
odom -> base_footprint
base_footprint -> base_link
```

## 12. 常见错误

| 现象 | 常见原因 | 处理 |
| --- | --- | --- |
| planner 等不到 `base_link -> map` | 没有 `map -> odom` | 检查定位系统 TF |
| 有 `/odom` 但不能导航 | 只有局部里程，没有地图定位 | 定位系统必须给 `map -> odom` |
| 路线贴障碍太近 | global footprint 太小或 inflation 太小 | 使用真实 footprint，加大 global inflation |
| 车不动 | safety gate 拦截、CAN dry-run、车辆未自动模式 | 查 `/forklift/safety_gate/status`、`vehicle_dry_run`、车辆模式 |
| 自定义 action 找不到 | `forklift_msgs` 没编译或没 source | 重新编译并 `source install/setup.bash` |
| Nav2 插件加载失败 | `forklift_nav2_plugins` 没编译或 plugin xml 没安装 | 编译插件包并检查 `ros2 pkg prefix forklift_nav2_plugins` |
