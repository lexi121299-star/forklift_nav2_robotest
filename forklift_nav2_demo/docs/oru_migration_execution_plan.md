# ORU Migration Execution Plan

这份文档是后续把 `navigation_oru-release` 逐步移植到当前 ROS2 forklift 仿真，并最终能上真车运行的执行清单。

目标不是一次性把 ORU 全套搬过来，而是按最小可验证闭环推进：

```text
地图/任务 -> 路径 -> 轨迹 -> 控制 -> 底盘协议 -> 车辆反馈 -> 定位/TF
```

每一步都必须能单独测试、能回退、能解释问题发生在哪一层。

## 0. 当前基线

当前已经具备：

- `forklift_nav2_demo`
  - Gazebo world、forklift URDF、Nav2 bringup、RViz。
  - 清理后的地图：`/home/pl/robotest/forklift_factory_big_map_clean.yaml`。
  - 手画路径工具：`forklift_manual_path_follower`，通过 `/follow_path` 让 controller 跟踪手画 path。

- `forklift_nav2_plugins`
  - `forklift_nav2_plugins/OruGlobalPlanner`
    - 当前是 costmap-aware A* scaffold，不是完整 ORU lattice/motion primitive planner。
  - `forklift_nav2_plugins/ForkliftMpcController`
    - 当前是 sampled predictive controller scaffold，不是完整 ORU QP-MPC。

当前主要问题：

- 仿真时间、`/clock`、TF、`/odom` 必须稳定，否则 2D Pose Estimate、costmap、FollowPath 都会失败。
- 当前 controller 能做简单预测跟踪，但还没有 ORU 的 `x/y/theta/phi` 状态、转角约束和 QP 求解。
- 当前 global planner 不理解三支点/叉车运动学，只是在 costmap 上找可通行格子。

## 1. 总体迁移顺序

按这个顺序做，不跳步：

```text
P0  稳定仿真基线和 TF
P1  建 forklift_msgs
P2  建 forklift_vehicle_interface
P3  定义真实车控制模型和命令接口
P4  把 ORU MPC 核心移进 ForkliftMpcController
P5  接入轨迹处理和平滑
P6  把 ORU motion planner / primitive 思路移进 GlobalPlanner
P7  加 task_manager，统一 A-B 导航、手画 path、调度任务
P8  加 safety，做急停、限速、防撞、掉边保护
P9  真车低速联调
```

原则：

- 不直接改 `/opt/ros/humble` 的 Nav2 源码。
- 不把底盘协议写进 planner/controller。
- 不把调度任务写进 controller。
- 所有算法通过 Nav2 plugin 或独立 node 接入。
- 每一步都保留一个能回退的配置文件。

## 2. P0: 稳定仿真基线

目标：

- Gazebo、Nav2、RViz、AMCL、TF、costmap 稳定。
- 能用 `2D Pose Estimate` 定位。
- 能用 `NavigateToPose` 或 `/follow_path` 触发 controller。

当前要检查：

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

验收标准：

- `/clock` 有数据。
- `/odom` 有数据。
- `odom -> base_link` 连续可查。
- `map -> odom` 在 2D Pose Estimate 后可查。
- `/controller_server` 和 `/planner_server` 是 `active`。
- RViz 里能显示 `/map`、global costmap、local costmap、robot footprint。

不要进入 P1/P2 的条件：

- `odom frame does not exist` 还在出现。
- `2D Pose Estimate` 点了之后 AMCL 没有响应。
- Gazebo 车没有 spawn 或 `/odom` 不发布。

## 3. P1: 建 forklift_msgs

目标：

定义真车和上层任务需要的消息，不直接复用 ORU 消息作为对外接口。

新 package：

```text
forklift_msgs
```

建议消息：

```text
msg/ForkliftControlCommand.msg
msg/ForkliftVehicleState.msg
msg/ForkliftFaultState.msg
msg/ForkliftTask.msg
msg/ForkliftTaskState.msg
srv/SetControlMode.srv
srv/SetEmergencyStop.srv
```

第一版 `ForkliftControlCommand` 建议包含：

```text
std_msgs/Header header
float64 velocity
float64 steering_angle
float64 steering_angle_velocity
float64 fork_height
bool enable
```

第一版 `ForkliftVehicleState` 建议包含：

```text
std_msgs/Header header
float64 velocity
float64 steering_angle
float64 battery_percent
bool enabled
bool emergency_stopped
string mode
```

ORU 参考：

- `orunav_msgs`
  - 只作为字段设计参考，不作为最终对外协议。

验收标准：

- `colcon build --packages-select forklift_msgs` 通过。
- `ros2 interface show forklift_msgs/msg/ForkliftControlCommand` 正常。

## 4. P2: 建 forklift_vehicle_interface

目标：

把 Nav2/controller 输出转换成真实底盘协议，同时把底盘反馈转换成 ROS 标准 `/odom` 和 TF。

新 package：

```text
forklift_vehicle_interface
```

职责：

```text
订阅:
  /cmd_vel
  /forklift/control_cmd
  /forklift/emergency_stop

发布:
  /odom
  odom -> base_link TF
  /forklift/vehicle_state
  /forklift/fault_state
```

第一阶段先做仿真版接口：

```text
/forklift/control_cmd -> /cmd_vel
/odom passthrough 或 fake odom monitor
```

第二阶段接真实协议：

```text
/forklift/control_cmd -> CAN/串口/TCP -> 底盘控制器
底盘反馈 -> /odom + /forklift/vehicle_state
```

ORU 参考：

- `orunav_mpc/src/commandSender.*`
- `orunav_mpc/src/canlibWrapper.*`
- `orunav_mpc/src/canSensorReader.*`

不要直接照搬：

- ORU 的 CAN 代码可以参考，但不要把它塞进 Nav2 controller。
- 真车协议必须独立在 `forklift_vehicle_interface`，这样后面换 controller 不影响底盘通信。

验收标准：

- 不启动 Nav2，只用测试命令也能驱动车：

```bash
ros2 topic pub /forklift/control_cmd forklift_msgs/msg/ForkliftControlCommand ...
```

- 真车或仿真能发布稳定 `/odom`。
- `tf2_echo odom base_link` 连续可查。
- 命令超时后车辆自动停车。

## 5. P3: 固定控制接口和车辆模型

目标：

明确三支点叉车 controller 到底输出什么。

推荐内部控制模型：

```text
state:
  x, y, theta, phi

control:
  v, steering_angle 或 steering_angle_velocity
```

其中：

- `x, y`: 地图/odom 平面位置。
- `theta`: 车体朝向。
- `phi`: 转向角。
- `v`: 车辆纵向速度。

当前仿真还是 diff drive，所以短期转换为：

```text
omega = v * tan(phi) / wheel_base
/cmd_vel.linear.x = v
/cmd_vel.angular.z = omega
```

真车上车时不要只用 `/cmd_vel.angular.z`，应该优先发：

```text
velocity + steering_angle
```

要做的文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_vehicle_model.hpp
forklift_nav2_plugins/src/forklift_vehicle_model.cpp
```

当前落地方式：

- `forklift_vehicle_model` 统一保存 `wheel_base`、最大转角、最大转角速度、最大速度、最大加速度等车辆参数。
- `ForkliftMpcController` 内部继续返回 Nav2 必须的 `TwistStamped`，但可通过 `publish_control_cmd` 参数同步发布 `/forklift/control_cmd`。
- 仿真阶段可以用 `sim_command_bridge` 把 `/forklift/control_cmd` 转成 Gazebo 的 `/cmd_vel`；默认不开，避免和 Nav2 默认 `/cmd_vel` 链路抢同一 topic。

验收标准：

- 同一条 path 下，仿真 controller 和真实车接口使用同一个车辆参数：

```text
wheel_base
max_steering_angle
max_steering_angle_velocity
max_velocity
max_acceleration
```

## 6. P4: 移植 ORU MPC 到 ForkliftMpcController

目标：

把 `orunav_mpc` 的核心控制能力移植进当前 Nav2 controller 插件，而不是单独运行 ORU controller 节点。

目标文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
```

ORU 源码参考：

```text
navigation_oru-release/orunav_mpc/src/state.*
navigation_oru-release/orunav_mpc/src/control.h
navigation_oru-release/orunav_mpc/src/trajectory.*
navigation_oru-release/orunav_mpc/src/previewWindow.*
navigation_oru-release/orunav_mpc/src/qpProblem.*
navigation_oru-release/orunav_mpc/src/qpConstraints.*
navigation_oru-release/orunav_mpc/src/controller.*
navigation_oru-release/orunav_mpc/qpOASES/
```

迁移顺序：

### P4.1 状态和控制结构

先移植或重写：

```text
State:
  x, y, theta, phi

Control:
  v, w 或 v, steering_rate
```

要求：

- 不依赖 ORU 全局变量。
- 不依赖 ORU 自己的 node/thread 框架。
- 可单元测试。

验收：

- 给定当前 pose、速度、转角，能构造 MPC 初始状态。

### P4.2 Path -> Trajectory

把 Nav2 的 `nav_msgs/Path` 转成 controller 内部 trajectory。

要求：

- 支持 `NavigateToPose` 生成的 path。
- 支持手画 `/follow_path` path。
- 支持未来 task_manager 下发的固定路线 path。

验收：

- 同一条 path 能输出带方向、曲率或转角估计的轨迹点。

### P4.3 预瞄窗口

参考 ORU：

```text
previewWindow
trajectoryCache
```

目标：

- 从当前车辆位置截取未来 N 个轨迹点。
- MPC 每次只优化有限时间窗口。

验收：

- RViz 或日志能显示当前 preview window 的起点、终点、长度。

### P4.4 QP/MPC 求解

参考 ORU：

```text
qpProblem
qpConstraints
qpOASES
```

先做最小版本：

- 只约束速度。
- 只约束转角。
- 只约束转角速度。
- 暂时不做复杂多机器人/任务状态。

验收：

- controller 每个周期输出稳定 `v + steering_angle`。
- 不出现剧烈振荡。
- 停车时速度收敛到 0。

### P4.5 接 Nav2 Controller API

保持接口：

```cpp
computeVelocityCommands()
setPlan()
setSpeedLimit()
```

输出策略：

- 仿真：输出 `geometry_msgs/TwistStamped` 给 Nav2 `/cmd_vel` 链路。
- 真车：通过 `forklift_vehicle_interface` 或后续控制命令 bridge 输出 `ForkliftControlCommand`。

验收：

- `/follow_path` 可以跟踪手画路径。
- `NavigateToPose` 可以使用同一个 controller 跟踪 global planner 输出路径。

## 7. P5: 接入轨迹处理和平滑

目标：

让 global planner 或手画 path 不直接变成生硬控制输入，而是先变成适合叉车的平滑轨迹。

ORU 参考：

```text
navigation_oru-release/orunav_path_smoother
navigation_oru-release/orunav_trajectory_processor
```

Nav2 接入方式：

```text
方案 A:
  写 nav2_core::Smoother 插件

方案 B:
  在 ForkliftMpcController 内部做 path preprocessing

方案 C:
  写独立 forklift_trajectory_processor node
```

推荐先做 B，再做 A。

第一版能力：

- 路径插密。
- 路径方向估计。
- 最小转弯半径检查。
- 曲率过大处降速。

验收：

- 手画稀疏 path 会自动变成密集、连续、方向合理的轨迹。
- 90 度急转会被提示或自动平滑。

## 8. P6: 移植 ORU motion planner / primitives

目标：

把 global planner 从普通 grid A* 升级成叉车可行驶路径规划。

当前：

```text
forklift_nav2_plugins/OruGlobalPlanner
  costmap-aware A*
```

目标：

```text
forklift_nav2_plugins/OruGlobalPlanner
  motion primitive / lattice / 车辆运动学约束
```

ORU 参考：

```text
navigation_oru-release/orunav_motion_planner
navigation_oru-release/orunav_geometry
navigation_oru-release/orunav_constraint_extract
navigation_oru-release/orunav_motion_planner/Primitives
navigation_oru-release/orunav_motion_planner/LookupTables
```

迁移顺序：

### P6.1 保留 Nav2 costmap 输入

不要绕开 Nav2 costmap。

输入仍然是：

```text
global_costmap
footprint
map frame
start pose
goal pose
```

验收：

- RViz 的 global costmap 和 planner 看到的障碍一致。

### P6.2 加 heading state

从 2D cell：

```text
x, y
```

升级到：

```text
x, y, theta_index
```

验收：

- 同一个位置不同朝向可以有不同可行性。

### P6.3 加 motion primitives

primitive 类型：

```text
forward straight
forward left arc
forward right arc
reverse straight
reverse left arc
reverse right arc
```

第一版可以先禁用 reverse，稳定后再开。

验收：

- 全局路径不会出现车辆无法执行的原地横移或过急转弯。

### P6.4 footprint collision along primitive

不是只检查终点，而是检查 primitive 轨迹上的多个采样点。

验收：

- 叉臂不会在转弯中扫到障碍。

## 9. P7: 建 forklift_task_manager

目标：

统一三种任务入口：

```text
入口 A:
  A 点到 B 点自动规划
  NavigateToPose

入口 B:
  多点任务
  NavigateThroughPoses / FollowWaypoints

入口 C:
  固定路线或手画路线
  FollowPath
```

新 package：

```text
forklift_task_manager
```

职责：

- 接调度系统。
- 保存站点。
- 保存固定路线。
- 调用 Nav2 action。
- 记录任务状态。
- 管理等待、装卸货、充电等流程。

验收：

- 通过一个简单 service/action 下发：

```text
go_to_station("A")
follow_route("dock_to_truck")
cancel_task()
```

- task_manager 不直接发布 `/cmd_vel`。

## 10. P8: 建 forklift_safety

目标：

上车必须有独立安全层，不依赖 planner/controller 自觉避障。

新 package：

```text
forklift_safety
```

职责：

- 急停。
- 命令 watchdog。
- 限速区。
- 掉边保护。
- 防撞监控。
- 车辆状态异常停车。

可结合 Nav2：

```text
nav2_collision_monitor
costmap filter / keepout mask
speed zone
```

验收：

- 任意时候急停都能切断运动命令。
- `/cmd_vel` 超时自动停车。
- 障碍物进入保护区时停车或限速。
- 地图边缘/掉落风险区域不会允许继续行驶。

## 11. P9: 真车低速联调

顺序：

```text
1. 不启动 Nav2，只测 vehicle_interface 通信
2. 测 /odom 和 TF
3. 测手动低速 /forklift/control_cmd
4. 测 FollowPath 短直线
5. 测 FollowPath 大圆弧
6. 测 NavigateToPose 空旷区域
7. 测障碍物停车
8. 测任务点和调度流程
```

低速限制：

```text
max_velocity <= 0.2 m/s
max_steering_angle_velocity 限制到保守值
必须有人看急停
```

禁止一开始测试：

- 窄门。
- 贴货架。
- 平板车边缘。
- 倒车入库。
- 动态人车混行。

## 12. 每次开发的固定验证命令

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/pl/robotest/install/setup.bash

ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_time:=true
```

检查 TF：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

手画 path：

```bash
ros2 run forklift_nav2_demo forklift_manual_path_follower \
  --ros-args -p use_sim_time:=true
```

抓日志：

```bash
grep -E "ForkliftMpcController|OruGlobalPlanner|follow_path|Failed to make progress|transform|ERROR|WARN" \
  /tmp/forklift_mpc_test.log
```

## 13. 第一轮实际执行清单

下一步从这里开始：

```text
[x] P0.1 修到 2D Pose Estimate、/odom、TF 稳定
[x] P0.2 让手画 FollowPath 能走完整短路径
[x] P1.1 创建 forklift_msgs
[x] P1.2 定义 ForkliftControlCommand / ForkliftVehicleState
[x] P2.1 创建 forklift_vehicle_interface
[x] P2.2 写仿真 bridge: ForkliftControlCommand -> cmd_vel
[x] P3.1 写 forklift_vehicle_model
[x] P4.1 把 ORU State / Control 概念移入 ForkliftMpcController
[x] P4.2 把 Path 转成内部 Trajectory
[x] P4.3 加 preview window
[x] P4.4 接最小 QP/MPC 求解
[x] P4.5 接 Nav2 Controller API，并通过 FollowPath / NavigateToPose 验收
[x] P5 接入轨迹处理和平滑
```

建议我们下一步先做：

```text
P6
```

原因：

- P5 已经能把稀疏 path 变成密集、连续、带曲率诊断和限速的 trajectory。
- 过急曲线现在会被提示并限速，但是否真的可通行还需要 P6 继续做 footprint/path feasibility。
- ORU qpOASES 求解器仍建议放到路径处理、约束和 footprint 检查稳定之后再接。

执行记录：

- 2026-06-09：P0.1 通过。headless Gazebo/Nav2 启动后 readiness gate 等到 `/clock`、`/odom`、`odom -> base_link`；发布 `/initialpose` 后 AMCL 响应，`map -> odom` 连续可查，`/controller_server` 和 `/planner_server` 均为 `active [3]`。
- 2026-06-09：P0.2 通过。使用 `/follow_path` 发送 `map` 坐标短直线路径 `(-2.0,-0.5) -> (-1.3,-0.5)`，`FollowPath` action 返回 `SUCCEEDED`，controller 日志显示 `Reached the goal!`。末态 `/odom` 约为 `x=-1.488, y=-0.500`，车辆速度接近 0。
- 备注：实测 `general_goal_checker.xy_goal_tolerance=0.08` 会被 AMCL/map 估计误差放大并触发 `Failed to make progress`，当前保留 Nav2 goal checker 基线 `0.25`；controller 内部 `FollowPath.xy_goal_tolerance` 仍为 `0.08`，并增加 terminal slowdown 以减少末端抢停/早停。
- 2026-06-10：P4.5 通过。`ForkliftMpcController` 保持 Nav2 `setPlan()`、`computeVelocityCommands()`、`setSpeedLimit()` 接口；bridge 模式下短直线 `/follow_path` 返回 `SUCCEEDED`，`NavigateToPose` 也返回 `SUCCEEDED`。`/forklift/control_cmd` 和 `/forklift/sim_cmd_vel` 均有连续输出并在末端停车。详见 `forklift_nav2_demo/docs/p4_5_nav2_controller_api_notes.md` 和 `log/p4_5_smoke/`。
- 2026-06-10：P5 通过。`ForkliftMpcController` 内部增加 path preprocessing：重复点过滤、0.10 m 插密、轻量 corner-cut smoothing、方向/曲率估计、最小转弯半径诊断和曲率限速。headless bridge 下短直线 `/follow_path`、温和大圆弧 `/follow_path`、稀疏 90 度 `/follow_path` 均返回 `SUCCEEDED`；90 度路径日志显示 `sharp_turns=1`，3 个输入点被处理成 15 个 trajectory 点。过急圆弧会打印曲率超限并限速，随后 progress checker abort，可作为后续 P6 feasibility 参考。详见 `forklift_nav2_demo/docs/p5_trajectory_preprocessing_notes.md` 和 `log/p5_smoke/`。
- 2026-06-11：P3/P4/P5 车辆几何补正通过。根据真车“双驱差速，90 度绕后轴旋转”的确认信息，继续沿 pivot-turn 方向而不是回到 Ackermann；`forklift_vehicle_model` 的 pivot predict 改为保持后轴点不动，并通过 `rear_axle_x_offset: -0.34` 表达当前 base reference 到后轴的偏移。Foxy docker 构建通过，`forklift_nav2_plugins` 相关 gtest 全部通过，headless bridge 下 `sparse_90_turn` acceptance 返回 `SUCCEEDED`。因此 P6 可以开始，但第一步建议做最小 `x/y/theta_index` lattice scaffold，并保留现有 costmap-aware A* fallback。详见 `forklift_nav2_demo/docs/foxy_pivot_turn_followup.md`。

## 14. ORU 包迁移优先级

优先移植：

```text
1. orunav_mpc
2. orunav_trajectory_processor
3. orunav_path_smoother
4. orunav_motion_planner
5. orunav_constraint_extract
```

暂时不移植，只参考：

```text
orunav_vehicle_execution
orunav_coordinator_fake
orunav_rviz
orunav_debug
orunav_pallet_detection_sdf
```

理由：

- `orunav_vehicle_execution` 是 ORU 自己的任务执行框架，和我们未来的 `forklift_task_manager + Nav2 actions` 职责重叠。
- `orunav_mpc` 的控制数学最有价值。
- `orunav_motion_planner` 的 primitive/lattice 思路很有价值，但要等 controller 和 vehicle interface 稳定后再做。
