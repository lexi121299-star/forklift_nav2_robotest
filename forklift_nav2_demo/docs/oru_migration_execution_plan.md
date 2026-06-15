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
    - 当前已经完成 P6.4a 最小倒车执行验证：`x/y/theta_index`、forward/reverse primitives、primitive 方向元数据、沿途 footprint collision、2D A* fallback、lattice cost/diagnostics，以及 reverse path 到 controller/bridge 的低速执行链路。
    - 还不是完整 ORU lattice/motion primitive planner；倒车场景调优、lookup/cache 和场景化 acceptance 仍待做。
  - `forklift_nav2_plugins/ForkliftMpcController`
    - 当前是 sampled predictive controller scaffold，不是完整 ORU QP-MPC。

当前主要问题：

- 仿真时间、`/clock`、TF、`/odom` 必须稳定，否则 2D Pose Estimate、costmap、FollowPath 都会失败。
- 当前 controller 已经有 ORU State / Control 概念、trajectory preview、最小 MPC 求解、path preprocessing 和 rear-axle pivot-turn 预测，但还不是完整 ORU QP-MPC。
- 当前 global planner 已经开始理解 heading、forward/reverse motion primitives、前进/倒车方向语义和换向代价；P6.4a 已在 runtime test 配置中低速打开 reverse，并通过 controller preview gating 避免普通 forward path 被倒车候选扰乱。

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
P8  加 safety，做急停、限速、防撞、掉边保护
P7  加 task_manager，统一 A-B 导航、手画 path、调度任务
P9  真车低速联调
```

编号仍保留 P7/P8，但 2026-06-12 起近期执行优先级调整为：

```text
[x] P6.3  reverse primitives + direction metadata
[x] P6.4a 最小倒车执行验证：controller/vehicle_interface 能执行倒车段
[x] P8.1  最小 safety gate：动态障碍停车/限速、急停、watchdog
[x] A-B acceptance 快速验证：空旷 NavigateToPose 简单 A-B
[x] A-B 动态障碍停车/放行快速验证
[x] P6.4b 倒车 acceptance 调优：当前最小 lattice scaffold 范围内，普通前进不乱倒、后方目标能倒车、90 度和动态障碍回归不退化
[ ] A-B acceptance 正式验收：普通路线、倒车/换向路线、障碍停车/放行
[ ] P8.2  独立 safety package / 命令闸门
[ ] P8.3  动态障碍等待、重新规划、简单绕行
[ ] P8.4  真车低速 safety acceptance 包
[ ] P2.3  真车 vehicle_interface 实际 I/O：底盘控制、反馈、急停、watchdog
[ ] P7.1  task_manager 最小任务入口
```

调整原因：

- 当前目标是先让车能稳定从简单 A 到 B，并且遇到动态障碍物能安全停下。
- 倒车是叉车运动能力的一部分，应该先在 planner/controller 闭环里打通。
- 动态障碍停车/限速是上车安全底线，应在最小倒车执行闭环后尽早完成，不等所有倒车场景调优结束。
- `forklift_task_manager` 负责站点、路线、任务暂停/恢复/取消和调度接口，可以等 A-B + safety gate 稳定后再做。
- `iliad/` 里的 human-aware navigation、HRSI、安全指标和 actor 仿真不属于当前 A-B + safety 主线，暂不作为 ORU 核心移植前置条件。

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
  lattice scaffold with x/y/theta_index, forward/reverse primitive metadata, A* fallback
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

### P6.5 当前分版路线

P6 不一次性做完整 ORU planner，按分版推进：

```text
[x] 第一版：最小 lattice scaffold
[x] 第二版：cost / diagnostics / goal approach tightening
[x] 第三版：reverse primitives + direction metadata
[x] 第四版 A：最小倒车执行验证
[ ] P8.1：最小 safety gate
[ ] 第四版 B：倒车 acceptance 调优
[ ] 第五版：multi-curvature / multi-length primitives + better heuristic + lookup/cache
[ ] 第六版：narrow aisle / docking / A-B scenario acceptance
```

第三版已完成 planner 侧最小闭环：

- 已加 `reverse straight / reverse left arc / reverse right arc`。
- 已为 primitive/transition 保留方向语义：
  - `forward`
  - `reverse`
  - `straight / left_arc / right_arc`
  - length / heading_delta
- 已加 `lattice_reverse_cost_multiplier` 和 `lattice_gear_switch_cost`。
- 已保留 A* fallback；P6.4a 之后 ORU test runtime 配置已低速打开 `lattice_reverse_enabled`，并通过 controller preview gating 只在 reverse-intent path 段允许负速度。

当前倒车链路说明：

- planner 搜索状态区分 `x/y/theta_index + arrival_direction`，前进到达和倒车到达不会被合并成同一种状态。
- `lattice_reverse_enabled=true` 时，planner 会在 forward straight / left arc / right arc 外，再加入 reverse straight / left arc / right arc。
- reverse primitive 会被 `lattice_reverse_cost_multiplier` 加价；forward/reverse 换向会被 `lattice_gear_switch_cost` 加价，所以 planner 不会无成本地乱用倒车。
- `nav_msgs/Path` 没有 gear 字段，当前用 path pose yaw 表达车体朝向；当 path yaw 与几何运动切线相差超过 90 度时，trajectory preprocessing 会标记 `reverse_motion`。
- controller 只有在 `allow_reverse=true`、`max_reverse_velocity>0`，并且当前 preview window 含 `reverse_motion` 点时，才允许负速度候选。
- 因此普通 forward path 即使全局打开 reverse，也不会在前进路线里随意倒车；P6.4a 的 `sparse_90_turn` 回归已验证 `forward=710 reverse=0`。

第三版之后不要直接等完整倒车场景都调好再做 P8。P6.4 拆成两段：

```text
P6.4a 最小倒车执行验证
P6.4b 倒车 acceptance 调优
```

P6.4a 在 P8.1 前完成，最低标准是：

- planner 可以输出倒车段。
- controller 不会把倒车段当成前进路径硬追。
- vehicle_interface / sim bridge 能收到正确方向的控制命令。
- 简单倒车 path 能低速执行，或至少能在仿真中正确发负速度。

P6.4a 不要求：

- 复杂窄通道倒车入库。
- 最优换向。
- 所有倒车场景稳定。

P6.4a 完成后，优先进入 P8.1 最小 safety gate。P6.4b 的倒车 acceptance 调优可以在 P8.1 后继续。

P6.4b 不从大改算法开始，而是先把倒车能力场景化验收，再根据失败现象调 planner/controller 参数和少量逻辑。

P6.4b 推荐 acceptance 场景：

```text
reverse_straight
  目标在车后方，验证 planner 输出 reverse，controller 输出负速度。

forward_straight
  目标在车前方，验证普通前进路线不乱倒车。

forward_with_goal_heading
  前方目标但终点姿态不同，验证不会为了贴终点姿态突然倒一下。

three_point_turn / narrow_turn
  空间不够直接掉头时，允许 forward + reverse 换向。

blocked_forward_reverse_escape
  前方被挡但后方可退，验证 planner 可以选择短倒车再前进。
```

每个场景至少记录：

```text
planner forward/reverse segment count
planner gear_switches
controller forward/reverse samples
/forklift/sim_cmd_vel 正负速度样本
action status
是否触发 Failed to make progress
路径是否出现横移、急转、不连续 heading
```

调优顺序：

1. 先补 acceptance 脚本和固定场景，不靠手工看 RViz 判断。
2. 先保证普通前进路线不随便倒车：
   - `forward_straight`、`sparse_90_turn`、A-B 普通路线应保持 `reverse_segments=0`、`reverse_samples=0`。
   - 只有目标在后方、空间受限或明确需要换向时，才允许 reverse。
3. 再稳定倒车段执行：
   - 倒车速度低速可控。
   - preview window 不频繁跳变。
   - reverse path heading 连续。
   - `/forklift/control_cmd` 和 `/forklift/sim_cmd_vel` 方向一致。
4. 再优化换向质量：
   - 避免短距离内 `forward/reverse/forward/reverse` 抖动。
   - 优先形成少量、明确的 `forward arc -> reverse arc -> forward arc`。
   - 必要时提高 `lattice_gear_switch_cost` 或加入 minimum segment length / suppress tiny gear changes。

优先检查和可调项：

```text
lattice_reverse_enabled
lattice_reverse_cost_multiplier
lattice_gear_switch_cost
lattice_goal_heading_weight / goal-heading heuristic
lattice_goal_tolerance / yaw tolerance
primitive length / arc radius
allow_reverse
max_reverse_velocity
respect_reverse_path_orientation
preview window 长度和 progress checker 参数
```

P6.4b 验收表：

```text
[x] forward route does not reverse
[x] rear goal uses reverse
[x] reverse segment reaches controller/bridge
[x] forward_with_goal_heading 不因为终点姿态引入倒车
[x] sparse_90_turn 回归不退化
[x] A-B dynamic obstacle 回归不退化
[>] three-point / narrow aisle / docking 正式场景放到 P10/P6.5
```

## 9. P7: 建 forklift_task_manager

当前优先级：

```text
P7 暂缓，放到 P6.4a、P8.1、A-B acceptance 和 P8.3 之后。
```

原因：

- 简单 A 到 B 可以先直接用 Nav2 `NavigateToPose` / `FollowPath` 验收，不需要 task_manager。
- 任务层不会直接提升车辆运动能力，也不会替代动态障碍安全停车。
- 等 planner/controller/safety 闭环稳定后，再做 task_manager 的暂停、恢复、重试和调度接口更稳。
- P7.1 进入前，至少要证明简单 A-B 能跑、动态障碍能停/放行、持续阻挡能等待或重新规划；否则 task_manager 只会把底层不稳定性包装成任务失败。

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

当前优先级：

```text
P8.1 已提前到 P7.1 之前完成；P8.2 再抽成独立 forklift_safety package。
```

第一阶段目标不是复杂动态绕行，而是安全停车/限速：

```text
动态障碍进入保护区 -> 停车
障碍物离开 -> 放行或允许 Nav2 继续
障碍物持续存在 -> 触发等待/后续重新规划策略
控制命令超时 -> 停车
急停输入 -> 停车
```

P8.1 当前落地：

- `ForkliftMpcController` 增加最小 safety gate，沿当前运动方向在 local costmap 上采样 footprint。
- 障碍进入 `safety_stop_distance` 时直接输出 brake/zero command。
- 障碍进入 `safety_slowdown_distance` 但还没到 stop zone 时压低当前方向速度上限。
- reverse-intent preview 时检查车后方保护区；普通 forward preview 时检查车前方保护区。
- `safety_emergency_stop_active` 参数可运行时置 true，让 controller 直接停车。
- `sim_command_bridge` 已有 `/forklift/set_emergency_stop` 和 `command_timeout_sec` watchdog，继续作为仿真/vehicle_interface 侧最后一道命令闸门。

P8.1 参数：

```yaml
safety_gate_enabled: true
safety_emergency_stop_active: false
safety_stop_distance: 0.55
safety_slowdown_distance: 1.25
safety_min_speed: 0.05
safety_sample_spacing: 0.10
```

P8.1 不是复杂动态绕行，也不是最终独立 safety 架构。它先给 A-B 运行加上可回退的最小停车/限速闸门；后续 P8.2 再把安全逻辑抽成独立 `forklift_safety` package 或接入 `nav2_collision_monitor`。

长期目标：

上车必须有独立安全层，不依赖 planner/controller 自觉避障。

后续新 package：

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
- 动态障碍物进入保护区时，对 controller 输出做安全闸门处理。

可结合 Nav2：

```text
nav2_collision_monitor
costmap filter / keepout mask
speed zone
local/global costmap obstacle observation
```

验收：

- 任意时候急停都能切断运动命令。
- `/cmd_vel` 超时自动停车。
- 障碍物进入保护区时停车或限速。
- 障碍物离开后，不需要重启 Nav2 即可继续执行或重新下发目标。
- 地图边缘/掉落风险区域不会允许继续行驶。

P8.2 最低标准：

- 把 controller 内的最小 safety gate 抽象成独立 `forklift_safety` package 或明确的命令闸门节点。
- controller、task_manager、手动控制等所有运动命令都必须经过 safety gate。
- 真车 recovery 命令也必须经过 safety gate；不允许 Nav2 默认 `/cmd_vel` 直接控制底盘。
- 明确 recovery command adapter：把允许的 recovery 动作转换成受限的 `ForkliftControlCommand` 或等价安全命令。
- 第一版只白名单低速、短时、可解释的 recovery primitive，例如 wait、clear-costmap 后重试、受限 pivot/backoff；所有 recovery 都要受急停、watchdog、速度上限、footprint collision 和车辆状态约束。
- 急停输入能锁住运动命令，解除后需要明确状态恢复。
- command timeout、vehicle fault、localization lost、costmap 数据异常时停车。
- 继续保留 controller-side 限速/停车作为可回退保护，不把安全完全交给 planner。

P8.3 最低标准：

- 动态障碍横穿或短时挡路时，车辆停车等待。
- 障碍离开后，Nav2 不需要重启即可继续执行，或由上层重新触发当前目标。
- 障碍持续挡住原路径时，触发重新规划。
- 定义真车 recovery 策略选择顺序：优先 wait，其次 clear/replan；只有安全闸门允许时才执行低速 pivot/backoff。
- recovery 失败时进入任务暂停/失败状态，不能反复执行可能扩大风险的动作。
- costmap 中存在足够通道时允许简单绕行；通道不足时不硬绕，保持等待或任务失败。
- 这一阶段不要求高级人群预测，也不要求复杂动态博弈避障。

P8.4 最低标准：

- 速度相关保护区：速度越快，前向/后向保护距离越大。
- 前进和倒车使用不同保护区，倒车时重点保护车尾与叉臂相关外形。
- footprint 覆盖车体、叉臂和必要安全余量。
- keepout 区、地图边缘、掉落风险区域不允许继续行驶。
- safety 日志和诊断能说明停车、限速、等待、失败的原因，便于真车低速复盘。

P8.1 到 P7.1 之间的推荐顺序：

```text
P8.1 已完成
-> A-B acceptance 快速验证已完成
-> A-B 动态障碍停车/放行快速验证已完成
-> P6.4b 倒车 acceptance 调优已完成
-> A-B acceptance 正式验收
-> P8.2 独立 safety gate
-> P8.3 动态障碍等待/重规划/简单绕行
-> P8.4 真车低速 safety acceptance
-> P7.1 task_manager 最小任务入口
```

快速 A-B acceptance 已证明空旷 `NavigateToPose` 链路可跑通；A-B 动态障碍停车/放行快速验证也已证明 P8.1 safety gate 在真实导航执行中能停车、放行后能继续跑完同一目标。P6.4b 已把当前最小 lattice scaffold 的倒车 acceptance 调稳。下一步建议做 A-B 正式验收，把普通路线、倒车/换向路线、障碍停车/放行放到同一套验收里。

真车 recovery 不是忽略项，但不要用仿真 bridge fallback 的方式直接上车。仿真 fallback 只是解决 Gazebo bridge 模式下 recovery `/cmd_vel` 到不了 `/forklift/sim_cmd_vel` 的接线问题；真车阶段应在 P8.2/P8.3 中实现受控 recovery：

```text
Nav2/controller normal command
  -> safety gate
  -> vehicle_interface
  -> 真车底盘

Nav2/custom recovery request
  -> recovery command adapter
  -> safety gate
  -> vehicle_interface
  -> 真车底盘
```

原则：

- 真车底盘不裸订阅 Nav2 默认 `/cmd_vel`。
- recovery 可以执行，但必须转换成叉车约束下的安全命令。
- 所有 recovery 命令都要经过急停、watchdog、限速、footprint collision、车辆状态和传感器健康检查。
- P8.2 解决“recovery 命令怎么安全进真车”；P8.3 解决“什么情况下执行哪一种 recovery”。

动态障碍停车/放行快速验证最低标准：

- `NavigateToPose` 正在执行时，障碍进入 slowdown 区，速度被压低。
- 障碍进入 stop 区，controller 输出停车或 brake。
- 障碍离开后，不重启 Nav2，车辆能继续执行；如果当前行为树无法自然继续，至少能重新下发同一目标继续。
- 无障碍时，P8.1 safety gate 不误停。
- 本阶段不要求复杂绕行；持续阻挡时允许等待或任务失败，但不能硬撞或继续推障碍。

如果后续正式验收暴露 safety gate 架构边界，再进入 P8.2 独立 safety package；如果只是更复杂的倒车、换向、窄通道路径质量问题，优先进入 P10/P6.5 的 primitive/heuristic/场景增强。

## 11. P9: 真车低速联调

第一版上真车前的门槛要分清三层：

```text
台架/离地硬件联调:
  可以在 P2.3 完成后做，不启动 Nav2。

地面低速手动控制:
  P2.3 + 急停 + watchdog + /odom + TF 稳定后做。

第一版 Nav2 自主 A-B:
  必须完成到 P8.4；P7.1、P10+ 和 ILIAD 都不是前置条件。
```

第一版 Nav2 自主 A-B 上真车前必须完成：

- P2.3 真车 `forklift_vehicle_interface` 实际 I/O：不再只是 dry-run 日志，必须能向底盘发送控制，并发布真实反馈。
- A-B acceptance 快速验证和正式验收：普通路线、倒车/换向路线、障碍停车/放行都在仿真中通过。
- P6.4b 倒车 acceptance：当前最小 lattice scaffold 中普通前进路线不乱倒、后方目标能倒、90 度和动态障碍回归不退化；更复杂的窄通道/三点掉头/倒车入库放到 P10/P6.5。
- P8.2 独立 safety gate 或明确命令闸门：所有运动命令经过安全层。
- P8.3 动态障碍等待、重新规划、简单绕行和受控 recovery 策略：能停、能等、能放行，通道不足时不硬绕。
- P8.4 真车低速 safety acceptance：速度相关保护区、前进/倒车不同保护区、keepout、地图边缘、诊断日志。
- 真实地图、真实传感器、真实 vehicle interface 参数都已经替换仿真默认值。

P7.1 不作为第一版上车前置条件。第一版可以直接用 Nav2 `NavigateToPose`、`FollowPath` 和 RViz/CLI 下发目标；等底层跑稳后，再加 `forklift_task_manager` 统一站点、路线、暂停、恢复、取消和调度接口。

第一版上真车前必须通过的仿真测试：

- `forklift_nav2_plugins` 单元测试全部通过，包括 planner、controller、trajectory、preview window、vehicle model、safety gate。
- headless Nav2/Gazebo readiness：`/clock`、`/odom`、`odom -> base_link`、2D Pose Estimate 后 `map -> odom`、Nav2 lifecycle active。
- `ComputePathToPose`：普通前进目标、90 度目标、后方倒车目标都能规划成功。
- `FollowPath`：短直线、大圆弧、稀疏 90 度、简单倒车 path 都能低速执行成功。
- `NavigateToPose`：空旷 A-B、含 90 度转弯 A-B、含一次倒车/换向 A-B 都能成功或给出可解释失败。
- 动态障碍测试：障碍进入保护区停车/限速，障碍离开放行，持续阻挡触发等待或重新规划。
- safety 回归：急停 service/输入触发停车，命令超时停车，local costmap 无障碍时不误停。
- 传感器异常测试：`/scan` 丢失、`/odom` 丢失、TF 超时、costmap stale 时进入停车或任务失败状态。
- 地图边界/keepout 测试：不会规划到地图外、未知区、掉边风险区或禁止通行区。

第一版真车实际会用到的本项目 package：

```text
forklift_msgs
  共享控制、车辆状态、故障、IO、急停和模式 service。

forklift_vehicle_interface
  真车底盘接口。第一版上车前必须把 curtis_vehicle_interface 从 dry-run
  补成真实 CAN/串口/TCP I/O，并发布 /odom、TF、vehicle_state、fault_state、io_state。

forklift_nav2_plugins
  OruGlobalPlanner、ForkliftMpcController、trajectory preprocessing、
  controller-side safety gate。第一版上车必须使用。

forklift_nav2_demo
  当前仍承载 launch、Nav2 参数、地图、URDF、仿真入口。第一版可以继续用，
  但真车应新增 real bringup launch，不能启动 Gazebo。

forklift_safety
  P8.2 后新增或等价实现。第一版 Nav2 自主 A-B 前应作为安全命令闸门存在。
```

第一版上车前需要改动的 package：

- `forklift_vehicle_interface`：最大改动点。补真实 CAN/串口/TCP 传输、反馈解析、`/odom`、TF、`vehicle_state`、`fault_state`、`io_state`、急停、模式切换、watchdog。
- `forklift_nav2_demo`：新增 real launch 和 real 参数文件，真车启动不带 Gazebo，`use_sim_time=false`，地图路径、传感器 topic、速度限制、safety 参数使用真车 profile。
- `forklift_nav2_plugins`：原则上保持算法 plugin，不写底盘协议；只根据 A-B/P8 acceptance 调 planner/controller/safety 参数，必要时补真实场景暴露的碰撞、倒车、限速逻辑。
- `forklift_safety`：P8.2 新增或等价实现，作为真车运动命令闸门。
- `forklift_msgs`：尽量保持稳定；只有真实底盘反馈或 IO 状态字段不够时才扩展消息。

第一版上车前不要改：

- 不改 `/opt/ros/*` 里的 Nav2 源码。
- 不把底盘协议写进 `forklift_nav2_plugins`。
- 不把 `navigation_oru-release` 或 `iliad` 直接接到真车主启动链路。

第一版真车不需要启动或迁移：

```text
navigation_oru-release/*
iliad/*
forklift_sim
forklift_warehouse_sim
orunav_vehicle_execution
orunav_coordinator_fake
```

`forklift_sim` 和 `forklift_warehouse_sim` 只是早期轻量仿真/开发包，不作为第一版真车或正式 Nav2 acceptance 的主入口。

真车建议启动链路：

```text
1. 传感器 driver
   发布 /scan，必要时发布 rear scan 或 360 度 scan。

2. forklift_vehicle_interface
   订阅 /forklift/control_cmd。
   发布 /odom、odom -> base_link TF、/forklift/vehicle_state、
   /forklift/fault_state、/forklift/io_state。
   提供 /forklift/set_emergency_stop、/forklift/set_control_mode。

3. robot_state_publisher 或 static TF
   发布 base_link -> base_footprint、base_link -> base_scan、
   以及其他传感器安装位姿。

4. Nav2 bringup
   map_server、AMCL、planner_server、controller_server、bt_navigator、
   local/global costmap、recoveries。
   参数使用 real profile，use_sim_time=false。

5. forklift_safety
   P8.2 后作为所有运动命令的最后安全闸门。

6. RViz
   只作为调试和初始位姿/目标下发工具，不作为安全链路。
```

当前 `forklift_nav2_demo/launch/forklift_navigation.launch.py` 会启动 Gazebo，不适合真车直接使用。上车前应新增：

```text
forklift_nav2_demo/launch/forklift_real_navigation.launch.py
```

或后续拆成：

```text
forklift_bringup/launch/real_navigation.launch.py
forklift_description
```

真车 real launch 最低要求：

- 不 include `forklift_gazebo.launch.py`。
- `use_sim_time=false`。
- 启动或接入真实 `forklift_vehicle_interface`。
- 启动 robot_state_publisher/static TF。
- 启动 Nav2 bringup，并加载真实地图和 real 参数文件。
- 可选启动 RViz，但 RViz 不参与安全闭环。

仿真主入口：

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_time:=true \
  use_rviz:=false \
  gazebo_gui:=false \
  use_sim_command_bridge:=true
```

仿真会用到：

- `forklift_nav2_demo`：Gazebo world、URDF、spawn、Nav2 launch、wait_for_sim_ready。
- `forklift_vehicle_interface`：`sim_command_bridge`，把 `/forklift/control_cmd` 转成 Gazebo `/forklift/sim_cmd_vel`。
- `forklift_nav2_plugins`：planner/controller/safety gate。
- `forklift_msgs`：共享控制和状态消息。
- Nav2、Gazebo、robot_state_publisher、AMCL、map_server、costmap。

地图格式要求：

- 第一版继续使用 Nav2 标准 2D occupancy grid 地图。
- 文件格式是 `map.yaml + .pgm`，例如：

```yaml
image: forklift_factory_big_map_clean.pgm
mode: trinary
resolution: 0.05
origin: [-12.4, -9.02, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- `map.yaml` 描述图片、分辨率、原点和阈值；`.pgm` 保存占据栅格。
- 地图坐标系是 `map`，AMCL 发布 `map -> odom`。
- 地图只表达静态环境：墙、货架、固定禁行区域。动态障碍不要画进静态地图。
- keepout、限速区、掉边风险区后续用 costmap filter 或 safety layer 表达，不直接涂进普通 occupancy map。
- 上真车前需要用真实场地重新建图或校准当前地图，确认分辨率、原点、方向和 RViz 中实际位置一致。

第一版必须输入的数据和话题：

```text
/scan
  sensor_msgs/LaserScan
  2D 激光雷达输入，当前 Nav2 local/global costmap 和 AMCL 都使用 scan。
  frame 建议为 base_scan，并通过 TF 固定到 base_link。

/odom
  nav_msgs/Odometry
  来自 forklift_vehicle_interface 或可靠里程计融合。
  必须和 odom -> base_link TF 一致。

odom -> base_link TF
  真车里程计坐标变换。方向、速度符号、时间戳必须稳定。

base_link -> base_scan TF
  激光安装外参。必须和真实安装位置一致。

base_link / base_footprint
  当前 AMCL 配置使用 base_footprint，costmap 使用 base_link。
  两者需要固定 TF 或统一配置，不能缺一条。

/forklift/control_cmd
  ForkliftControlCommand，controller 或 safety gate 输出给 vehicle_interface。

/forklift/vehicle_state
/forklift/fault_state
/forklift/io_state
  真车反馈、故障、IO、安全输入状态。

/forklift/set_emergency_stop
/forklift/set_control_mode
  急停和控制模式 service。

/initialpose
  geometry_msgs/PoseWithCovarianceStamped，AMCL 初始位姿。

NavigateToPose / FollowPath action
  第一版 A-B 和固定路径测试入口。
```

第一版传感器最低要求：

- 2D 激光雷达，能覆盖前进方向的局部避障区域，发布 `sensor_msgs/LaserScan`。
- 如果真车第一版要测试倒车，必须有后向雷达、360 度雷达，或能覆盖车尾/叉臂风险区的等价传感器；否则真车倒车必须限速并限制场景。
- 轮速/编码器/底盘反馈，能生成稳定 `/odom` 和 `odom -> base_link`。
- 转向角或等价运动状态反馈。如果底盘没有真实转向角，也要在 `ForkliftVehicleState` 中提供 controller/safety 可解释的状态。
- 急停、自动/手动模式、停车制动、故障码、通信状态输入。
- IMU 不是第一版硬要求，但如果 odom yaw 漂移明显，应接入 IMU 或 robot_localization。
- 3D 点云/相机不是第一版硬要求；如果后续使用 PointCloud2，需要同步修改 Nav2 costmap observation source。

上车前需要你确认或提供的输入：

- 真车底盘通信协议：CAN ID、字节定义、比例系数、方向/刹车/使能语义、反馈帧、故障帧。
- 车辆几何：车体长宽、叉臂长度、base_link 位置、后轴/旋转中心偏移、最小转弯/原地转能力。
- 速度和加速度限制：前进、倒车、转向角速度、最大角速度、急停/减速时间。
- 传感器安装位姿：雷达相对 base_link 的 x/y/z/yaw，是否有后向或 360 度覆盖。
- 地图：真实场地 `map.yaml + pgm`，或允许重新 SLAM 建图。
- 安全区域：禁行区、掉边区域、限速区、测试区域边界。
- 上车测试流程：谁看急停、最大速度、测试路线、允许倒车的区域、失败时如何人工接管。

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
[ ] P2.3 真车 vehicle_interface 实际 I/O：底盘控制、反馈、急停、watchdog
[x] P3.1 写 forklift_vehicle_model
[x] P4.1 把 ORU State / Control 概念移入 ForkliftMpcController
[x] P4.2 把 Path 转成内部 Trajectory
[x] P4.3 加 preview window
[x] P4.4 接最小 QP/MPC 求解
[x] P4.5 接 Nav2 Controller API，并通过 FollowPath / NavigateToPose 验收
[x] P5 接入轨迹处理和平滑
[x] P6.1 最小 lattice planner scaffold：`x/y/theta_index`、forward primitives、沿途 footprint collision、A* fallback
[x] P6.2 lattice 代价/诊断第二版：turn/obstacle/goal-heading cost、拒绝原因统计、goal tolerance 收紧
[x] P6.3 reverse primitives + direction metadata：倒车 primitive、方向语义、reverse/gear switch cost
[x] P6.4a 最小倒车执行验证：确认 planner 输出的倒车段能被 controller/vehicle interface 执行
[x] P8.1 最小 safety gate：动态障碍停车/限速、急停、watchdog
[x] A-B acceptance 快速验证：空旷 NavigateToPose 简单 A 到 B
[x] A-B 动态障碍停车/放行快速验证
[x] P6.4b 倒车 acceptance 调优：当前最小 lattice scaffold 范围内，普通前进不乱倒、后方目标能倒车、90 度和动态障碍回归不退化
[ ] A-B acceptance 正式验收：普通路线、倒车/换向路线、障碍停车/放行
[ ] P8.2 独立 safety package / 命令闸门
[ ] P8.3 动态障碍等待、重新规划、简单绕行
[ ] P8.4 真车低速 safety acceptance 包
[ ] P7.1 task_manager 最小任务入口
[ ] P10 motion planner lookup/primitives 加强
[ ] P11 path smoother + constraint_extract
[ ] P12 评估是否上完整 ORU QP-MPC
[ ] P13 需要 human-aware 能力时，再评估 ILIAD 相关包
```

建议我们下一步继续做：

```text
A-B acceptance 正式验收缺口收敛
```

原因：

- P6.4b 已经把当前最小 lattice scaffold 的倒车 acceptance 固化成脚本，并完成前进、倒车、90 度和动态障碍回归。
- P8.1 已经加上最小 safety gate、急停参数和 bridge watchdog 基线。
- 空旷 A-B 快速验证和动态障碍停车/放行快速验证都已经通过。
- A-B 正式验收已经开始：普通前进和后方目标倒车通过；90 度 `NavigateToPose` 闭环和动态障碍正式复测仍有缺口。
- 真车 recovery 命令闸门已经明确放入 P8.2/P8.3；但在进入真车 safety 架构前，先把 planner/controller 的基础运动能力调稳，能减少后续 safety/recovery 层需要兜底的问题。

执行记录：

- 2026-06-09：P0.1 通过。headless Gazebo/Nav2 启动后 readiness gate 等到 `/clock`、`/odom`、`odom -> base_link`；发布 `/initialpose` 后 AMCL 响应，`map -> odom` 连续可查，`/controller_server` 和 `/planner_server` 均为 `active [3]`。
- 2026-06-09：P0.2 通过。使用 `/follow_path` 发送 `map` 坐标短直线路径 `(-2.0,-0.5) -> (-1.3,-0.5)`，`FollowPath` action 返回 `SUCCEEDED`，controller 日志显示 `Reached the goal!`。末态 `/odom` 约为 `x=-1.488, y=-0.500`，车辆速度接近 0。
- 备注：实测 `general_goal_checker.xy_goal_tolerance=0.08` 会被 AMCL/map 估计误差放大并触发 `Failed to make progress`，当前保留 Nav2 goal checker 基线 `0.25`；controller 内部 `FollowPath.xy_goal_tolerance` 仍为 `0.08`，并增加 terminal slowdown 以减少末端抢停/早停。
- 2026-06-10：P4.5 通过。`ForkliftMpcController` 保持 Nav2 `setPlan()`、`computeVelocityCommands()`、`setSpeedLimit()` 接口；bridge 模式下短直线 `/follow_path` 返回 `SUCCEEDED`，`NavigateToPose` 也返回 `SUCCEEDED`。`/forklift/control_cmd` 和 `/forklift/sim_cmd_vel` 均有连续输出并在末端停车。详见 `forklift_nav2_demo/docs/p4_5_nav2_controller_api_notes.md` 和 `log/p4_5_smoke/`。
- 2026-06-10：P5 通过。`ForkliftMpcController` 内部增加 path preprocessing：重复点过滤、0.10 m 插密、轻量 corner-cut smoothing、方向/曲率估计、最小转弯半径诊断和曲率限速。headless bridge 下短直线 `/follow_path`、温和大圆弧 `/follow_path`、稀疏 90 度 `/follow_path` 均返回 `SUCCEEDED`；90 度路径日志显示 `sharp_turns=1`，3 个输入点被处理成 15 个 trajectory 点。过急圆弧会打印曲率超限并限速，随后 progress checker abort，可作为后续 P6 feasibility 参考。详见 `forklift_nav2_demo/docs/p5_trajectory_preprocessing_notes.md` 和 `log/p5_smoke/`。
- 2026-06-11：P3/P4/P5 车辆几何补正通过。根据真车“双驱差速，90 度绕后轴旋转”的确认信息，继续沿 pivot-turn 方向而不是回到 Ackermann；`forklift_vehicle_model` 的 pivot predict 改为保持后轴点不动，并通过 `rear_axle_x_offset: -0.34` 表达当前 base reference 到后轴的偏移。Foxy docker 构建通过，`forklift_nav2_plugins` 相关 gtest 全部通过，headless bridge 下 `sparse_90_turn` acceptance 返回 `SUCCEEDED`。因此 P6 可以开始，但第一步建议做最小 `x/y/theta_index` lattice scaffold，并保留现有 costmap-aware A* fallback。详见 `forklift_nav2_demo/docs/foxy_pivot_turn_followup.md`。
- P6 最小 lattice scaffold 的范围说明见 `forklift_nav2_demo/docs/p6_lattice_planner_scaffold_notes.md`。
- 2026-06-11：P6.1 最小 lattice scaffold 通过。`OruGlobalPlanner` 增加可开关 `x/y/theta_index` lattice 搜索，保留原 2D A* fallback；第一版只启用 forward straight / left arc / right arc primitives，并对 primitive 沿途采样做 costmap + footprint collision。Foxy docker 构建通过，`forklift_nav2_plugins` 6 个 gtest 全部通过；headless `ComputePathToPose` smoke 中 planner server 加载 `use_lattice=true`，日志显示 `Lattice planner produced 5 states`，action 返回 `SUCCEEDED`，path heading 从起点逐步过渡到 90 度。详见 `forklift_nav2_demo/docs/p6_lattice_planner_scaffold_notes.md`。
- 2026-06-11：P6.2 lattice 第二版通过。保持 forward-only primitives 和 2D A* fallback，新增 lattice search 统计日志、primitive 拒绝原因分类、沿途 sample obstacle cost、turn cost、goal-heading heuristic，以及 `lattice_goal_tolerance` 收紧目标收尾。Foxy docker 构建通过；`forklift_nav2_plugins` 6 个测试目标全部通过，其中 `test_oru_global_planner` 扩展到 6 个用例；headless `ComputePathToPose` 90 度 smoke 返回 `SUCCEEDED`，planner 日志显示 `Lattice search succeeded: expanded=7 generated=18 accepted=18 improved=18 rejected_oob=0 rejected_costmap=0 rejected_footprint=0 best_goal_distance=0.050` 和 `Lattice planner produced 5 states`。第二版/第三版目标、步骤和解释见 `forklift_nav2_demo/docs/p6_lattice_planner_scaffold_notes.md`。
- 2026-06-12：迁移优先级调整。根据当前目标“简单 A 到 B 能跑起来，并且动态障碍物至少能安全停下”，P7 task_manager 暂缓；近期路线改为先做 P6.3 reverse primitives + direction metadata，再做 P6.4a 最小倒车执行验证，然后提前进入 P8.1 最小 safety gate。P6.4b 的倒车 acceptance 调优和 P7.1 task_manager 放在 safety gate 之后。编号保留 P7/P8，但实际执行顺序以本清单为准。
- 2026-06-12：P6.3 reverse primitives + direction metadata 通过。`OruGlobalPlanner` 的 lattice transition 现在携带 `direction`、`primitive_kind`、`length`、`heading_delta`；`lattice_reverse_enabled=true` 时会生成 `reverse straight / reverse left arc / reverse right arc`，搜索 key 在 `x/y/theta_index` 外区分 arrival direction，避免 gear-switch cost 错误合并不同到达方向；搜索会保留到达每个 state 的 primitive 元数据并记录 forward/reverse 段和 gear switch 数量；新增 `lattice_reverse_cost_multiplier` 和 `lattice_gear_switch_cost`。运行配置仍保持 `lattice_reverse_enabled: false`，把真实倒车执行留给 P6.4a。Foxy docker 构建通过；`forklift_nav2_plugins` 49 个 gtest 全部通过，其中 `test_oru_global_planner` 扩展到 8 个用例，覆盖 reverse primitive gating/metadata 和 reverse/gear-switch cost。
- 2026-06-12：P6.4a 最小倒车执行验证通过。`ForkliftMpcController` 新增 `respect_reverse_path_orientation`，在 path pose yaw 与运动切线相反时保留车体朝向并标记 `reverse_motion`；controller 只在当前 preview window 含 reverse-intent 点时允许负速度候选，避免普通 forward path 末端被倒车微调扰乱。ORU test 配置低速打开 `allow_reverse=true`、`max_reverse_velocity=0.15`、`lattice_reverse_enabled=true`。Foxy docker 构建通过；`forklift_nav2_plugins` 51 个 gtest 全部通过；headless `ComputePathToPose` 后方目标 smoke 返回 `SUCCEEDED`，planner 日志显示 `forward=0 reverse=1`；`reverse_straight` FollowPath 返回 `SUCCEEDED`，观测到 `control_direction_samples forward=0 reverse=10` 和 `/forklift/sim_cmd_vel.linear.x=-0.150`；`sparse_90_turn` 回归返回 `SUCCEEDED`，观测到 `forward=710 reverse=0`，说明 forward path 不再启用倒车候选。
- 2026-06-15：P8.1 最小 safety gate 通过。`ForkliftMpcController` 增加 controller-side safety gate，沿当前 forward/reverse 运动方向在 local costmap 上采样 footprint；障碍进入 `safety_stop_distance` 时直接 brake，进入 `safety_slowdown_distance` 时压低当前方向速度上限；`safety_emergency_stop_active` 可运行时置 true 触发 controller 停车。ORU test 配置打开 `safety_gate_enabled=true`，并保留 `sim_command_bridge` 既有 `/forklift/set_emergency_stop` 和 `command_timeout_sec` watchdog 作为 vehicle_interface 侧闸门。Foxy docker 构建通过；`forklift_nav2_plugins` 59 个 gtest 全部通过，其中新增 `test_forklift_safety_gate` 5 个用例；headless `sparse_90_turn` safety gate 回归返回 `SUCCEEDED`，日志确认 `safety_gate=true`，观测到 `control_direction_samples forward=560 reverse=0` 和 `max_linear_x=0.450`，说明无障碍时不会误停。详见 `forklift_nav2_demo/docs/p8_safety_gate_notes.md`。
- 2026-06-15：A-B acceptance 快速验证通过。Foxy docker headless + Gazebo + Nav2 + sim bridge + P8.1 safety gate 打开，从 `(-2.0, -0.5)` NavigateToPose 到 `(-0.9, -0.5)`，action 返回 `SUCCEEDED`；观测到 `feedback_count=397`、`control_samples=31`、`max_velocity_mps=0.395`、`control_direction_samples forward=31 reverse=0`、`sim_cmd_samples=100`、`max_linear_x=0.395`、末态 odom 约 `x=-1.066 y=-0.571`。planner 多次重规划均输出 forward-only lattice path，例如 `forward=5 reverse=0`、`forward=3 reverse=0`、`forward=1 reverse=0`；日志确认 `safety_gate=true safety_stop=0.550 safety_slowdown=1.250`。本次只覆盖空旷简单 A-B，不覆盖动态障碍停车/放行。
- 2026-06-15：A-B 动态障碍停车/放行快速验证通过。新增 `forklift_ab_dynamic_obstacle_acceptance` 脚本：下发 `NavigateToPose`，在路线中间生成临时 Gazebo box 障碍，观察 safety gate 停车，删除障碍后等待 Nav2 自动恢复并完成目标。`sim_command_bridge` 增加可选 `twist_fallback_topic`，在 `/forklift/control_cmd` 超时后把 recovery `/cmd_vel` 转发到 `/forklift/sim_cmd_vel`，解决 bridge 模式下 Nav2 recovery spin 只发 `/cmd_vel`、Gazebo 不动的问题；正常控制仍优先使用 `/forklift/control_cmd`。Foxy docker 构建通过；`forklift_nav2_plugins` 59 个 gtest 全部通过；headless 动态障碍验收从 `(-2.0, -0.5)` 到 `(1.2, -0.5)` 返回 `SUCCEEDED`，脚本打印 `dynamic_obstacle_acceptance=PASS`，观测到 `control_samples=399`、`sim_cmd_samples=1136`、`phase_control_zero blocked=4`、`phase_sim_max after=0.450`、`forward=94 reverse=0`。launch 日志确认 `P8.1 safety gate stopping: obstacle at 0.100 m in forward protection zone`，障碍释放后 planner 继续输出 forward-only lattice path，recovery 期间 bridge 日志出现 `Using fallback Twist command.`。
- 2026-06-15：P6.4b 倒车 acceptance 调优通过。新增 `forklift_p6_reverse_acceptance`：脚本先低频发布 `/initialpose` 并等待 `map -> base_link`，再调用 `ComputePathToPose`，从 path yaw 与几何切线推断 `forward_segments / reverse_segments / gear_switches`，需要执行的场景会把 planner path 交给 `FollowPath` 并记录 `/forklift/control_cmd` 与 `/forklift/sim_cmd_vel` 的方向样本。脚本在交给 controller 前清空 path stamps，避免 Foxy 中 ComputePath 返回旧 stamp 导致 `ForkliftMpcController could not transform the global plan`。`forklift_ab_dynamic_obstacle_acceptance` 也补入同样的 initial pose 初始化，减少外部 shell 对 `/initialpose` 的依赖。本轮验收结果：
  - `forward_straight`：`ComputePathToPose` 和 `FollowPath` 均 `SUCCEEDED`；`poses=6 forward_segments=5 reverse_segments=0 gear_switches=0`；`control_direction_samples forward=31 reverse=0`；`sim_cmd_vel` 无负速度。
  - `reverse_straight`：`ComputePathToPose` 和 `FollowPath` 均 `SUCCEEDED`；`poses=2 forward_segments=0 reverse_segments=1 gear_switches=0`；`control_direction_samples forward=0 reverse=17`；`min_signed_linear_x=-0.096`。
  - `sparse_90_turn`：`ComputePathToPose` 和 `FollowPath` 均 `SUCCEEDED`；`poses=5 forward_segments=4 reverse_segments=0 gear_switches=0`；`control_direction_samples forward=158 reverse=0`。
  - `forward_with_goal_heading`：planner-only `SUCCEEDED`；`poses=5 forward_segments=4 reverse_segments=0 gear_switches=0`，说明前方目标带终点 yaw 时不会为了贴姿态引入倒车。
  - A-B 动态障碍回归：`NavigateToPose` `SUCCEEDED`，`dynamic_obstacle_acceptance=PASS`；`control_samples=389 sim_cmd_samples=1169 forward=85 reverse=0`，说明 P8.1 动态障碍停车/放行未被 P6.4b acceptance 改动破坏。
  P6.4b 的完成边界是“当前最小 lattice scaffold 的倒车验收可回归”：普通前进路线不乱倒，后方目标会倒，controller/bridge 能执行倒车段，90 度和动态障碍回归不退化。窄通道、三点掉头、倒车入库等正式场景需要更多曲率/长度 primitive、场景生成和更强 heuristic，放到 P10/P6.5，不作为本次 P6.4b 的通过条件。
- 2026-06-15：A-B acceptance 正式验收开始，暂未通过。新增 `forklift_ab_acceptance` 统一入口，脚本支持 Gazebo `/set_entity_state` 重置 `forklift` 实体、重新发布 `/initialpose`、记录 `/forklift/control_cmd` 和 `/forklift/sim_cmd_vel`，并覆盖 `forward_ab`、`reverse_ab`、`sparse_90_turn_ab`、`dynamic_stop_release_ab` 场景。为抑制普通前进路线中不必要倒车，ORU test 配置把 `lattice_reverse_cost_multiplier` 从 `0.5` 提到 `3.0`，`lattice_gear_switch_cost` 从 `1.0` 提到 `4.0`。当前验收结果：
  - `forward_ab`：`NavigateToPose` `SUCCEEDED`；`control_samples=80 sim_cmd_samples=248 forward=80 reverse=0`；`sim_cmd max_signed_linear_x=0.450`；`odom_final x=0.928 y=-0.544`；`ab_acceptance=PASS scenario=forward_ab`。
  - `reverse_ab`：`NavigateToPose` `SUCCEEDED`；`control_samples=20 sim_cmd_samples=78 forward=1 reverse=19`；`sim_cmd min_signed_linear_x=-0.096`；`odom_final x=-2.159 y=-0.504`；`ab_acceptance=PASS scenario=reverse_ab`。单个 forward 样本在验收阈值内，主路径仍是倒车执行。
  - `sparse_90_turn_ab`：调参前失败时出现 `forward=115 reverse=223`，说明重规划会用倒车贴终点姿态；调参后不再出现倒车，`forward=257 reverse=0`，但 `NavigateToPose` 仍 `ABORTED`，末态 `odom_final x=-1.147 y=-1.325`。这说明 90 度 FollowPath 已通过，但 90 度 `NavigateToPose` 闭环仍未达到正式 A-B 验收标准，后续需要继续调 planner/controller/replanning 行为，而不是把它视为通过。
  - `dynamic_stop_release_ab`：新统一脚本暂未通过，专用 `forklift_ab_dynamic_obstacle_acceptance` 复测又被一次 Foxy `map_server`/lifecycle crash 打断，未得到有效动态复测结论。动态障碍快速验证的既有通过记录仍有效，但正式验收需要下一轮在干净 launch 下复测并固化。
  当前结论：A-B 正式验收未完成；普通前进和后方目标倒车已经通过，90 度闭环和动态障碍正式复测是下一轮 blockers。

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

P8.4 / P7.1 之后，ORU 核心算法还剩这些主要部分：

```text
1. 完整 ORU QP-MPC
2. 更完整的 ORU motion planner / primitives / lookup tables
3. trajectory processor / path smoother
4. constraint_extract / 几何约束提取
5. ORU debug / rviz / execution 框架只参考，不作为主线迁移
```

完整 ORU QP-MPC：

- 当前 `ForkliftMpcController` 已有 ORU 的 State / Control 概念、trajectory preview、path preprocessing 和 sampled predictive controller scaffold。
- 当前还不是完整 ORU `qpProblem / qpConstraints / qpOASES`。
- 是否迁移完整 QP-MPC，取决于 sampled controller 在真车低速场景是否足够稳定。
- 如果低速 A-B、倒车、换向、末端停车已经满足需求，可以继续保留 sampled controller。
- 如果速度提高、路径更复杂或末端控制不够顺，再迁移 ORU QP-MPC。

更完整的 ORU motion planner：

- 当前 P6 已经实现 ORU-inspired lattice：`x/y/theta_index`、forward/reverse primitives、gear switch cost、footprint collision。
- 还不是完整 ORU motion planner。
- 后续可补 ORU primitive 文件加载、lookup table、multi-curvature / multi-length primitives、更强 heuristic、lookup/cache 加速。
- 重点验收窄通道、倒车、换向次数、贴边距离和路径稳定性。

trajectory processor / path smoother：

- 当前 controller 内部已有轻量 path preprocessing。
- 后续如果要靠近 ORU，应把路径转轨迹、方向语义、曲率、速度约束和平滑从 controller 内部逐步抽出来。
- `orunav_path_smoother` 的约束优化思路有价值，但 ACADO 依赖成本高，不作为第一步硬移植目标。

constraint_extract / 几何约束：

- 可用于从 costmap 提取局部可行 corridor。
- 可用于更严格的 footprint、叉臂、车尾碰撞检查。
- 可给 smoother 或 MPC 提供边界约束，让路径不只是“不撞”，而是保持合理障碍余量。

`iliad/` 当前处理策略：

- `iliad/` 暂时不移植。
- `iliad_human_aware_navigation`、`iliad_hrsi`、`iliad_safety_metrics` 更偏 human-aware navigation、HRSI、人车交互和安全指标。
- `moving_actor_gazebo`、`gazebo_plugin_actor_collision`、`iliad_base_simulation` 更偏 ROS1/Gazebo actor 仿真环境。
- `iliad_goal_manager`、`iliad_init_pose_manager` 和未来 `forklift_task_manager + Nav2 actions` 有部分职责重叠，不作为当前主线入口。
- 只有当后续明确要做人群附近通行策略、人车交互安全指标或动态人行为预测时，再从 `iliad/` 里选模块参考或局部迁移。

P8.4 / P7.1 之后的建议 ORU 后续顺序：

```text
P10  motion planner lookup/primitives 加强
P11  path smoother + constraint_extract
P12  评估是否上完整 ORU QP-MPC
P13  需要 human-aware 能力时，再评估 ILIAD 相关包
```
