# Forklift ROS2 System Architecture and ORU Integration Roadmap

这份文档用于规划后续把当前 `forklift_nav2_demo` 仿真项目升级成一套可以逐步上真车的 ROS2 forklift 导航系统。

核心原则：

- 不直接修改 `/opt/ros/humble` 里的 Nav2 源码。
- Nav2 继续作为导航系统框架，负责 lifecycle、BT navigator、planner/controller/smoother server、costmap、map server、AMCL 等基础能力。
- ORU 里的算法先作为参考和可移植算法库，不整套直接搬进来。
- 自己的全局规划、局部控制、平滑器、代价地图层都放到 `forklift_nav2_plugins`，通过 pluginlib 接入 Nav2。
- 真车底盘协议单独放到 `forklift_vehicle_interface`，不要混在 planner/controller 里。
- 调度任务和安全逻辑单独分层，后续接上位调度系统、站点任务、装卸流程时不会污染 Nav2 参数。

## 1. 目标 package 结构

后续建议从当前 `forklift_nav2_demo` 过渡成下面这些 package。当前 demo 可以先保留作为仿真验证入口，等新结构稳定后再逐步迁移。

```text
forklift_bringup
  启动 Gazebo / RViz / Nav2 / 自己的节点

forklift_description
  URDF / xacro / 真实车体参数

forklift_navigation
  Nav2 参数、地图、行为树配置

forklift_nav2_plugins
  自己的 GlobalPlanner / Controller / Smoother / CostmapLayer

forklift_vehicle_interface
  底盘协议适配，接真实车控制器

forklift_task_manager
  接调度系统，下发任务点、装卸点、路径任务

forklift_safety
  急停、限速区、防撞、掉边保护、状态监控

forklift_msgs
  自定义任务、车辆状态、底盘命令消息
```

### forklift_bringup

职责：

- 统一启动仿真版系统。
- 统一启动真车版系统。
- 管理 Gazebo、RViz、Nav2、robot_state_publisher、自定义节点。
- 负责 launch argument，比如地图路径、是否使用仿真时间、是否启动 RViz、是否启用安全节点。

建议文件：

```text
forklift_bringup/
  launch/
    sim_bringup.launch.py
    real_bringup.launch.py
    nav2_only.launch.py
    rviz.launch.py
```

设计重点：

- 仿真启动和真车启动要分开。
- 真车启动不要依赖 Gazebo。
- 所有节点参数文件从 `forklift_navigation/config` 或各自 package 的 `config` 引入。
- 后续接调度系统时，由 bringup 决定是否启动 `forklift_task_manager`。

### forklift_description

职责：

- 存放 forklift 的 URDF/xacro。
- 存放 mesh、惯性参数、传感器安装位置。
- 管理真实车体尺寸、叉臂长度、base_link、laser、odom 等 frame。
- 给 Nav2 footprint 参数提供真实几何依据。

建议文件：

```text
forklift_description/
  urdf/
    forklift.urdf.xacro
    sensors.xacro
    wheels.xacro
  meshes/
  config/
    vehicle_dimensions.yaml
```

设计重点：

- 真实车长、车宽、叉臂伸出长度、激光位置必须准确。
- Nav2 的 `footprint` 不会自动读取 Gazebo collision，所以要从这里同步到 `forklift_navigation` 参数。
- 如果真车是三支点或后轮转向，URDF 和控制模型要单独描述，不要继续假装成普通四轮车。

### forklift_navigation

职责：

- 存放 Nav2 参数。
- 存放地图。
- 存放行为树 XML。
- 存放 keepout mask、speed zone、dock point 等导航配置。

建议文件：

```text
forklift_navigation/
  config/
    nav2_sim.yaml
    nav2_real.yaml
    controller_profiles.yaml
    planner_profiles.yaml
  maps/
    forklift_factory_map.yaml
    forklift_factory_map.pgm
  behavior_trees/
    navigate_to_pose.xml
  rviz/
    forklift_nav2.rviz
```

设计重点：

- 仿真参数和真车参数分开。
- 地图继续使用 Nav2 标准 `yaml + pgm/png`，运行时都是 `/map` 上的 `nav_msgs/msg/OccupancyGrid`。
- 后续调度系统一般不直接依赖图片地图，而是依赖地图坐标系里的任务点、站点、路线约束。
- 升级 planner/controller 通常不需要换地图格式，只要它们能读取 costmap 或 occupancy grid。

### forklift_nav2_plugins

职责：

- 存放自己实现的 Nav2 插件。
- 通过 pluginlib 注册，然后在 Nav2 yaml 里替换插件名。
- 承接 ORU 里可用的规划、约束、平滑、控制思想。

建议结构：

```text
forklift_nav2_plugins/
  include/forklift_nav2_plugins/
    oru_global_planner.hpp
    forklift_controller.hpp
    forklift_smoother.hpp
    forklift_safety_layer.hpp
  src/
    oru_global_planner.cpp
    forklift_controller.cpp
    forklift_smoother.cpp
    forklift_safety_layer.cpp
  plugins.xml
  CMakeLists.txt
  package.xml
  test/
```

这是后续真正改算法的主 package。

### forklift_vehicle_interface

职责：

- 接真实底盘控制器协议。
- 把 Nav2 输出的 `/cmd_vel` 或自定义控制命令转换成底盘协议。
- 读取底盘反馈，发布 `/odom`、车辆状态、故障状态。
- 做通信 watchdog、命令超时、底盘模式切换。

建议接口：

```text
订阅:
  /cmd_vel
  /forklift/control_cmd
  /forklift/emergency_stop

发布:
  /odom
  /forklift/vehicle_state
  /forklift/control_report
  /forklift/fault_state
```

设计重点：

- 底盘协议不要写进 Nav2 controller 插件。
- Nav2 controller 只负责算运动命令。
- 真车底层通信、串口/CAN/TCP、校验、超时都放在这里。

### forklift_task_manager

职责：

- 接调度系统。
- 管理任务点、装卸点、等待点、充电点。
- 调用 Nav2 action，比如 `NavigateToPose` 或 `NavigateThroughPoses`。
- 后续支持路径任务、站点任务、装卸动作编排。

建议接口：

```text
订阅或服务:
  /forklift/task_request
  /forklift/cancel_task

发布:
  /forklift/task_state
  /forklift/current_mission
```

设计重点：

- 调度系统不应该直接控制 `/cmd_vel`。
- 调度系统发任务，task manager 决定调用 Nav2 和车辆动作。
- 地图上的任务点应该用 `map` 坐标系保存。

### forklift_safety

职责：

- 急停。
- 限速区。
- 掉边保护。
- 防撞监控。
- 状态 watchdog。
- 关键区域地理围栏。

建议能力：

```text
急停:
  软件急停、硬件急停状态接入

限速:
  根据区域、任务状态、载货状态限制速度

掉边保护:
  yard 边缘、平台边缘、禁行区检测

防撞:
  激光点云/scan 检查、Nav2 collision monitor、额外安全距离
```

设计重点：

- 安全层不只依赖 planner。
- 即使 planner 规划错误，安全层也要能限速或停车。
- 真车阶段建议启用 `nav2_collision_monitor`，再叠加自己的状态监控。

### forklift_msgs

职责：

- 定义项目自己的消息、服务、action。
- 给底盘接口、任务系统、安全系统统一数据结构。

建议消息：

```text
msg/
  VehicleState.msg
  ControlReport.msg
  SafetyState.msg
  TaskState.msg
  Station.msg

srv/
  SetOperationMode.srv
  ClearFault.srv

action/
  ExecuteTask.action
  DockPallet.action
```

设计重点：

- 先不要过度设计。
- 等底盘协议发来后，再把真实字段映射成 `VehicleState` 和 `ControlReport`。
- Nav2 原生 action 继续用，自己的 action 主要负责业务任务。

## 2. 当前地图格式与后续限制

当前地图格式是 Nav2 标准静态地图：

```text
forklift_factory_map.yaml
forklift_factory_map.pgm
```

YAML 描述分辨率、原点、阈值和图片文件。PGM 保存占据栅格图。Nav2 `map_server` 启动后会把它发布成：

```text
nav_msgs/msg/OccupancyGrid
```

这对后续升级算法基本没有问题，因为大多数 ROS2 导航算法最终都能使用以下两类输入：

- `nav_msgs/msg/OccupancyGrid`
- `nav2_costmap_2d::Costmap2D`

地图格式真正需要额外扩展的情况：

- 要做多楼层地图。
- 要保存语义信息，比如货架、月台、禁行区、限速区、装卸区。
- 要做路线预约或调度系统的拓扑路网。
- 要给不同车型保存不同可通行区域。

建议做法：

- 基础障碍地图继续用 `yaml + pgm/png`。
- 任务点、站点、禁行区、限速区另存 YAML/JSON 或 Nav2 keepout/speed mask。
- 调度路网单独保存，不要直接画进 occupancy map。

这样后续换 Nav2 planner、接 ORU planner、写自己的 planner，都不需要推翻地图格式。

## 3. ORU 与当前 Nav2 的关系

`navigation_oru-release` 更像一套研究/工程算法集合，不是一个可以直接替换 Nav2 的完整 ROS2 bringup。

当前 ORU 中与我们最相关的模块：

```text
orunav_motion_planner
  lattice motion planner，A* / ARA*，带车辆模型和 motion primitives

orunav_constraint_extract
  从 occupancy grid 和车辆几何中提取碰撞约束区域

orunav_path_smoother
  路径平滑，偏优化方法，依赖 ACADO

orunav_mpc
  MPC 轨迹跟踪，偏 car-like 车辆模型

orunav_geometry
  几何计算工具

orunav_trajectory_processor
  轨迹后处理、速度相关逻辑
```

对当前 forklift 的判断：

- ORU 的 motion planner 有价值，适合改造成自己的 Nav2 GlobalPlanner。
- ORU 的 constraint extract 有价值，适合借鉴到 footprint collision、path validator、costmap layer。
- ORU 的 path smoother 有价值，但 ACADO 依赖会增加移植成本，先不要作为第一步。
- ORU 的 MPC 是 car-like 思路，等真车三支点运动学和底盘协议明确后再决定是否迁移。
- ORU 里面已有的一些 primitives 是针对特定车辆，不应该直接套在当前 forklift 上。

## 4. ORU 接入策略

推荐路线：

```text
第一阶段:
  Nav2 原生框架 + 调参数 + 修 footprint/costmap

第二阶段:
  新建 forklift_nav2_plugins
  先写最小可运行插件，确认 pluginlib、Nav2 yaml、launch 都能跑

第三阶段:
  把 ORU motion planner 思路迁移成 GlobalPlanner 插件

第四阶段:
  加路径平滑和约束检查

第五阶段:
  等底盘协议和真实运动学明确后，再写 Controller 插件

第六阶段:
  接 task manager、vehicle interface、safety，上真车闭环
```

不建议路线：

- 不建议直接把 ORU 的 service 节点接到 Nav2 外面，让 Nav2 和 ORU 各跑各的。
- 不建议直接修改 Nav2 内部 planner server 或 controller server。
- 不建议第一步就迁移 ORU MPC。
- 不建议把底盘协议写进 planner/controller 插件。

## 5. forklift_nav2_plugins 开发步骤

### Step 0: 固化当前基线

目标：

- 当前 `forklift_nav2_demo` 能稳定启动 Gazebo、RViz、Nav2。
- 地图能加载。
- forklift 能在 RViz 中接收目标点并移动。
- RViz 能显示 global costmap、local costmap、footprint、global path、local trajectory。

必须记录：

```text
当前地图:
  forklift_factory_map.yaml

当前全局规划:
  nav2_navfn_planner/NavfnPlanner

当前局部控制:
  dwb_core::DWBLocalPlanner

当前模型:
  Gazebo diff drive 简化模型
```

验收：

- 可以复现穿障碍、贴边、掉出场地等问题。
- 有固定测试场景，后续每改一次算法都能对比。

### Step 1: 先用 Nav2 参数修明显问题

在写自定义插件前，先处理最容易导致穿障碍的问题：

```text
footprint:
  覆盖车体 + 叉臂真实外形

global_costmap / local_costmap:
  inflation_radius 一致或按风险调大
  cost_scaling_factor 合理
  obstacle_layer 确认接收到 laser scan

DWB:
  提高 BaseObstacle.scale
  检查 PathAlign / GoalAlign / PathDist / GoalDist 权重

Navfn:
  allow_unknown 根据场景改成 false 测试
```

这一步不算算法升级，但能先确认问题到底是参数、footprint、costmap，还是 planner/controller 本身。

### Step 2: 创建 forklift_nav2_plugins package

目标：

- 新建 package。
- 添加 `plugins.xml`。
- 添加一个最小 `GlobalPlanner` 插件。
- 先返回一条简单路径或包一层现成逻辑，用来确认 Nav2 能加载自己的插件。

GlobalPlanner 插件接口：

```cpp
class MyPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(...) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
};
```

Nav2 yaml 中替换方式：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["ForkliftPlanner"]
    ForkliftPlanner:
      plugin: "forklift_nav2_plugins/OruGlobalPlanner"
```

验收：

- `planner_server` 日志显示成功加载 `forklift_nav2_plugins/OruGlobalPlanner`。
- RViz 中能看到插件输出路径。
- 出错时能安全 fallback 或清晰报错。

### Step 3: ORU-inspired GlobalPlanner

目标：

- 把 `orunav_motion_planner` 的 lattice planner 思路改造成 Nav2 GlobalPlanner。
- 输入来自 Nav2 costmap，而不是 ORU 原 service。
- 输出 `nav_msgs/msg/Path`。

迁移重点：

```text
输入转换:
  Costmap2D -> ORU-like grid

车辆模型:
  不直接用 ORU 默认 CarModel
  根据 forklift 三支点或当前 diff-drive 阶段重新定义

motion primitives:
  分辨率要匹配地图
  转弯半径要匹配真实车
  footprint 要覆盖叉臂

搜索算法:
  先 A*
  再考虑 ARA* 限时搜索

输出:
  pose 序列 -> nav_msgs/msg/Path
```

注意：

- 当前 ORU `get_path_service` 更偏独立 service，不是 Nav2 插件。
- 它里面对车辆模型和 primitives 有历史假设，不能直接当 forklift 可用算法。
- 第一版可以先只做 2D footprint-aware planner，不急着做复杂三支点完整模型。

验收：

- 全局路径不穿越已知障碍。
- 在窄门口不会规划明显不可能通过的路径。
- 在 yard 边缘不会贴着平台外侧走。
- 输出路径能被当前 DWB 或后续 controller 跟踪。

### Step 4: 自定义 Smoother

目标：

- 对 GlobalPlanner 输出路径做平滑。
- 保持路径不压障碍。
- 控制曲率、倒车、转向变化。

Nav2 smoother 插件方向：

```text
forklift_nav2_plugins/ForkliftSmoother
  实现 nav2_core::Smoother
```

ORU 可借鉴内容：

- `orunav_path_smoother` 的约束优化思路。
- `orunav_constraint_extract` 的局部多边形约束。
- `orunav_geometry` 的 footprint 几何计算。

第一版建议：

- 不直接迁移 ACADO。
- 先做轻量 smoothing + collision check。
- 如果平滑后碰撞，就回退到原路径或降低平滑强度。

验收：

- 平滑后路径更易跟踪。
- 平滑后不穿障碍。
- 靠近货架、平板车、窄门时不会把路径拉到障碍物上。

### Step 5: 自定义 Controller

目标：

- 等真实底盘协议和三支点运动学明确后，再决定 controller。
- 当前仿真 diff-drive 阶段，可以先用 DWB、Regulated Pure Pursuit 或 MPPI 试验。

Controller 插件接口：

```cpp
class ForkliftController : public nav2_core::Controller
{
public:
  void configure(...) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
};
```

分阶段策略：

```text
仿真阶段:
  继续输出 geometry_msgs/Twist
  适配 Gazebo diff drive

真车第一阶段:
  Nav2 controller 仍输出 Twist
  forklift_vehicle_interface 转成底盘协议

真车高级阶段:
  如果底盘需要转角、舵角、轮速等命令
  再考虑自定义控制命令和 vehicle_interface 内部控制器
```

ORU MPC 判断：

- 适合研究轨迹跟踪和约束控制。
- 不是第一步要迁移的模块。
- 如果真车底盘可接受速度 + 转角类命令，后续可参考 ORU MPC。
- 如果真车底盘只接受线速度/角速度，先不要硬接 MPC。

验收：

- 控制输出不超过速度/角速度/加速度限制。
- 路径跟踪误差可接受。
- 靠近障碍时不会为了追路径压到障碍。
- 命令超时时 vehicle interface 能停车。

### Step 6: 自定义 CostmapLayer

目标：

- 把 forklift 特有风险写进 costmap，而不是只靠 planner 猜。
- 支持 yard 边缘、平板车附近、禁行区、限速区、调度预约区域。

CostmapLayer 方向：

```text
forklift_nav2_plugins/ForkliftSafetyLayer
  继承 nav2_costmap_2d::Layer 或 CostmapLayer
```

可做能力：

```text
keepout:
  场地边缘、掉落风险区域、禁行区域

speed zone:
  平板车附近、装卸区、窄通道限速

reservation:
  后续调度系统给某段路加临时占用

fork risk:
  叉臂扫到障碍物的风险区域加高代价
```

ORU 可借鉴内容：

- `orunav_constraint_extract` 的占据栅格 + 车辆几何约束。
- `orunav_geometry` 的多边形和 footprint 检查。

验收：

- RViz 中能看到新增 costmap layer 生效。
- Planner 会绕开 keepout 区域。
- Controller 在限速区域能减速。
- yard 边缘不会被规划为可通行区域。

## 6. 推荐实现顺序

建议按下面顺序做，不要一次全改：

```text
1. 新建文档和目标架构
2. 当前 Nav2 参数调优，确认 footprint/costmap 正确
3. 新建 forklift_nav2_plugins package
4. 写最小 GlobalPlanner 插件并加载成功
5. 接入 ORU-inspired lattice planner
6. 加 path collision validator
7. 加 smoother
8. 再考虑 controller
9. 新建 vehicle_interface，接底盘协议
10. 新建 task_manager 和 safety
```

原因：

- 全局规划插件最容易先验证，输入输出清楚。
- Controller 和底盘协议强相关，协议没来之前过早写会返工。
- Safety 必须独立于 planner/controller，后期上车前再加强。

## 7. 测试计划

### RViz 必须显示

```text
/map
/global_costmap/costmap
/local_costmap/costmap
robot footprint
global path
local trajectory
laser scan
TF
```

### 仿真测试场景

```text
窄门口:
  检查是否规划穿墙

平板车附近:
  检查是否贴边掉落

叉臂靠近障碍:
  检查 footprint 是否覆盖叉臂

目标点靠近障碍:
  检查是否拒绝或绕行

未知区域:
  检查 allow_unknown 对路径的影响

恢复行为:
  检查卡住后是否旋转、后退或清图导致风险
```

### 插件单元测试

```text
Costmap 转换:
  Costmap2D -> planner grid 不丢分辨率和原点

Footprint collision:
  给定 pose + footprint，能判断是否压障碍

Planner:
  简单地图能输出路径
  障碍地图不穿障碍
  无路时返回失败

Smoother:
  平滑后路径仍在 free space

Controller:
  输出速度不超过限制
  goal 附近能稳定停下
```

### 真车前验收

```text
定位:
  map -> odom -> base_link TF 稳定

底盘:
  odom 方向、速度符号、角速度符号正确

安全:
  急停可用
  命令超时停车
  障碍物近距离停车
  掉边区域不进入

任务:
  调度任务可取消
  任务失败能上报原因
```

## 8. 后续第一步建议

下一步先做两件事：

```text
第一:
  在当前 forklift_nav2_demo/config/forklift_nav2.yaml 中调 footprint、inflation、DWB obstacle critic
  先解决明显穿障碍问题

第二:
  新建 forklift_nav2_plugins package
  先实现一个最小 GlobalPlanner 插件
  确认 Nav2 能加载自己的插件
```

等最小插件能被 Nav2 加载后，再开始把 ORU 的 `orunav_motion_planner` 拆出来迁移。这样每一步都有可运行系统，不会陷入“搬了一大套 ORU 但不知道哪里坏了”的状态。

