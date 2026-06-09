# Forklift Nav2 Algorithm Guide

这份文档说明当前 `forklift_nav2_demo` 项目里 Nav2 的结构、正在使用的算法、关键参数位置，以及后续如果要优化避障或写自己的 Nav2 算法应该从哪里下手。

当前先按差速模型处理 forklift，不改 Gazebo 车辆动力学。优先建议先通过 footprint、costmap 和现成 planner/controller 参数解决穿障碍问题，再考虑写自定义插件。

## 1. 当前项目用到的 Nav2 结构

### 启动链路

主要启动文件：

- `forklift_nav2_demo/launch/forklift_navigation.launch.py`
  - 启动 Gazebo world。
  - 启动 Nav2 bringup。
  - 启动 RViz。
  - 将 `map:=...` 传给 Nav2 的 `map_server`。
  - 将 `nav2_params_file` 传给 Nav2 的参数文件。

- `forklift_nav2_demo/launch/forklift_gazebo.launch.py`
  - 启动 Gazebo。
  - 用 `robot_state_publisher` 发布 `robot_description`。
  - 用 `gazebo_ros/spawn_entity.py` 从 `robot_description` 生成名为 `forklift` 的 Gazebo 实体。

- `forklift_nav2_demo/urdf/forklift_diff_drive.urdf.xacro`
  - 定义 forklift 的车体、叉臂、轮子、激光雷达和 Gazebo 插件。
  - 当前运动插件是 `libgazebo_ros_diff_drive.so`。

主要参数文件：

- `forklift_nav2_demo/config/forklift_nav2.yaml`
  - 当前所有 Nav2 算法、costmap、footprint、controller、planner 参数都在这里。

## 2. 当前实际使用的算法

### 地图与定位

当前使用：

- `map_server`
  - 读取 `.yaml/.pgm` 地图。
  - 发布 `/map`。

- `amcl`
  - 使用粒子滤波定位。
  - 当前机器人模型为：

```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

这和当前 Gazebo diff drive 简化模型一致。

### 全局规划

当前使用：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

含义：

- `NavfnPlanner` 是基于 costmap 的经典全局规划器。
- `use_astar: false` 时更偏 Dijkstra 风格。
- `allow_unknown: true` 表示允许路径经过未知区域。

重要限制：

- `NavfnPlanner` 本身不是车辆运动学约束规划器。
- 它不会真正理解 forklift 的长车体、叉臂和转弯半径。
- 它主要依赖 global costmap 里已经膨胀好的障碍物来避障。
- 如果 inflation 或 footprint 不合适，就可能规划出看起来穿墙、贴边、穿障碍的路径。

### 局部控制

当前使用：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
```

DWB 会在局部窗口里采样速度，模拟一小段轨迹，然后用 critics 给轨迹打分。

当前 critics：

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
BaseObstacle.scale: 0.02
```

关键点：

- `BaseObstacle` 用来惩罚靠近障碍物的轨迹。
- 当前 `BaseObstacle.scale: 0.02` 很低，避障惩罚很弱。
- DWB 更容易为了贴近路径而选择擦着障碍甚至看起来穿障碍的轨迹。

## 3. 车体大小与避障在哪里生效

Nav2 不会自动从 Gazebo collision 里推导导航 footprint。导航时车体大小主要看 costmap 里的 `footprint`。

当前 local costmap：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.78, 0.42], [0.78, -0.42], [-1.18, -0.42], [-1.18, 0.42]]"
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 1.0
        cost_scaling_factor: 3.0
```

当前 global costmap：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[0.78, 0.42], [0.78, -0.42], [-1.18, -0.42], [-1.18, 0.42]]"
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 0.55
        cost_scaling_factor: 3.0
```

当前 footprint 是一个矩形：

- 前方 `x = 0.78`
- 后方 `x = -1.18`
- 左右宽度 `y = +/-0.42`

如果 forklift 的叉臂、车尾、车宽、传感器位置和这个 footprint 不一致，Nav2 就会低估车体大小。结果就是 RViz 里看起来机器人中心路径没撞，但实际模型、叉臂或边角会扫到障碍物。

## 4. 现在穿障碍优先排查什么

建议按这个顺序排查，不要一上来改源码。

### 4.1 检查 RViz 显示

在 RViz 打开这些显示：

- `/map`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/global_costmap/published_footprint`
- `/local_costmap/published_footprint`
- global path
- local trajectory
- laser scan `/scan`

判断标准：

- footprint 是否包住 forklift 和叉臂。
- local costmap 是否能看到动态障碍。
- global costmap 是否能看到地图障碍和膨胀区。
- global path 是否穿过障碍。
- local trajectory 是否让 footprint 压到障碍。

### 4.2 调整 footprint

如果车体或叉臂实际比 footprint 长，就先改：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "..."

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "..."
```

原则：

- footprint 要覆盖真实会碰撞的外形。
- 如果叉臂需要算进避障，就把叉臂长度也算进去。
- local 和 global costmap 的 footprint 要保持一致，除非有明确理由。

### 4.3 提高 DWB 障碍权重

当前：

```yaml
BaseObstacle.scale: 0.02
```

这个值太弱。可以逐步提高，例如先试：

```yaml
BaseObstacle.scale: 0.2
```

如果仍然贴边，再试更高。调参时观察 local trajectory 是否远离障碍。

### 4.4 加强 inflation

当前 local inflation 是 `1.0`，global inflation 是 `0.55`。如果全局路径贴障碍，可以提高 global inflation：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 1.0
```

如果局部轨迹贴障碍，可以提高 local inflation 或调整 `cost_scaling_factor`。

注意：

- inflation 太大时，窄门可能会被完全堵死。
- inflation 太小时，路径会贴障碍。
- 调参时要看 costmap，不要只看车的最终运动。

### 4.5 检查未知空间

当前：

```yaml
track_unknown_space: true
allow_unknown: true
```

这意味着地图未知区域会被跟踪，但 planner 仍允许穿过未知区域。如果车经常往地图外走，可以考虑把全局规划改成不允许未知区域：

```yaml
allow_unknown: false
```

这通常能减少向地图外规划的行为。

## 5. 如果要换现成算法

### 5.1 全局规划器可选项

当前：

```yaml
plugin: "nav2_navfn_planner/NavfnPlanner"
```

可以考虑切换到：

- `nav2_smac_planner/SmacPlanner2D`
  - 仍是 2D 栅格规划。
  - 通常比 Navfn 配置空间更现代。

- `nav2_smac_planner/SmacPlannerHybrid`
  - 更适合考虑车体朝向和非完整约束。
  - 如果后续想让 forklift 更像车一样规划，可以重点看它。

- `nav2_theta_star_planner/ThetaStarPlanner`
  - 路径更平滑，但仍要依赖 costmap 避障。

换全局规划器主要改：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "..."
```

### 5.2 局部控制器可选项

当前：

```yaml
plugin: "dwb_core::DWBLocalPlanner"
```

可以考虑切换到：

- `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
  - 跟踪路径简单稳定。
  - 常用于差速/阿克曼类机器人。

- `nav2_mppi_controller::MPPIController`
  - 更强的采样优化控制器。
  - 参数更多，但更适合复杂避障和轨迹优化。

换局部控制器主要改：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "..."
```

## 6. 如果要写自己的 Nav2 算法

Nav2 的算法通常通过 pluginlib 插件接入。不要直接改 `/opt/ros/humble` 里的系统包，建议在工作区新建自己的 ROS 2 package。

### 6.1 写自己的全局规划器

实现接口：

```cpp
nav2_core::GlobalPlanner
```

需要实现的核心能力：

- 接收起点和终点。
- 读取 global costmap。
- 生成 `nav_msgs/msg/Path`。
- 在路径不可达时返回失败或空路径。

适合改的内容：

- 自己的 A* / Hybrid A* / lattice planner。
- 更强的 footprint-aware 碰撞检查。
- 针对 forklift 的倒车、转弯半径、叉臂安全距离。

接入方式：

- 新建插件 package。
- 用 pluginlib 注册 planner。
- 在 `forklift_nav2.yaml` 的 `planner_server` 中把 plugin 改成你的插件名。

### 6.2 写自己的局部控制器

实现接口：

```cpp
nav2_core::Controller
```

需要实现的核心能力：

- 接收 global path。
- 读取 local costmap。
- 输出 `geometry_msgs/msg/Twist`。
- 做局部避障、速度限制、角速度限制。

适合改的内容：

- 轨迹采样策略。
- footprint 轨迹碰撞检查。
- forklift 转弯和叉臂安全区。
- 贴边惩罚。
- 防止驶出地图边界。

接入方式：

- 新建插件 package。
- 用 pluginlib 注册 controller。
- 在 `forklift_nav2.yaml` 的 `controller_server` 中把 `FollowPath.plugin` 改成你的插件名。

### 6.3 写自己的 costmap layer

如果问题不是 planner/controller，而是“地图里没有正确表达障碍或危险区域”，可以写 costmap layer。

实现方向：

- 在 `nav2_costmap_2d` 里新增 layer 插件。
- 把 yard 边缘、禁行区、叉车工作区等写成高代价区域。
- 让 planner 和 controller 都能看到这些区域。

适合场景：

- 不想让车靠近平台边缘。
- 地图上某些区域虽然没有墙，但也不允许走。
- 需要把语义区域变成 navigation cost。

## 7. Nav2 Package Map

当前 Humble 环境中和这个项目相关的 Nav2 包：

- 核心接口：`nav2_core`
- 启动：`nav2_bringup`
- 行为树：`nav2_bt_navigator`、`nav2_behavior_tree`
- 全局规划：`nav2_planner`、`nav2_navfn_planner`、`nav2_smac_planner`、`nav2_theta_star_planner`
- 局部控制：`nav2_controller`、`nav2_dwb_controller`、`nav2_regulated_pure_pursuit_controller`、`nav2_mppi_controller`
- 地图和代价地图：`nav2_map_server`、`nav2_costmap_2d`、`nav2_voxel_grid`
- 定位：`nav2_amcl`
- 恢复和行为：`nav2_behaviors`
- 平滑和速度：`nav2_smoother`、`nav2_constrained_smoother`、`nav2_velocity_smoother`
- 安全：`nav2_collision_monitor`
- 可视化和工具：`nav2_rviz_plugins`、`nav2_simple_commander`
- 生命周期和通用工具：`nav2_lifecycle_manager`、`nav2_common`、`nav2_util`、`nav2_msgs`

## 8. 推荐优化路线

### 第一阶段：先调参数，不改源码

目标：解决穿障碍、贴边、掉出 yard。

建议修改：

- 调整 local/global `footprint`。
- 提高 `BaseObstacle.scale`。
- 统一并加强 global/local `inflation_radius`。
- 必要时把 `allow_unknown` 改成 `false`。
- 在 RViz 中确认 costmap 和 footprint 显示正确。

这是最快、风险最低的路线。

### 第二阶段：换现成算法

目标：让路径更适合 forklift。

建议尝试：

- 全局规划：`SmacPlanner2D` 或 `SmacPlannerHybrid`。
- 局部控制：`RegulatedPurePursuit` 或 `MPPI`。

这一步仍然不需要写 C++ 算法，但需要认真调参数。

### 第三阶段：写自定义插件

目标：实现自己的 forklift-aware 规划或控制。

建议优先写：

- 自定义 controller：如果主要问题是跟踪路径时穿障碍或贴边。
- 自定义 global planner：如果主要问题是生成的全局路径本身不合理。
- 自定义 costmap layer：如果主要问题是地图没有表达 yard 边界、禁行区或语义风险。

## 9. 测试场景和验收标准

### RViz 必看显示

- `/map`
- `/scan`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/global_costmap/published_footprint`
- `/local_costmap/published_footprint`
- global path
- local trajectory

### 测试场景

- 窄门口是否会规划穿墙。
- 平板车附近是否会贴边掉落。
- 叉臂是否会扫到障碍物。
- 目标点放在障碍物附近时，是否能拒绝或绕行。
- 地图边缘或未知区域附近，是否还会往外规划。

### 验收标准

- 全局路径不穿越已知障碍。
- 局部轨迹不让 footprint 压到障碍。
- forklift 不再从 yard 边缘掉下去。
- RViz 中 costmap 的膨胀区和 footprint 能解释车的实际行为。
- 后续要替换算法时，能直接从 `planner_server` 或 `controller_server` 的 plugin 配置入手。

