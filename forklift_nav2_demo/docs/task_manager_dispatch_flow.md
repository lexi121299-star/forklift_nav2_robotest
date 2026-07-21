# Task Manager、调度系统与规划控制流程图

本文档描述当前 forklift 项目里 Task Manager、调度系统、Nav2 规划控制、安全层、车辆接口和各 package 之间的关系。

当前结论：

- 调度系统负责“分配任务、授权路线、协调多车资源”。
- `forklift_task_manager` 负责“单车任务编排、状态维护、分段执行、失败上报”。
- `forklift_task_manager.dispatch_reporter_node` 负责“聚合任务、故障、车辆和 safety 状态，形成调度上报报文”。
- Nav2 负责“路径规划、行为树调度、局部控制、costmap、定位地图服务”。
- `forklift_nav2_plugins` / `forklift_oru_planner` 负责“叉车运动学约束下的规划和控制算法”。
- `forklift_safety` 负责“所有运动命令的最终安全拦截”。
- `forklift_vehicle_interface` 负责“把安全后的控制命令转成仿真或真车底盘命令”。
- `forklift_nav2_demo` 目前承担 bringup/demo 职责，统一启动 Gazebo、RViz、Nav2、safety、vehicle bridge 等节点。

## 1. 整体运行流程

```mermaid
flowchart TD
    Dispatcher["上位调度系统 / Fleet Coordinator<br/>任务分配、路线授权、资源预约、多车协调"]
    Rviz["RViz / CLI<br/>人工点选 goal 或调试任务"]

    TM["forklift_task_manager<br/>单车任务编排<br/>站点/路线管理<br/>状态机"]
    Reporter["dispatch_reporter_node<br/>聚合 task/fault/vehicle/safety<br/>发布调度上报"]

    Nav2BT["Nav2 BT Navigator<br/>navigate_to_pose action"]
    Planner["Nav2 Planner Server<br/>全局路径规划"]
    Controller["Nav2 Controller Server<br/>轨迹跟踪 / 局部控制"]
    Costmap["Nav2 Costmaps<br/>global_costmap / local_costmap"]
    MapLoc["Map Server / AMCL / TF<br/>地图、定位、坐标变换"]

    Plugins["forklift_nav2_plugins<br/>自定义 planner/controller/smoother/costmap 插件"]
    ORU["forklift_oru_planner<br/>ORU-style state lattice 核心算法"]

    Safety["forklift_safety<br/>safety_command_gate<br/>急停、命令超时、costmap timeout、故障拦截"]
    Vehicle["forklift_vehicle_interface<br/>sim_command_bridge / curtis_vehicle_interface<br/>仿真或真车底盘适配"]

    Sim["Gazebo / forklift_sim / forklift_warehouse_sim<br/>仿真车辆、传感器、环境"]
    RealTruck["真实叉车底盘<br/>CAN / 串口 / 控制器"]

    Msgs["forklift_msgs<br/>任务、控制、车辆状态、故障状态、自定义 service/action"]

    Dispatcher -->|"任务请求 / 站点 / 路线 / 优先级<br/>未来接口"| TM
    Rviz -->|"/goal_pose"| TM

    TM -->|"ExecuteRoute / go_to_station / pause / resume / cancel"| Msgs
    TM -->|"/navigate_to_pose action"| Nav2BT
    TM -->|"/forklift/task_status"| Reporter
    Reporter -->|"/forklift/dispatch_report<br/>或 HTTP POST"| Dispatcher
    TM -. "可选调试: /forklift/task_status" .-> Dispatcher
    TM -->|"/forklift/task_status"| Rviz

    Nav2BT --> Planner
    Nav2BT --> Controller
    Planner --> Costmap
    Controller --> Costmap
    Costmap --> MapLoc

    Planner -. "pluginlib" .-> Plugins
    Controller -. "pluginlib" .-> Plugins
    Plugins --> ORU
    Plugins -->|"/forklift/control_cmd_raw"| Safety
    Controller -->|"/cmd_vel recovery fallback"| Safety

    Safety -->|"/forklift/control_cmd"| Vehicle
    Safety -->|"/forklift/safety_gate/status"| TM
    Safety -->|"/forklift/safety_gate/status"| Reporter
    Safety -->|读取 costmap / odom / fault / vehicle_state| Costmap

    Vehicle -->|"/odom / vehicle_state / fault_state"| Nav2BT
    Vehicle -->|"/odom / vehicle_state / fault_state"| Safety
    Vehicle -->|"/forklift/vehicle_state<br/>/forklift/fault_state"| Reporter

    Vehicle -->|仿真 /cmd_vel| Sim
    Vehicle -->|真车控制命令| RealTruck
    Sim -->|"/scan / odom / tf"| MapLoc
    RealTruck -->|"odom / fault / io / state"| Vehicle

    Msgs -. "共享消息定义" .-> TM
    Msgs -. "共享消息定义" .-> Safety
    Msgs -. "共享消息定义" .-> Vehicle
    Msgs -. "共享消息定义" .-> Plugins
```

### 关键原则

1. 调度系统不直接发 `/cmd_vel`，也不直接控制底盘。
2. Task Manager 不直接发 `/cmd_vel` 或 CAN 指令，只调用 Nav2 action 和车辆业务动作。
3. Planner / Controller 只负责算路径和控制命令，不负责业务任务状态。
4. 所有运动命令最终都要经过 `forklift_safety`，不能绕过 safety gate。
5. `forklift_vehicle_interface` 只做底盘适配和状态反馈，不写任务逻辑。

## 2. 当前 package 关系图

```mermaid
flowchart LR
    subgraph Entry["启动 / 仿真入口"]
        Demo["forklift_nav2_demo<br/>当前 bringup/demo 入口<br/>launch、map、rviz、Gazebo、Nav2 参数"]
        SimPkg["forklift_sim<br/>轻量三点叉车运动学仿真"]
        WarehouseSim["forklift_warehouse_sim<br/>仓库 pickup demo"]
    end

    subgraph TaskLayer["任务层"]
        TaskMgr["forklift_task_manager<br/>路线/站点配置<br/>任务状态机<br/>Nav2 action client"]
        ReporterPkg["dispatch_reporter_node<br/>统一调度上报"]
    end

    subgraph NavigationLayer["规划控制层"]
        Nav2["Nav2<br/>BT navigator / planner_server / controller_server / costmaps"]
        Plugins["forklift_nav2_plugins<br/>自定义 Nav2 插件"]
        ORUCore["forklift_oru_planner<br/>state lattice planner core"]
        ACADO["ACADOtoolkit<br/>MPC/优化相关第三方代码"]
    end

    subgraph SafetyVehicle["安全与车辆接口层"]
        SafetyPkg["forklift_safety<br/>command gate<br/>recovery adapter<br/>急停/超时/故障拦截"]
        VehicleIf["forklift_vehicle_interface<br/>sim bridge<br/>Curtis 真车接口"]
    end

    subgraph Common["公共接口"]
        MsgPkg["forklift_msgs<br/>ForkliftControlCommand<br/>TaskStatus<br/>ExecuteRoute<br/>GoToStation<br/>SetEmergencyStop"]
    end

    ExternalDispatch["外部调度系统<br/>未来接入"]
    RvizCli["RViz / CLI<br/>人工调试入口"]
    Gazebo["Gazebo Classic"]
    Truck["真实叉车底盘"]

    ExternalDispatch -. "任务请求 / 路线授权<br/>未来接口" .-> TaskMgr
    ReporterPkg -. "/forklift/dispatch_report / HTTP POST" .-> ExternalDispatch
    RvizCli -->|"/goal_pose / service / action"| TaskMgr

    Demo --> Nav2
    Demo --> Gazebo
    Demo --> SafetyPkg
    Demo --> VehicleIf
    Demo -. "可选启动" .-> TaskMgr

    TaskMgr -->|"NavigateToPose action"| Nav2
    TaskMgr --> MsgPkg
    TaskMgr -->|"/forklift/task_status"| ReporterPkg

    Nav2 -->|"pluginlib 加载"| Plugins
    Plugins --> ORUCore
    Plugins -. "可选优化库" .-> ACADO
    Plugins --> MsgPkg

    Plugins -->|"/forklift/control_cmd_raw"| SafetyPkg
    Nav2 -->|"/cmd_vel recovery"| SafetyPkg
    SafetyPkg -->|"/forklift/control_cmd"| VehicleIf
    SafetyPkg -->|"/forklift/safety_gate/status"| ReporterPkg
    SafetyPkg --> MsgPkg

    VehicleIf --> Gazebo
    VehicleIf --> Truck
    VehicleIf -->|"/forklift/vehicle_state<br/>/forklift/fault_state"| ReporterPkg
    VehicleIf --> MsgPkg

    SimPkg -. "轻量仿真/算法验证" .-> Nav2
    WarehouseSim -. "仓库任务仿真" .-> TaskMgr
```

## 3. 单车任务执行流程

```mermaid
sequenceDiagram
    participant Dispatch as 上位调度系统 / RViz
    participant TM as forklift_task_manager
    participant Reporter as dispatch_reporter_node
    participant Nav2 as Nav2 NavigateToPose
    participant Planner as planner_server
    participant Controller as controller_server
    participant Safety as forklift_safety
    participant Vehicle as forklift_vehicle_interface
    participant Robot as Gazebo / 真实叉车

    Dispatch->>TM: 下发任务：站点 / 路线 / goal_pose
    TM->>TM: 校验任务、加载站点、拆分路线段
    TM->>Nav2: 发送 NavigateToPose goal
    Nav2->>Planner: 请求全局路径
    Planner->>Planner: 调用 forklift_nav2_plugins / ORU lattice
    Planner-->>Nav2: 返回可执行 path
    Nav2->>Controller: FollowPath
    Controller->>Controller: 调用自定义 controller
    Controller->>Safety: 发布 /forklift/control_cmd_raw
    Safety->>Safety: 检查急停、costmap、定位、车辆故障、命令超时

    alt 安全允许
        Safety->>Vehicle: 发布 /forklift/control_cmd
        Vehicle->>Robot: 转换为 Gazebo /cmd_vel 或真车底盘命令
        Robot-->>Vehicle: odom / state / fault
        Vehicle-->>Nav2: /odom / tf / vehicle_state
        Vehicle-->>Safety: /vehicle_state / fault_state
        Vehicle-->>Reporter: /vehicle_state / fault_state
    else 安全不允许
        Safety->>Vehicle: 输出 stop command
        Safety-->>TM: /forklift/safety_gate/status
        Safety-->>Reporter: /forklift/safety_gate/status
        TM->>TM: 任务转 PAUSED / BLOCKED / FAILED
    end

    Nav2-->>TM: 当前导航段结果
    alt 当前段成功，后面还有段
        TM->>Nav2: 发送下一段 NavigateToPose goal
    else 所有段完成
        TM-->>Reporter: /forklift/task_status = SUCCEEDED
        Reporter-->>Dispatch: /forklift/dispatch_report 或 HTTP POST
    else 段失败
        TM-->>Reporter: /forklift/task_status = FAILED + reason
        Reporter-->>Dispatch: /forklift/dispatch_report 或 HTTP POST
    end
```

## 4. Task Manager 状态流转

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> RUNNING: 接收 ExecuteRoute / go_to_station / goal_pose
    RUNNING --> SUCCEEDED: 所有路线段完成
    RUNNING --> RECOVERING: 单段失败但允许重试
    RECOVERING --> RUNNING: 重试当前段
    RECOVERING --> FAILED: 超过最大重试次数

    RUNNING --> PAUSED: pause service / safety gate blocking
    PAUSED --> RUNNING: resume service
    PAUSED --> CANCELED: cancel service / action cancel

    RUNNING --> CANCELED: cancel service / action cancel
    RUNNING --> FAILED: Nav2 失败 / 配置错误 / action 不可用

    SUCCEEDED --> IDLE: 等待新任务
    FAILED --> IDLE: 任务清理后等待新任务
    CANCELED --> IDLE: 任务清理后等待新任务
```

当前实现里，Task Manager 已经有最小状态机和以下入口：

- action：`execute_route`
- service：`go_to_station`
- service：`pause`
- service：`resume`
- service：`cancel`
- topic 输入：`/goal_pose`
- topic 输入：`/forklift/safety_gate/status`
- topic 输出：`/forklift/task_status`
- Nav2 action client：`navigate_to_pose`

当前 `task_manager.launch.py` 默认也启动 `dispatch_reporter_node`。它不发运动命令，只聚合以下输入：

- `/forklift/task_status`：任务状态，由 `forklift_task_manager` 发布。
- `/forklift/fault_state`：底盘故障状态，由 `forklift_vehicle_interface` 发布。
- `/forklift/vehicle_state`：车辆运行状态和 `battery_percent`，由 `forklift_vehicle_interface` 发布。
- `/forklift/safety_gate/status`：最终安全闸状态，由 `forklift_safety` 发布。

它默认输出 `/forklift/dispatch_report`（`std_msgs/String`，内容是 JSON），可选配置 `dispatch_http_url` 后同时 HTTP POST 到调度系统。

## 5. 职责划分表

| 模块 / package | 当前职责 | 不应该负责 |
| --- | --- | --- |
| 外部调度系统 | 多车任务分配、路线授权、资源预约、优先级、死锁/会车策略 | 直接发 `/cmd_vel`、直接控制 CAN、绕过车端 safety |
| `forklift_task_manager` | 单车任务编排、站点/路线管理、分段执行、暂停/恢复/取消、任务状态发布；`dispatch_reporter_node` 聚合调度上报 | 底盘控制、路径算法、局部避障算法、急停执行 |
| Nav2 | lifecycle、BT navigator、planner/controller/costmap/map server/AMCL | 调度业务、多车资源协调、底盘协议 |
| `forklift_nav2_plugins` | 自定义全局规划、局部控制、平滑、costmap 插件 | 任务队列、调度接口、真车通信 |
| `forklift_oru_planner` | ORU-style state lattice 规划核心算法 | ROS bringup、任务状态、底盘协议 |
| `forklift_safety` | safety gate、急停、命令超时、costmap 超时、故障拦截、recovery adapter | 路线调度、业务任务编排、地图站点管理 |
| `forklift_vehicle_interface` | 仿真 bridge、真车 Curtis 接口、车辆状态/故障/里程计反馈 | 任务决策、路径规划、安全策略决策 |
| `forklift_msgs` | 自定义 msg/srv/action 公共接口 | 节点运行逻辑 |
| `forklift_nav2_demo` | 当前仿真/真车 bringup 入口、地图、RViz、Gazebo、Nav2 参数 | 长期不应承载全部业务逻辑，后续可拆成 bringup/navigation/description |
| `forklift_sim` | 轻量三点叉车运动学仿真 | 真实 Gazebo 场景和完整系统 bringup |
| `forklift_warehouse_sim` | 仓库 pickup demo 和场景验证 | 调度系统主体 |
| `ACADOtoolkit` | 优化/MPC 相关第三方库 | ROS 任务状态和车辆接口 |

## 6. 后续接调度系统建议接口

第一版调度接入不要直接做复杂多车协议，建议先定义单车任务接口：

```text
调度系统 -> forklift_task_manager:
  task_id
  task_type: go_to_station / execute_route / park / charge / pickup / dropoff
  target_station
  route_name
  priority
  allowed_resources
  timestamp

dispatch_reporter_node -> 调度系统:
  task_id            # 后续接真实调度任务 ID 时补入
  state: IDLE / RUNNING / PAUSED / BLOCKED / FAILED / SUCCEEDED / CANCELED
  active_route
  current_segment
  progress
  failure_reason
  robot_pose         # 后续由定位/TF 聚合补入
  safety_status
  vehicle_state
  fault_state
  alarms
```

当前 `dispatch_reporter_node` 已实现 `state`、`active_route`、`current_segment`、
`failure_reason`、`safety_status`、`vehicle_state`、`fault_state` 和 `alarms`；
`task_id`、`progress`、`robot_pose` 属于接真实调度协议时的下一步字段。

后续扩展多车时，调度系统可以增加 resource lease、route reservation、intersection permission、reroute decision 等字段，但仍然保持一个原则：调度系统授权路线和资源，车端 Task Manager 执行任务，底层运动命令必须经过 Safety Gate。

## 7. 当前 Dispatch Reporter 实现

新增节点：

```bash
ros2 run forklift_task_manager dispatch_reporter_node
```

或随任务管理器启动：

```bash
ros2 launch forklift_task_manager task_manager.launch.py
```

常用 launch 参数：

```text
use_dispatch_reporter:=true
robot_id:=forklift_001
dispatch_report_topic:=/forklift/dispatch_report
dispatch_http_url:=
battery_low_threshold:=20.0
```

默认只发布 ROS 话题，不主动访问外部调度系统。配置 `dispatch_http_url` 后，每个报告周期会把同一份 JSON 以 `Content-Type: application/json` POST 到该 URL。

`/forklift/dispatch_report` 示例结构：

```json
{
  "robot_id": "forklift_001",
  "task": {
    "available": true,
    "state": "PAUSED",
    "active_route": "route_a",
    "current_segment": "station_b",
    "segment_index": 1,
    "segment_count": 3,
    "reason": "vehicle fault"
  },
  "fault": {
    "available": true,
    "has_fault": true,
    "summary": "left_drive=7",
    "left_drive_fault_code": 7,
    "right_drive_fault_code": 0,
    "steering_fault_code": 0,
    "lift_fault_code": 0
  },
  "vehicle": {
    "available": true,
    "battery_percent": 12.5,
    "enabled": true,
    "auto_mode": true,
    "emergency_stopped": false,
    "soft_emergency_stop": false,
    "parking_brake": false,
    "interlock": true,
    "mode": "auto",
    "velocity_mps": 0.0
  },
  "safety_status": "vehicle fault",
  "alarms": [
    {"level": "ERROR", "code": "VEHICLE_FAULT", "message": "left_drive=7"},
    {"level": "WARN", "code": "LOW_BATTERY", "message": "battery 12.5% <= 20.0%"}
  ]
}
```

调度系统第一版推荐消费 `/forklift/dispatch_report`；排障时再直接 echo 原始话题：

```bash
ros2 topic echo /forklift/task_status
ros2 topic echo /forklift/fault_state
ros2 topic echo /forklift/vehicle_state
ros2 topic echo /forklift/safety_gate/status
```
