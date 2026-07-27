# 两段式取托盘任务实现记录

日期：2026-07-21

## 1. 实现范围

本次实现把“两段式 N 号库位取托盘”方案落到 `forklift_task_manager` 和 `forklift_msgs` 中。

两段式流程定义为：

1. **第一段：导航到库位前等待位**
   - 复用已有 `go_to_station` 或 `execute_route`。
   - Task Manager 调用 Nav2。
   - 成功后只上报到位，不自动执行取叉。

2. **第二段：当前位置取托盘**
   - 新增 `execute_pallet_pickup` action。
   - 流程为：等待就绪、升叉到 `N+3` 检测高度、调用雷达检测偏移、校验偏移、下降到 N 层、横向侧移、低速插叉、轻抬托盘。

本次实现不包含真实货叉控制节点、真实雷达识别节点、真实低速相对运动节点的内部算法。Task Manager 只调用这些模块暴露出来的 action。

## 2. 新增 ROS Action 接口

在 `forklift_msgs/action/` 下新增 4 个 action。

### 2.1 ExecutePalletPickup.action

用途：调度或人工触发第二段“当前位置取托盘”任务。

接口：

```text
string task_id
string slot_id
---
bool success
string message
float64 offset_x_m
float64 offset_y_m
---
string phase
float32 progress
float64 current_height_m
```

说明：

- `slot_id` 表示要取的库位。
- `phase` 上报当前取叉阶段。
- `offset_x_m` / `offset_y_m` 返回雷达检测到的偏移，便于调试和追溯。

### 2.2 ForkMoveTo.action

用途：货叉控制节点执行目标货叉姿态，不让 Task Manager 直接控制阀电流。

接口：

```text
float64 target_height_m
float64 side_shift_m
float64 tilt_rad
---
bool success
string message
float64 final_height_m
---
float64 current_height_m
string phase
```

### 2.3 DetectPalletOffset.action

用途：雷达/感知节点在检测高度识别托盘位置。

接口：

```text
string slot_id
float64 scan_height_m
---
bool success
string message
float64 offset_x_m
float64 offset_y_m
float64 confidence
---
string phase
```

### 2.4 MoveRelative.action

用途：执行短距离、低速、相对位移插叉动作。底层运动仍应经过 safety gate。

接口：

```text
float64 distance_m
float64 max_speed_mps
---
bool success
string message
---
float64 remaining_distance_m
```

同时更新了 `forklift_msgs/CMakeLists.txt`，确保 Foxy/Humble 构建时生成这些 action 的 C/C++/Python 类型支持。

## 3. 新增配置

新增配置文件：

```text
forklift_task_manager/config/pallet_slots.yaml
```

示例内容：

```yaml
slots:
  slot_001:
    approach_station: slot_001_approach
    level_index: 1
    pick_height_m: 0.55
  slot_002:
    approach_station: slot_002_approach
    level_index: 2
    pick_height_m: 1.10

pickup_defaults:
  level_pitch_m: 0.55
  scan_level_offset: 3
  fork_insert_depth_m: 0.80
  pallet_clearance_m: 0.08
  max_offset_x_m: 0.20
  max_offset_y_m: 0.08
  min_detection_confidence: 0.75
  fork_timeout_sec: 8.0
  detection_timeout_sec: 3.0
  fine_motion_timeout_sec: 6.0
  fine_motion_speed_mps: 0.08
```

高度计算规则：

```text
H_pick = pick_height_m
H_scan = H_pick + scan_level_offset * level_pitch_m
```

默认 `scan_level_offset = 3`，即检测高度为 `N+3`。

同时在默认 `stations.yaml` 和 `routes.yaml` 中增加了两个示例等待位：

- `slot_001_approach`
- `slot_002_approach`

对应示例路线：

- `route_to_slot_001_approach`
- `route_to_slot_002_approach`

## 4. 新增 Python 模块

### 4.1 pallet_model.py

新增文件：

```text
forklift_task_manager/forklift_task_manager/pallet_model.py
```

职责：

- 加载并校验 `pallet_slots.yaml`。
- 定义 `PalletSlot`、`PickupDefaults`、`PalletPickupConfig`。
- 计算取货高度和雷达检测高度。

关键行为：

- 检查高度、层距、超时、速度、偏移阈值必须是合法数值。
- 检查 `min_detection_confidence` 必须在 `0.0 ~ 1.0` 范围内。
- 检查每个 slot 必须有 `approach_station`、`level_index`、`pick_height_m`。

### 4.2 pallet_pickup_state_machine.py

新增文件：

```text
forklift_task_manager/forklift_task_manager/pallet_pickup_state_machine.py
```

职责：

- 实现第二段取叉流程的 ROS 无关状态机。
- 通过注入的 `devices` 适配器调用货叉、雷达、低速相对运动动作。
- 支持 pause、resume、cancel 和 safety pause。

状态阶段：

```text
WAIT_READY
RAISE_TO_SCAN_HEIGHT
DETECT_OFFSET
VALIDATE_OFFSET
LOWER_TO_PICK_HEIGHT
APPLY_LATERAL_OFFSET
INSERT_FORK
LIFT_CLEARANCE
```

核心顺序：

1. 检查就绪。
2. 货叉升到 `H_scan`。
3. 雷达检测 `dx` / `dy` / `confidence`。
4. 校验偏移和置信度。
5. 货叉下降到 `H_pick`。
6. 按 `dy` 侧移。
7. 按 `dx + fork_insert_depth_m` 低速插叉。
8. 抬升 `pallet_clearance_m`。

失败策略：

- 雷达置信度不足：失败，不下降插叉。
- `dx` 超限：失败。
- `dy` 超限：失败。
- 货叉或低速运动 action 失败：失败。
- safety gate 触发：暂停当前动作。

## 5. Task Manager 节点改动

修改文件：

```text
forklift_task_manager/forklift_task_manager/task_manager_node.py
```

### 5.1 新增 action server

新增：

```text
execute_pallet_pickup
```

类型：

```text
forklift_msgs/action/ExecutePalletPickup
```

该 action 只执行第二段取叉，不负责导航。

### 5.2 新增设备 action client 适配层

新增内部类：

```text
PickupDeviceActions
```

连接以下 action：

- `/forklift/fork/move_to`
- `/forklift/perception/detect_pallet_offset`
- `/forklift/fine_motion/move_relative`

这些 action 名可以通过 launch 参数覆盖。

适配层负责：

- action server 可用性检查。
- action goal 发送。
- action result 转换为状态机回调。
- 超时 timer。
- cancel 时取消未完成 action 并清理 timer。

### 5.3 任务互斥

新增互斥规则：

- 导航任务运行中，拒绝取叉任务。
- 取叉任务运行中，拒绝导航任务。
- 取叉任务运行中，拒绝 `/goal_pose`。
- 取叉任务运行中，`go_to_station` 返回失败。

### 5.4 等待位约束

新增参数：

```text
enforce_pallet_approach_station
```

默认值：

```text
true
```

行为：

- Task Manager 记录最近一次成功完成的导航目标站点。
- 启动 `execute_pallet_pickup(slot_id)` 时，要求最近完成站点等于该 slot 配置里的 `approach_station`。
- 如果不满足，则拒绝取叉任务。

这样可以防止车辆还没到目标库位前等待位时误触发取叉。

### 5.5 pause / resume / cancel

已有 service 继续复用：

- `pause`
- `resume`
- `cancel`

现在会根据当前活跃任务类型分别作用于：

- 导航状态机 `TaskStateMachine`
- 取叉状态机 `PalletPickupStateMachine`

### 5.6 `/forklift/task_status`

取叉段也复用 `/forklift/task_status` 上报。

取叉任务上报字段：

- `state`：`RUNNING` / `PAUSED` / `SUCCEEDED` / `FAILED` / `IDLE`
- `active_route`：`pickup:<slot_id>`
- `current_segment`：当前 phase，例如 `RAISE_TO_SCAN_HEIGHT`
- `segment_index`：phase index
- `segment_count`：phase 总数
- `reason`：失败或完成原因

## 6. Launch 改动

修改文件：

```text
forklift_task_manager/launch/task_manager.launch.py
```

新增 launch 参数：

```text
pallet_slots_file
fork_move_to_action
detect_pallet_offset_action
move_relative_action
enforce_pallet_approach_station
```

默认值：

```text
pallet_slots_file:=<package_share>/config/pallet_slots.yaml
fork_move_to_action:=/forklift/fork/move_to
detect_pallet_offset_action:=/forklift/perception/detect_pallet_offset
move_relative_action:=/forklift/fine_motion/move_relative
enforce_pallet_approach_station:=true
```

## 7. 新增文档

新增设计文档：

```text
forklift_task_manager/docs/two_stage_pallet_pickup_task.md
```

内容包括：

- 背景。
- 设计目标。
- 总体流程图。
- 第一段导航。
- 第二段取叉。
- ROS 接口。
- 配置项。
- 异常处理。
- Task Manager 职责边界。
- 测试计划。
- 默认假设。

## 8. 新增测试

新增测试文件：

```text
forklift_task_manager/test/test_pallet_model.py
forklift_task_manager/test/test_pallet_pickup_state_machine.py
```

覆盖内容：

- `pallet_slots.yaml` 配置加载。
- `H_scan = H_pick + 3 * level_pitch_m` 计算。
- 非法置信度配置拒绝。
- 缺少 `approach_station` 拒绝。
- 正常取叉流程。
- 雷达低置信度失败。
- `dx` 超限失败。
- pause / resume / cancel。

## 9. 为什么使用 Python

本次逻辑使用 Python 的理由：

1. 现有 `forklift_task_manager` 已经是 Python package。
2. 本逻辑属于秒级业务编排，不是高频闭环控制。
3. Task Manager 只调用 action，不直接发 CAN、阀电流或 `/cmd_vel`。
4. 底层实时控制、安全拦截、车辆接口仍由独立底层节点负责。
5. Python 状态机便于现场调试和快速调整流程。
6. ROS2 action client/server 在 Python 中和当前 Nav2 编排风格一致。

职责边界：

```text
Python Task Manager：慢速任务编排、状态、失败原因、action 调用
底层控制节点：实时控制、闭环执行、安全链路、CAN/阀控/运动输出
```

## 10. 验证记录

### 10.1 宿主机验证

宿主机 ROS 环境为 Humble。

执行过：

```bash
python3 -m py_compile \
  forklift_task_manager/forklift_task_manager/pallet_model.py \
  forklift_task_manager/forklift_task_manager/pallet_pickup_state_machine.py \
  forklift_task_manager/forklift_task_manager/task_manager_node.py
```

结果：通过。

执行过：

```bash
PYTHONPATH=forklift_task_manager PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  forklift_task_manager/test/test_route_model.py \
  forklift_task_manager/test/test_pallet_model.py \
  forklift_task_manager/test/test_pallet_pickup_state_machine.py
```

结果：

```text
16 passed
```

执行过 focused build/test：

```bash
colcon build --packages-select forklift_msgs forklift_task_manager
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select forklift_task_manager
```

结果：

```text
24 passed
```

### 10.2 Foxy Docker 验证

使用镜像：

```text
forklift-nav2:foxy
```

Foxy Docker 内 Python 版本：

```text
Python 3.8.10
```

执行 build：

```bash
bash scripts/foxy_docker_run.sh bash -lc '
  set -eo pipefail
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  colcon --log-base log_foxy build \
    --packages-select forklift_msgs forklift_task_manager \
    --symlink-install \
    --build-base build_foxy \
    --install-base install_foxy \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
'
```

结果：

```text
2 packages finished
```

执行 test：

```bash
bash scripts/foxy_docker_run.sh bash -lc '
  set -eo pipefail
  set +u
  source /opt/ros/foxy/setup.bash
  source /workspace/install_foxy/setup.bash
  set -u
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  colcon --log-base log_foxy test \
    --packages-select forklift_task_manager \
    --build-base build_foxy \
    --install-base install_foxy \
    --event-handlers console_direct+
  colcon --log-base log_foxy test-result \
    --test-result-base build_foxy \
    --verbose
'
```

结果：

```text
24 passed in 0.29 seconds
Summary: 150 tests, 0 errors, 0 failures, 0 skipped
```

## 11. 后续需要接入的真实模块

本次 Task Manager 已经完成编排层实现，但现场运行还需要对应 action server：

1. 货叉控制节点提供：

```text
/forklift/fork/move_to
```

2. 雷达/感知节点提供：

```text
/forklift/perception/detect_pallet_offset
```

3. 低速相对运动节点提供：

```text
/forklift/fine_motion/move_relative
```

这些节点需要保证：

- 可取消。
- 有清晰的 success/message。
- 超时或故障时返回失败。
- 底盘运动不绕过 `forklift_safety`。

## 12. 注意事项

- 默认 `enforce_pallet_approach_station=true`，第二段取叉必须在成功导航到对应等待位后才能启动。
- 如果现场需要人工停车后直接测试取叉，可以临时用 launch 参数关闭该限制：

```bash
enforce_pallet_approach_station:=false
```

- `dx` / `dy` 的正负方向必须由雷达节点固定并记录，否则侧移和插叉方向会有风险。
- 取叉段中的低速相对运动必须仍然经过 safety gate，不允许 Task Manager 直接发布底盘控制命令。
- 真实货叉高度闭环、侧移能力限制、雷达识别算法不在 Task Manager 内实现。

## 13. TODO List

### 13.1 雷达检测 adapter

当前状态：

- Task Manager 已经定义并调用 `DetectPalletOffset.action`。
- 当前仓库中还没有真实雷达 action server。
- 该 action 不是模拟节点，也不会自己产生假数据。

后续需要实现一个雷达适配节点：

```text
雷达原始 topic/message
  -> lidar/pallet detector adapter
  -> /forklift/perception/detect_pallet_offset action server
  -> Task Manager
```

建议职责：

- 订阅雷达或感知模块持续发布的托盘检测 message。
- 收到 `DetectPalletOffset.action` goal 后，开始等待当前 `slot_id` 的有效检测结果。
- 校验 message 时间戳，避免使用旧数据。
- 校验目标库位、置信度、偏移范围。
- 返回 `offset_x_m`、`offset_y_m`、`confidence`。
- 支持 timeout、cancel 和明确失败原因。

建议不要让 Task Manager 直接订阅雷达 topic。雷达 message 通常是持续流，而 Task Manager 需要的是“一次检测请求 -> 一次结果”的任务语义，这层转换更适合放在 adapter/action server 中。

### 13.2 叉臂控制 adapter

当前状态：

- Task Manager 已经定义并调用 `ForkMoveTo.action`。
- 已新增第一版叉臂控制 adapter：
  - `forklift_vehicle_interface/forklift_vehicle_interface/fork_control_adapter.py`
  - executable: `fork_control_adapter`
  - launch: `forklift_vehicle_interface/launch/fork_control_adapter.launch.py`
- Task Manager 不直接发 CAN，不直接控制阀电流，也不做叉臂闭环。

当前没有新增自定义 interface。第一版复用已有接口：

```text
上层任务接口：ForkMoveTo.action
控制输出接口：ForkliftControlCommand
反馈输入接口：sensor_msgs/JointState
```

默认 topic：

```text
Action server:
/forklift/fork/move_to

Command output:
/forklift/control_cmd_raw

Fork feedback input:
/forklift/fork/joint_state
```

`/forklift/fork/joint_state` 默认 joint name：

```text
fork_height_m
fork_side_shift_m
fork_tilt_rad
```

后续 CANopen 拉线编码器节点只需要把编码器位置换算成 `fork_height_m`，并发布到该 JointState topic，即可作为高度闭环反馈来源。如果现场侧移或倾角也要闭环，需要继续接入对应位置反馈。

通过 CAN 控制叉臂电机时，整体链路如下：

```text
Task Manager
  -> /forklift/fork/move_to action
  -> fork control adapter
  -> CAN command
  -> 叉臂电机 / 泵 / 阀 / 控制器
  -> CAN feedback / 编码器 / 限位 / 高度传感器
  -> fork control adapter 判断到位
  -> action result 返回 Task Manager
```

当前 adapter 已实现的逻辑：

- 接收 `ForkMoveTo.action` goal：
  - `target_height_m`
  - `side_shift_m`
  - `tilt_rad`
- 按顺序控制三个轴：
  1. 高度先到位。
  2. 再执行侧移。
  3. 最后执行倾角。
- 将目标高度、侧移、倾角转换为 `ForkliftControlCommand`：
  - 泵电机 rpm。
  - 升降阀电流。
  - 下降阀电流。
  - 侧移阀电流。
  - 倾斜阀电流。
- 默认发布到 `/forklift/control_cmd_raw`，再由 `forklift_safety` 转发到 `/forklift/control_cmd`。
- 到位后发布零泵速/零阀电流命令并返回 success。
- cancel / timeout / feedback missing 时发布停止命令并返回失败。

高度目标和比例阀电流的关系：

- 要升到多高由 `target_height_m` 决定。
- 对 N 号库位取叉时，`target_height_m` 来自 `pallet_slots.yaml`：
  - `H_pick = pick_height_m`
  - `H_scan = H_pick + scan_level_offset * level_pitch_m`
- 比例阀电流不是高度设定值，而是根据高度误差计算出来的运动强度：

```text
height_error = target_height_m - current_height_m
ratio = abs(height_error) / height_slow_zone_m
valve_ma = min_ma + (max_ma - min_ma) * ratio
```

- 当距离目标高度较远时，阀电流接近 `*_valve_max_ma`。
- 当距离目标高度进入 `height_slow_zone_m` 后，阀电流逐渐减小。
- 当 `abs(height_error) <= height_tolerance_m` 时，认为高度到位，停止泵和阀输出。

当前默认高度控制参数：

```text
height_tolerance_m = 0.01
height_slow_zone_m = 0.08
lift_pump_rpm = 1500
lift_valve_min_ma = 250
lift_valve_max_ma = 800
lower_valve_min_ma = 180
lower_valve_max_ma = 650
```

这些参数已集中放到：

```text
forklift_vehicle_interface/config/fork_control_adapter.yaml
```

现场调试时优先修改 yaml 或通过 launch 指定新的 `params_file`，不要直接改代码默认值。例如：

```bash
ros2 launch forklift_vehicle_interface fork_control_adapter.launch.py \
  params_file:=/path/to/site_fork_control_adapter.yaml
```

CAN 协议编码对应关系：

```text
pump_rpm                  -> CAN 0x303 Data[0:2], uint16 little-endian
lower_valve_ma > 0        -> CAN 0x303 Data[3] bit2
lower_valve_ma / 10       -> CAN 0x403 Data[0]
lift_valve_ma / 10        -> CAN 0x403 Data[1]
side_shift_left_ma / 10   -> CAN 0x403 Data[4]
side_shift_right_ma / 10  -> CAN 0x403 Data[5]
tilt_forward_ma / 10      -> CAN 0x403 Data[6]
tilt_backward_ma / 10     -> CAN 0x403 Data[7]
```

示例：

```text
pump_rpm = 1500
lift_valve_ma = 400

0x303 = DC 05 00 00 00 00 00 00
0x403 = 00 28 00 00 00 00 00 00
```

其中 `lift_valve_ma=400mA` 被编码为 `400 / 10 = 40 = 0x28`。如果现场协议不是 `mA / 10`，需要调整 `curtis_can_codec.py` 中的阀电流编码函数，或在 adapter 中增加现场标定曲线。

仍需现场接入的反馈：

```text
CANopen 拉线编码器
  -> encoder count
  -> height_m = (count - zero_count) * meter_per_count + height_offset_m
  -> sensor_msgs/JointState(name=["fork_height_m"], position=[height_m])
```

建议反馈：

- 当前叉高。
- 当前侧移位置。
- 当前倾角。
- 限位开关。
- 故障码。
- 电机状态。

- 做闭环判断：

```text
abs(current_height_m - target_height_m) <= height_tolerance_m
```

- 到位后停止输出并返回 success。
- cancel 时立即停止升降、侧移、倾斜输出。
- timeout 时停止输出并返回失败。
- 故障或限位触发时停止输出并返回明确原因。

第一版 adapter 可以先复用项目已有控制消息：

```text
ForkMoveTo.action server
  -> ForkliftControlCommand
  -> pump_rpm / lift_valve_ma / lower_valve_ma / side_shift_* / tilt_*
  -> vehicle_interface 编码成 CAN
```

如果现场叉臂 CAN 协议与底盘 CAN 协议分离，也可以由 adapter 直接使用 SocketCAN。无论哪种实现，原则都是：叉臂闭环、CAN 细节、限位和故障处理放在 adapter；Task Manager 只做任务编排。

### 13.3 低速相对运动 adapter

当前状态：

- Task Manager 已经定义并调用 `MoveRelative.action`。
- 当前仓库中还没有真实低速相对运动 action server。

后续需要实现短距离低速移动适配节点：

```text
Task Manager
  -> /forklift/fine_motion/move_relative action
  -> fine motion adapter
  -> 底盘控制链路
  -> forklift_safety
  -> vehicle_interface / CAN
```

建议职责：

- 接收相对移动距离 `distance_m` 和最大速度 `max_speed_mps`。
- 控制车辆低速前进或后退。
- 使用 odom / 编码器 / 控制器反馈判断距离是否完成。
- 所有底盘控制必须经过 `forklift_safety`。
- 支持 timeout、cancel、急停暂停和明确失败原因。
