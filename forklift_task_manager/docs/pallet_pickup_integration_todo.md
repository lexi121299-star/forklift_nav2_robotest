# 取托盘任务联调后续待办

## 目的

本文记录 Task Manager 的两段式取托盘任务在完成基础联调后，进入实车闭环、调度接入和量产准备前仍需完成的工作。

当前任务链为：

```text
NAVIGATE_TO_APPROACH
-> RAISE_TO_SCAN_HEIGHT
-> DETECT_OFFSET
-> VALIDATE_OFFSET
-> LOWER_TO_PICK_HEIGHT
-> APPLY_LATERAL_OFFSET
-> INSERT_FORK
-> LIFT_CLEARANCE
```

当前已具备：

- `ExecuteDispatchTask` action 的 `NAVIGATE` 和 `PICKUP` 类型。
- `/forklift/dispatch/task_status` 的任务、阶段、进度和失败原因上报。
- `PICKUP` 的导航到等待位和后续取货 phase 编排。
- 可选的 `/forklift/fork/command`、`/forklift/fork/state` 叉臂位置接口客户端。
- pause、resume、cancel 与普通导航/托盘任务的运动控制互斥。

`DROPOFF` 已在 action 协议中预留，但尚未实现。

## 1. 联调前置条件

在开始真实取货动作前，以下接口必须稳定可用。

### 1.1 TC397 叉臂接口

TC397 bridge 需要提供：

```text
/forklift/fork/command  forklift_msgs/msg/ForkPositionCommand
/forklift/fork/state    forklift_msgs/msg/ForkState
```

确认项目：

- `height_m`、`side_shift_m`、`tilt_rad` 的零位、单位和正负方向。
- `active_command_id` 能对应 ROS 下发的 `command_id`。
- `STATUS_REACHED` 只在实际进入位置容差后发布。
- `STATUS_FAULT`、`fault_code`、位置有效位、限位开关状态可正确上报。
- STOP 命令能立即停止当前叉臂动作。
- 叉臂状态发布频率和超时设置满足 Task Manager 的状态新鲜度要求。

Task Manager 初始保持：

```yaml
use_fork_position_topics: false
```

完成上述接口验证后才切换为：

```yaml
use_fork_position_topics: true
```

### 1.2 雷达接口

雷达需要给出托盘口相对 `base_link` 的测量，而不是直接给电机或阀控制量。

最小数据契约：

```text
header.stamp
header.frame_id = base_link
offset_x_m        # 前后偏移
offset_y_m        # 横向偏移
confidence        # 识别置信度
valid             # 当前测量是否有效
```

必须确认：

- `offset_x_m`、`offset_y_m` 的正方向。
- 量测点是托盘前沿、托盘口中心还是叉孔中心。
- 雷达输出时间戳是否可用于判定超时。
- 无托盘、遮挡、低置信度和多托盘时的输出规则。
- 检测时叉臂必须位于 `N+3` 高度的条件是否由雷达侧校验。

雷达适配层负责把真实消息转换为 Task Manager 使用的 `DetectPalletOffset` 结果；Task Manager 不实现识别算法。

### 1.3 库位配置

每个真实库位必须补齐并现场复核：

```yaml
slot_xxx:
  approach_station: slot_xxx_approach
  level_index: N
  pick_height_m: <N 层取货高度>
```

还需要确认全局默认值：

- `level_pitch_m`
- `scan_level_offset: 3`
- `fork_insert_depth_m`
- `pallet_clearance_m`
- 最大 `dx`、`dy`
- 最小检测置信度
- 各 action timeout

库位 `approach_station` 必须存在于站点配置中；启动时会进行校验。

## 2. 推荐联调顺序

每次只新增一个真实动作。前一阶段失败时，不进入后一阶段。

### 阶段 A: 只验证接口与状态

1. 启动 Task Manager，不发取货任务。
2. 验证持续收到 `/forklift/fork/state`。
3. 分别发送高度、侧移、倾角单轴命令。
4. 验证 `command_id`、状态、位置、到位、停止、故障和限位反馈。
5. 验证雷达消息的 frame、单位、时间戳和正负方向。

通过标准：无真实动作超时；错误和故障能被清楚识别。

### 阶段 B: 到等待位和升降叉

1. 下发 `PICKUP`，只允许 `NAVIGATE_TO_APPROACH`。
2. 到达等待位后，执行 `RAISE_TO_SCAN_HEIGHT`。
3. 验证叉臂到达 `H_pick + 3 * level_pitch_m`。
4. 人工 cancel，验证叉臂停止、任务进入 `CANCELED`。

通过标准：导航失败不触发叉臂；叉臂超时或故障不会继续检测或插叉。

### 阶段 C: 雷达检测和下降

1. 在正常托盘、空库位、偏移托盘三种场景运行。
2. 验证 `DETECT_OFFSET` 和 `VALIDATE_OFFSET` 的成功、超时、低置信度、`dx/dy` 超限。
3. 仅在偏移有效后执行 `LOWER_TO_PICK_HEIGHT`。
4. 记录测得偏移与人工测量的差异。

通过标准：空库位和低置信度绝不进入插叉阶段。

### 阶段 D: 侧移、插叉和轻抬

1. 先启用 `APPLY_LATERAL_OFFSET`，验证侧移正负方向和限位处理。
2. 低速启用 `INSERT_FORK`，使用保守插叉深度。
3. 最后启用 `LIFT_CLEARANCE`，确认托盘离架。
4. 验证插叉期间的 Safety Gate、急停、取消和超时。

通过标准：任何异常都停止底盘和叉臂，不绕过 `forklift_safety`。

## 3. 失败、暂停与恢复策略

每个 phase 都应上报明确的 `phase`、`reason`、`task_id` 和 `slot_id`。

| 情况 | 当前任务处理 | 是否可 resume |
| --- | --- | --- |
| 导航失败 | `FAILED`，不执行叉臂动作 | 否，重新下发或人工确认 |
| 货叉状态超时/故障 | `FAILED`，停止叉臂 | 故障清除后重新下发 |
| 雷达超时/无托盘/低置信度 | `FAILED`，不下降插叉 | 重新检测或重新下发 |
| 偏移超限 | `FAILED`，不插叉 | 人工调整车辆或托盘后重新下发 |
| Safety Gate/急停 | `PAUSED`，停止当前动作 | 条件恢复后显式 resume |
| 调度取消 | `CANCELED`，停止当前动作 | 否，重新下发新任务 |

第一版原则：取货失败后不自动插叉重试，不自动换库位，不自动退出库位。此类策略应由调度系统或后续上层任务树决定。

### 3.1 RViz 托盘靠近的受控退出

RViz 托盘靠近任务的最终低速靠近阶段只豁免选中托盘自身的矩形区域，矩形外的货架、人员和未知 scan 点仍会触发停车。该机制不是通用的 scan 关闭开关。

- 正常完成最终靠近后，使用 `/forklift/pallet_approach/egress` 退出。
- 若 `pallet_final_approach` 被框外 scan 障碍或动作超时停止，Task Manager 保留该次选中托盘目标，并开放一次 `/forklift/pallet_approach/egress_recovery`。
- `egress_recovery` 只执行配置的低速、有限距离退出；退出完成、失败或取消后都会清除托盘豁免状态。
- 未进入最终靠近阶段、或任务在此前失败时，恢复服务会拒绝请求，避免对未知托盘位置盲目豁免。

实车命令：

```bash
ros2 service call /forklift/pallet_approach/egress std_srvs/srv/Trigger "{}"
ros2 service call /forklift/pallet_approach/egress_recovery std_srvs/srv/Trigger "{}"
```

## 4. 放货任务链

`TYPE_DROPOFF` 的建议任务链：

```text
NAVIGATE_TO_DROP_APPROACH
-> ALIGN_TO_DROP
-> INSERT_TO_DROP
-> LOWER_TO_DROP_HEIGHT
-> RELEASE_LOAD
-> RETREAT_FROM_DROP
-> MOVE_FORK_TO_SAFE_HEIGHT
```

实现前需要先增加和验证：

- `has_load` 或等价的载货状态来源。
- 放货库位高度、等待位和最终停靠位。
- 托盘已放下/叉根无载荷的判据。
- 退出货架后的安全高度。
- 放货失败后的停留、退出和人工接管策略。

放货应在取货实车闭环稳定后再实现，避免同时联调两条高风险动作链。

## 5. 调度系统接入

ROS 内部统一入口保持为：

```text
/execute_dispatch_task  forklift_msgs/action/ExecuteDispatchTask
/forklift/dispatch/task_status  forklift_msgs/msg/DispatchTaskStatus
```

后续增加独立的调度 adapter：

1. 接收调度协议任务。
2. 生成或校验唯一 `task_id`。
3. 转换为 `ExecuteDispatchTask` goal。
4. 转发 feedback、最终 result 和 `/forklift/dispatch/task_status`。
5. 映射 pause、resume、cancel、故障确认和人工接管命令。

调度 adapter 不应直接调用 Nav2、货叉 CAN 或底盘 CAN。

## 6. 量产前需要补充

- 持久化任务日志：任务输入、phase 时间线、雷达偏移、叉臂反馈、失败原因。
- 明确错误码表：导航、雷达、叉臂、底盘、安全门、调度协议。
- 参数版本管理：车型、地图、库位、货叉高度、雷达坐标外参。
- 回归测试：mock Nav2、mock 叉臂、mock 雷达、mock fine motion。
- 实车录包清单：`/scan`、`/odom`、`/fusion/localization`、`/tf`、控制命令、叉臂 command/state、雷达结果、任务状态。
- 安全评审：ROS 侧保护只作为运行保护；最终功能安全急停仍由嵌入式安全链路负责。

## 7. 当前下一步

先完成 TC397 的 `ForkPositionCommand/ForkState` 单轴闭环验证，再接入真实雷达消息。两者通过后，才开始库位的完整 `PICKUP` 实车联调。
