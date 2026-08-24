# 车队调度协调与车端避障设计

## 1. 架构结论

车队会车不能只押在调度系统，也不能让每辆车完全自由重规划。推荐采用三层权责：

```text
车队调度 / coordinator
  负责：已知受控车辆之间的路线冲突、资源预约、通行优先级和死锁解除
  权限：可以限制车辆进入某区域，但不能证明该区域现场一定安全

车端 planning / behavior
  负责：执行已授权路线、等待/恢复、上报阻塞、在授权走廊内做有限重规划
  权限：可以选择更保守的行为，不能绕过调度进入未授权资源区

车端 safety
  负责：基于现场传感器、车辆状态和短时扫掠做最终减速/停车
  权限：拥有最终否决权；即使调度已放行，现场有障碍也必须停车
```

核心规则：

> 调度系统负责“不要计划相撞”，车端 safety 负责“现场绝不硬撞”；
> planning 负责在两者约束内把任务完成。

第一版对其他受控车辆采用“预约 + 停车让行”，不让两车在窄道中各自自由绕行。
持续阻挡时，先由调度系统给其中一辆换路或下达受控退让；以后只在宽阔、明确允许绕行的区域，
开放车端局部自主避让。

### 1.1 当前阶段决策：直接 `/scan` safety 暂缓

> **决策日期：2026-07-06。** safety gate 直接订阅 `/scan`、绕过 costmap 做快速碰撞停车，
> 暂不作为当前仿真和第一轮低速联调的开发前置。当前继续使用已经跑通的
> `/scan → local costmap → controller safety + independent safety gate` 双重检查链路。

暂缓原因：

- 现有链路已经能支持仿真、dry-run、架空轮台架和封闭场地低速功能测试；
- 当前更紧迫的是完成 P2.3 CAN/反馈/里程计台架验收，以及把停车后的等待/恢复行为跑顺；
- 直接 scan 层会新增 TF、scan watchdog、自反射过滤和另一套碰撞诊断，应该作为独立安全增强实施，
  不与当前上车基本链路混在一起。

暂缓不等于取消。出现以下任一条件时，必须重新提升优先级：

- 实测 `scan → costmap → gate → CAN → 零速` 延迟或停止距离不满足安全余量；
- 运行速度提高、载荷增大，现有固定保护区不够；
- 进入人车混行、人工叉车混行或非封闭区域；
- local costmap 的 2 Hz 对外发布成为 safety gate 明确的响应瓶颈；
- 需要把 ROS 软件链路纳入正式安全验收，而不只是功能测试。

在直接 scan 层完成前允许的测试范围：

- Gazebo 仿真；
- `VEHICLE_DRY_RUN=true` 全链路演练；
- 驱动轮架空、硬件急停可用的 CAN 台架；
- 清空并隔离的场地内，以最低速、人工全程监护的短距离测试。

当前明确不允许据此宣称已具备人车混行安全能力。硬件急停/安全雷达、人工接管和实车制动距离验收
仍是任何落地测试的前提，不能由 costmap 软件停车替代。

## 2. 障碍类型与处理责任

| 对象 | 主要信息来源 | 首选策略 | 最终兜底 |
|---|---|---|---|
| 同一系统管理的无人叉车 | 调度状态、路线、预约、车端感知 | 调度提前消除冲突，指定谁等待 | 车端 `/scan` / tracked object 仍可停车 |
| 人工叉车或外来车辆 | 车端感知 | 减速、停车、预测后有限避让 | safety gate / 硬件安全系统 |
| 行人 | 车端感知 | 提前减速、让行、等待；不主动贴身绕行 | safety gate / 硬件安全系统 |
| 托盘、掉落物、临时施工 | `/scan`、点云或检测 | 当静态障碍停车；持续存在再请求换路 | safety gate |
| 调度登记存在但现场未识别的车辆 | 调度约束 | 不进入其预约资源 | 现场感知独立复核 |
| 现场识别但调度未知的对象 | 车端感知 | 立即按未知动态障碍处理并上报 | safety gate |

调度信息只能增加约束，不能用“调度说没人”覆盖现场传感器的障碍结果。

## 3. 调度侧最小方案：资源区预约

仓库地图先抽象为受控资源：

- 窄巷道 `aisle_segment`；
- 交叉口 `intersection`；
- 门洞、坡道、盲区 `conflict_zone`；
- 装卸位、充电位 `station`；
- 必须单向通行或禁止会车的 lane。

每个资源配置入口停止线、允许方向、容量和占用长度。车辆进入前必须申请 lease：

```text
APPROACHING_RESOURCE
  -> REQUESTING_LEASE
  -> WAITING_FOR_LEASE
  -> GRANTED
  -> OCCUPYING
  -> RELEASED
```

预约记录至少包含：

```text
resource_id
robot_id
lease_id
route_id / task_id
direction
priority
granted_at
lease_expiry
estimated_enter_time
estimated_exit_time
```

车辆周期上报：

```text
pose / velocity / gear
active route and segment
occupied resource
requested resources
task priority
vehicle/safety health
blocked reason and blocking track_id（若有）
heartbeat timestamp
```

### 3.1 会车规则

- 单容量窄道同一时刻只允许一个方向获得 lease。
- 未获授权的车辆在停止线前进入 `WAITING_FOR_LEASE`，不能靠本地 planner 穿进去试探。
- 优先级使用确定性规则，例如任务安全等级、已等待时间、到达顺序和固定 robot ID tie-break；
  不能让两车各自判断“对方应该让”。
- 使用 aging 防止低优先级车辆永久饥饿。
- 已进入资源的车辆通常优先退出，不在窄道中途让另一辆钻入。
- 需要退让时只能由协调器指定一辆车执行，并给出已验证的退让点/原子路径。

### 3.2 死锁与断联

- coordinator 维护 wait-for graph，检测循环等待。
- 发现死锁后只选择一辆 victim 释放后续预约、换路或受控退让；其他车辆保持等待。
- lease 必须续租，但通信断开不能立刻把车辆当前占用的资源判为空闲。
- 车辆 heartbeat 丢失时：未进入资源的车辆停车；已进入资源的车辆按安全策略停车并保持资源占用，
  直到重新确认位置或人工解除。
- coordinator 重启后先重建所有车辆实际占用状态，再发新 lease。

## 4. 车端行为与仲裁

车端建议状态：

```text
RUNNING
APPROACHING_RESOURCE
WAITING_FOR_LEASE
WAITING_FOR_OBSTACLE
REQUESTING_REPLAN
EXECUTING_REROUTE
BLOCKED
FAILED / MANUAL_ASSIST
```

每个控制周期按以下优先级仲裁：

```text
1. emergency / hardware safety
2. safety gate collision or sensor-health stop
3. scheduler resource permission
4. dynamic behavior decision (slow/wait/replan)
5. controller tracking command
```

典型行为：

- 已获得 lease，但 `/scan` 看到障碍：立即停车并上报 `BLOCKED_IN_GRANTED_RESOURCE`；不能继续走。
- 未获得 lease，但本地 costmap 看起来为空：继续等待；不能自行进入。
- 动态障碍短时横穿：`SLOW/WAIT`，风险解除并经过 hysteresis 后继续。
- 障碍持续存在：上报调度，请求 reroute；不能高频重复请求。
- reroute 没有安全通道：保持 `BLOCKED`，最终任务暂停/人工介入。
- 调度下发的新路线必须重新经过车端静态碰撞、运动学和 safety 检查。

### 4.1 长时间堵塞的职责边界（2026-07-08）

结论：长期堵塞后的“换哪条通道”由调度系统决定；新路线内部“具体怎么安全行驶”仍由车端
Nav2/planner 计算。调度系统不直接消费 `/scan` 生成转角或 CAN 指令，车端也不能因为局部
costmap 发现一条空隙，就自行驶入未授权的巷道、交叉口或其他车辆已经预约的资源。

```text
短时障碍、局部减速/停车       -> 车端 planning + safety
换通道、换任务路线、多车让行   -> 调度系统 / coordinator
新路线内部的几何与运动学规划   -> 车端 Nav2/planner
任何现场碰撞风险               -> 车端 safety 最终否决
```

第一版建议流程：

```text
检测到前方障碍
  -> 立即减速/停车，保持原 NavigateToPose goal
  -> 短时等待；障碍清除后继续同一 goal
  -> 持续阻挡时上报 BLOCKED，并保持停车
  -> 调度决定继续等待、安排其他车辆先走、换路线或人工介入
  -> 若调度下发新路线，车端取消旧 goal，再用更新后的 costmap 规划并执行新路线
```

建议把时间做成参数，而不是写死在状态机中。第一轮低速联调可用以下初值起步，实测后再调整：

- `0~5 s`：`WAITING_FOR_OBSTACLE`，按临时动态障碍处理，保留原 goal；
- `5~15 s`：继续停车，同时发送一次阻塞预警，后续按低频 heartbeat 更新，避免重复刷请求；
- `>=15 s`：进入 `BLOCKED` 并请求调度决策；
- `>=30 s`：若仍无可执行决策，任务转 `PAUSED/FAILED` 或请求人工介入，不能无限重试。

上报调度的最小阻塞消息建议包含：

```text
vehicle_id
task_id / route_id
current_pose
current_segment / occupied_resource_id
goal_pose
blocked_since / blocked_duration
blocked_reason
nearest_obstacle_distance
blocking_track_id（若 tracked object 可用）
safety and localization health
```

调度返回的决策至少区分：

```text
WAIT                 # 继续等待，不改变路线
YIELD_TO(vehicle_id) # 指定让行对象和规则
REROUTE(route_id)    # 下发已授权的新路线/资源序列
CONTROLLED_RETREAT   # 仅执行预先验证的退让点或原子路径
MANUAL_ASSIST        # 保持停车，等待人工处理
```

当前代码尚未完整实现上述 `WAITING_FOR_OBSTACLE -> BLOCKED -> 调度决策` 协议；现阶段已有的是
costmap 障碍停车、障碍清除后在原 goal 仍有效时恢复，以及 Nav2 自身超时/重试。P8.3 和调度接口
落地时应按本节补齐，不能把当前自动停车等同于已经具备长期堵塞换路能力。

## 5. 自主重规划的开放边界

### V1：预约停车，调度换路（推荐先做）

- 会车、交叉口和窄道由调度分配通行权。
- 车端只执行 wait/release，不绕其他受控车辆。
- 持续阻挡由调度系统重新选全局路线。
- 优点是行为确定、容易复盘，适合叉车和仓库窄道。

### V2：授权走廊内局部避让

满足以下条件才允许：

- 区域被标记为允许绕行且宽度足够；
- 新轨迹始终留在 coordinator 授权 corridor 内；
- 不进入其他车辆预约资源；
- ORU planner、controller 和 safety 都验证新轨迹；
- 绕行失败立即回到 WAIT，不反复左右切换。

### V3：带时间的多车/动态规划

只有 V1/V2 无法满足吞吐量时，再考虑：

- coordinator 使用 trajectory envelope / time reservation；
- 车端 ORU 从 `x/y/theta` 扩展到 `x/y/theta/time`；
- planner 输出 timed trajectory，controller 必须遵守到达时刻；
- 对受控车辆使用预约轨迹，对非受控目标继续使用感知预测和保守 safety。

ORU 的 `Trajectory`、`TimeEnvelope`、`CoordinatorTime` 可参考时间预约的消息和思路；
当前本地 `orunav_coordinator_fake` 明确不做真实协调，不能直接用作车队调度器。

## 6. 当前车端避障能力盘点

### 6.1 已有能力

- `/scan` 同时进入 local/global costmap。
- local costmap 以 5 Hz 更新，controller 以 10 Hz运行。
- controller 对候选自车轨迹做 costmap footprint collision，并配置 1.25 m 减速、0.55 m停车。
- 独立 `forklift_safety` 以 20 Hz仲裁命令，检查 costmap、车辆短时预测姿态和完整 footprint。
- normal、recovery 命令统一经过 safety gate；急停、命令超时和 costmap stale 可停车。
- Gazebo 已验证障碍出现时停车、删除后恢复运动。

### 6.2 仍需加强

| 优先级 | 缺口 | 当前风险 | 建议改造 |
|---|---|---|---|
| **暂缓增强** | 硬安全链路仍经 costmap | safety gate 虽以 20 Hz运行，但新障碍依赖 2 Hz发布的 `/local_costmap/costmap_raw` | 当前低速受控测试沿用现有链路；满足 §1.1 任一触发条件后，增加直接 `/scan`/点云 collision monitor，costmap 作为第二信息源 |
| **P0** | 未测端到端响应与制动距离 | 0.55 m / 1.25 m目前是配置值，不是真车签收值 | 测量 sensor→TF→costmap→gate→CAN→制动全过程，按速度、方向、载荷建立停止距离表 |
| **P0** | 保护区未随速度/载荷动态变化 | 高速、重载和倒车仍可能使用不足余量 | 使用反应距离 + 制动距离 + 固定余量生成前进/倒车动态保护区 |
| **P0** | fail-closed 健康条件未全部启用 | `require_vehicle_state/fault_state/localization` 默认 false，部分反馈异常不会锁车 | 真车反馈稳定后设为 true，并分别测试消息丢失、stale、非法值和恢复流程 |
| **P0** | 传感器覆盖与遮挡未签收 | 单前向视场无法可靠保护倒车、车尾和货叉盲区 | 验证 360°或前后覆盖；检查货叉/载荷自遮挡、自反射过滤和不同举升高度 |
| **P0** | footprint/TF仍需实车标定 | 外参或轮廓错误会导致过晚停车或误停 | 按实车后轴基准标定 TF、车体/货叉/载荷 footprint，并做 RViz贴墙和实物边界试验 |
| **P0** | 当前“动态”测试不是真移动目标 | 验收脚本生成一个静止箱子再删除；正式场景还会 clear costmap、重新下发 goal | 增加连续移动 actor：横穿、迎面、同向慢行、突然停下、掉头和遮挡恢复；验收不允许人工 clear/reissue |
| **P1** | 自动清障与恢复不够自然 | 障碍离开后可能有 costmap 残影或 Nav2已 abort | 调整 clearing/TTL，增加 WAIT→RELEASE hysteresis；同一 goal 自动恢复，失败才进入受控 replan |
| **P1** | P8.3 行为状态机未完成 | 现在会停，但短挡/长挡、等待/绕行边界不明确 | 实现 `CLEAR/SLOW/WAIT/REPLAN/BLOCKED`、等待超时、replan cooldown和最大重试次数 |
| **P1** | 缺少端到端诊断 | 很难区分感知晚、TF晚、costmap晚还是制动慢 | 记录 scan stamp、costmap update、首次 slow/stop、gate输出、CAN发送、车辆零速时间和触发原因 |
| **P1** | recovery 场景安全策略仍需实车验收 | pivot/backoff 可能把车尾或货叉扫向障碍 | 对每种 recovery 独立做 swept footprint、方向保护区、最大距离/时长和人工接管验收 |
| **P2** | tracked-object 接口尚未落地 | 不能判断目标ID、速度、方向和横穿趋势 | 实现 `TrackedObjectArray`、质量校验、自车运动补偿和 stale watchdog |
| **P2** | 没有动态预测/TTC | 只能等目标进入当前 costmap后响应 | 按 `dynamic_obstacle_prediction_plan.md` 完成 CV预测、时序碰撞和 controller candidate拒绝 |
| **P2** | 没有目标不确定性处理 | ID切换、遮挡或速度噪声可能导致突然放行 | covariance膨胀、短时track memory、预测降级和保守release策略 |
| **P3** | 没有调度资源预约接口 | 多车只能在现场互相看到后停车，容易堵死 | 建 resource graph、lease、heartbeat、优先级、deadlock和reroute协议 |

## 7. 推荐实施顺序

```text
阶段 A：先完成当前第一轮低速联调门槛
  P2.3 CAN台架 + 现有costmap safety验证 + 实车响应/制动标定 + fail-closed + 传感器覆盖

阶段 B：把“停住以后怎么办”补齐
  WAIT/RELEASE/REPLAN/BLOCKED + 自动 clearing + moving actor acceptance

阶段 C：多车先用调度避免相遇
  资源区预约 + 停止线 + lease + 优先级 + 断联/死锁处理

阶段 D：增加对非受控动态目标的提前量
  tracked objects + CV prediction + TTC + controller 时序碰撞

阶段 E：最后提升效率
  调度 reroute -> 授权走廊局部避让 -> 必要时才做 ORU time lattice

阶段 F：按触发条件补直接传感器 safety
  safety gate直接订阅/scan或点云；独立watchdog、TF、扫掠碰撞和仿真/真车验收
```

阶段 A/B 是任何自主避障和车队调度的前提；阶段 F 的实际顺序由 §1.1 的触发条件决定，
如果实测延迟不达标或测试范围升级到人车混行，就必须前移，不能继续暂缓。

## 8. 验收原则

- **安全**：无碰撞、停止余量满足实车标定、任何数据源 stale 都有明确降级。
- **确定性**：相同会车场景得到相同优先级，不发生双方同时绕行或同时退让。
- **活性**：障碍离开后能自动恢复；持续阻挡最终 reroute、BLOCKED 或人工介入，不无限循环。
- **可解释**：每次停车能区分 obstacle、lease denied、sensor stale、vehicle fault、deadlock victim 等原因。
- **可回退**：关闭 prediction/local avoidance 后仍保留预约停车和 `/scan` safety 基线。

相关详细计划：

- 动态目标消息、预测、TTC 和 ORU 时间维：`dynamic_obstacle_prediction_plan.md`
- 真车 TF、footprint 和 safety 标定：`real_vehicle_tuning_guide.md`
- P8.1/P8.2 当前安全实现：`p8_safety_gate_notes.md`
