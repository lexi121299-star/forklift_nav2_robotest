# 动态障碍物预测与避让改造计划

## 1. 结论先行

受控车队之间的会车、窄道和交叉口冲突，优先由资源预约/调度协调解决；车端动态预测主要处理
行人、人工叉车、外来车辆和调度信息与现场不一致等情况。两者的权责、死锁和断联策略见
`fleet_coordination_and_onboard_avoidance.md`。

动态障碍物能力分三层，不应一次性全部塞进 ORU global planner：

```text
Layer 0  /scan + costmap + safety gate
         无论预测是否正常，都负责最后的减速/停车兜底

Layer 1  tracked objects + prediction + behavior
         判断障碍是否会相交，执行 continue / slow / wait / replan

Layer 2  time-aware local collision checking
         controller 按时间比较自车候选轨迹与障碍预测轨迹

Layer 3  x/y/theta/time global planning（按需）
         只有“等一等或局部避让”不够时，才扩展 ORU lattice 的时间维
```

推荐先完成 Layer 1–2。现有 `ForkliftMpcController` 已按时间步预测候选控制，
在这里增加动态碰撞检查，比直接把 ORU global planner 改成四维搜索风险更小、收益更直接。

必须始终保留现有 `/scan`、Nav2 costmap 和独立 safety gate。tracked object、预测节点或 TF
异常时，系统最多降级成“把障碍当静态物停车”，不能降级成忽略障碍继续行驶。

## 2. 当前能力与缺口

当前已经具备：

- `/scan` 更新 local/global costmap；动态障碍会表现为不断变化的占据区域。
- controller 对静态 costmap 做候选轨迹 footprint 碰撞检查。
- `forklift_safety` 对当前命令做短时扫掠 footprint 检查，并在障碍进入保护区时停车。
- 动态障碍停车、移除后放行的 Gazebo acceptance 已有。
- ORU-style planner 已有 `x/y/theta_index`、forward/reverse/pivot primitive 和 swept footprint 检查。

当前缺少：

- 视觉 tracked object 的正式 `.msg`、发布频率/超时/质量契约。
- 自车运动补偿后的目标速度和不确定性。
- 目标未来位置预测。
- 自车候选轨迹与目标预测轨迹的时间对齐碰撞检查。
- wait/release/replan 的行为状态机与防抖。
- 能表达到达时间的 timed trajectory；`nav_msgs/Path` 本身不能可靠承载完整时间语义。

因此当前系统能“看到后停”，不能可靠做到“预判横穿、提前减速、从目标身后通过”。

## 3. ORU / ILIAD 中能参考什么

### 3.1 ORU 可参考，但没有现成的单车动态目标预测器

| 本地参考 | 能借鉴的部分 | 不能直接解决的部分 |
|---|---|---|
| `navigation_oru-release/orunav_motion_planner` | state lattice、primitive、footprint collision | 世界查询是静态占据；没有 tracked object 预测，也没有 `time_index` 状态 |
| `orunav_msgs/Trajectory*`、`TimeEnvelope`、`RobotEnvelopes`、`CoordinatorTime` | 已知车辆轨迹的时间表达、时空冲突/预约思路 | 面向可协调机器人，不是对未知行人/叉车做感知预测 |
| `orunav_vehicle_execution` 激光 e-brake / slowdown 区 | 反应式近场减速和停车 | 只看当前激光点，不预测未来运动 |
| `orunav_coordinator_fake` | 接口形状和时间参数流 | 源码函数名就是 `computeCTSDirectlyWithoutDoingAnyCoordinationAtAll`，明确不做协调 |

真正的多车协调器不在当前 `navigation_oru-release` 主仓库中；README 也说明示例中的多车没有运行协调。
即使接入完整 coordination ORU，它解决的也是“多台受控机器人共享已知轨迹”的预约问题，不能替代
对行人、人工叉车等非受控目标的 tracking 和 prediction。

另外，本地 `navigation_oru-release/LICENSE` 默认是 CC BY-NC-SA 4.0。项目继续采用现有原则：
只参考算法和接口思想，商业代码走 clean-room 实现，不直接复制上游实现。

### 3.2 ILIAD 更接近人感知，但只建议参考

本地 `iliad/iliad_human_aware_navigation` 有 tracked persons、human-aware constraint costmap、
人车交互代价和 moving actor 仿真。它比 ORU motion planner 更接近“感知到人后改变导航行为”。

但这些模块主要是 ROS1、实验项目接口，其中一部分脚本已经标记 deprecated，且侧重社会导航代价，
不是本项目需要的通用时序碰撞预测器。可以参考以下思想：

- tracked person 转到固定坐标系再参与规划；
- 对人的位置、朝向和安全区生成额外代价；
- 使用 moving actor 场景做横穿/迎面/跟随测试；
- 将安全指标和任务成功率分开统计。

不建议直接把 ILIAD costmap 脚本接到真车主链路。

## 4. 目标架构

```text
视觉 / 多传感器融合
  ├─ /scan 或 PointCloud2 ──────────────> Nav2 costmap ──> safety gate（兜底）
  └─ /perception/tracked_objects
          |
          v
forklift_prediction
  - 时间戳和 TF 校验
  - 自车运动补偿
  - 静止/运动判定
  - CV/CA 预测 + 协方差传播
          |
          v
/perception/predicted_objects
  ├─> dynamic collision checker ──> controller 候选轨迹评分/拒绝
  ├─> dynamic behavior manager  ──> continue/slow/wait/replan/blocked
  └─> safety gate               ──> imminent collision 强制停车
```

职责边界：

- perception 负责观测、关联、稳定 ID 和当前运动状态。
- prediction 负责未来状态分布，不负责决定车辆怎么走。
- controller 负责短时、带时间的局部碰撞规避。
- behavior manager 负责等待、放行、重新规划和任务失败策略。
- global planner 先继续处理静态几何；是否增加时间维由后续场景决定。
- safety gate 永远拥有最终否决权，不依赖预测结果才能停车。

## 5. 分阶段实施

### D0：冻结接口和基线

交付：

- 把 `real_vehicle_tuning_guide.md` 中的 `TrackedObject.msg`、`TrackedObjectArray.msg`
  真正加入 `forklift_msgs`。
- 增加 `geometry_msgs`、`builtin_interfaces` 的 rosidl 依赖。
- 话题固定为 `/perception/tracked_objects`，推荐 `header.frame_id="odom"`。
- 写 bag/仿真回放工具，记录 `/scan`、tracked objects、`/odom`、TF、控制命令和 safety 状态。
- 固化当前 `dynamic_stop_release_ab`，作为以后每阶段必跑回归。

验收：

- 同一目标连续帧 ID 稳定；短时遮挡不频繁换 ID。
- stationary 目标经过自车运动补偿后速度接近 0。
- 时间戳、frame、尺寸、概率和 covariance 非法时有明确诊断，不静默使用。
- tracked object 链路完全关闭时，现有 `/scan` 停车/放行行为不变。

### D1：预测节点，先做可解释基线

新增 package：

```text
forklift_prediction
```

第一版算法：

- constant velocity（CV）作为主模型；低速且速度置信度差时按 stationary 模型处理。
- 预测 horizon、time step 参数化；初值只用于仿真，最终按真车速度和制动距离标定。
- 每一步传播 `pose` 和 covariance；目标包围盒按位置不确定性和类别安全余量膨胀。
- 预测概率随时间衰减。
- 输入超时、TF 失败、速度异常或尺寸非法时，不输出“看起来很准”的轨迹；发布 degraded/invalid 状态。

建议新增消息：

```text
PredictedState.msg
  builtin_interfaces/Duration time_from_start
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
  float32 probability

PredictedTrajectory.msg
  uint64 track_id
  uint8 classification
  float32 existence_probability
  geometry_msgs/Vector3 size
  forklift_msgs/PredictedState[] states

PredictedObjectArray.msg
  std_msgs/Header header
  forklift_msgs/PredictedTrajectory[] trajectories
```

后续有真实数据再评估 IMM、CTRV、地图约束或学习模型；第一版不要用黑盒预测替代可解释 CV 基线。

验收：

- 静止、匀速直行、横穿、迎面、远离五类合成轨迹有确定性单测。
- 不同消息延迟下，预测起点对齐到统一 evaluation time。
- covariance 随预测时间合理增长；观测恢复后能收敛。

### D2：独立动态碰撞检查库

新增与 ROS 解耦的几何核心，例如：

```text
forklift_dynamic_collision
```

输入：

- 自车带时间的 pose 序列和 footprint；
- 每个目标带时间的 pose、尺寸、协方差；
- 类别/速度相关安全余量。

输出：

- 是否碰撞；
- 首次碰撞时间 TTC；
- 最小预测间距；
- 目标 `track_id`；
- 风险等级和拒绝原因。

碰撞检查必须比较同一个未来时刻的自车与目标，不能把目标未来所有位置简单并成一个永久静态障碍。
几何上第一版可用“自车 polygon + 目标 oriented box + uncertainty inflation”；不要只比较中心点距离。

验收：

- 两条空间轨迹交叉但到达时间不同，不应误判必撞。
- 同时到达交叉点必须检出。
- 车尾、叉臂和 pivot 扫掠均使用完整 footprint。
- ID 切换不导致已有目标瞬间消失；允许短时保守延续旧预测。

### D3：先接 controller，再接 safety gate

`ForkliftMpcController::scoreCandidate()` 已按 `time_step` 推进候选自车状态，是第一接入点：

- 对每个候选控制生成的状态 `state(t)`，查询 D2 同时刻的目标预测。
- 预测硬碰撞的候选直接 invalid。
- 未碰撞但间距较小的候选增加 risk cost。
- 没有安全候选时输出停车/让上层进入 WAIT，不用异常触发危险 recovery。
- 保留原 costmap footprint collision，动态检查是附加条件而不是替代条件。

随后给 safety gate 增加独立的 imminent-risk 输入：

- TTC/距离进入硬阈值时强制减速或停车。
- prediction stale、TF 不可用或输入质量下降时，不允许放宽 `/scan` 保护区。
- safety 的硬阈值必须由真车制动测试得到，不能照搬仿真参数。

验收：

- 横穿障碍尚未进入当前 footprint 前，车辆能提前减速。
- 目标停下、变向或预测错误时，仍由 `/scan` 和 safety gate 防撞。
- prediction 节点崩溃、消息超时、TF 断开均无越过安全层的运动命令。

### D4：动态行为状态机（对应 P8.3）

建议状态：

```text
CLEAR -> SLOW -> WAIT -> REPLAN -> BLOCKED
```

策略：

- 短时横穿：优先减速/停车等待，不立即绕行。
- 风险解除：经过 release hysteresis 后恢复，避免每帧启停。
- 持续占路：达到 `replan_after` 才请求重新规划；同一目标设置 replan cooldown。
- costmap 有明确安全通道才简单绕行；否则保持 WAIT。
- 超过最大等待/重规划次数后进入 BLOCKED，由 task_manager 暂停或失败并给出原因。
- pivot/backoff 只作为安全闸允许的受控 recovery，不能因行人挡路自动朝人附近倒车。

实现位置可放在 Nav2 BT 自定义 condition/action 或独立 `dynamic_behavior_manager`；不要把等待计时、
重试次数和任务状态塞进 global planner。

验收场景：

- 横穿后离开：wait -> release，不 replan 抖动。
- 目标停在路径上：wait -> 一次 replan -> 绕行或 BLOCKED。
- 迎面目标：提前减速，不左右反复选路。
- 同向慢目标：保持距离，不贴近尾随。
- 目标短时遮挡：不瞬间加速穿过其最后位置。
- false positive / ID switch / sensor dropout：进入可解释降级，不失控。

### D5：全局重规划先保守投影，最后才扩展 ORU 时间维

第一版 replan 可把“短预测窗口内高概率占据区域”投影到临时动态 costmap layer，触发当前 ORU
planner 绕开。这个方法实现简单但偏保守，只用于持续占路和确有旁路的场景；短时横穿仍应 WAIT。

只有以下场景反复证明保守投影不够时，才做 `x/y/theta/time` lattice：

- 必须通过同一窄口，需要选择“先等后走”；
- 多个移动目标使静态并集完全堵死，但实际存在时间窗口；
- controller 的短 horizon 无法处理较长时序冲突。

届时 ORU core 需要的结构性修改：

- `State` 增加 `time_index` 或 arrival-time interval；状态 key 从 `x/y/theta/direction`
  扩展为 `x/y/theta/direction/time`。
- primitive 增加 duration 和每个 sample 的 `time_from_start`。
- `GridAdapter` 增加 `footprint_traversable(x, y, yaw, t)` / dynamic risk 查询。
- 增加 WAIT primitive，并惩罚无意义等待。
- `rejectReason()` 在 primitive 每个时序 sample 上检查 predicted object。
- `transitionCost()` 增加时间、风险、等待和舒适性代价。
- 空间 heuristic 继续作为忽略动态障碍的下界；必须重新检查 ARA*/A* 的一致性与内存上界。
- 输出不能只靠无时间语义的 `nav_msgs/Path`；需要 timed trajectory 或明确的 arrival-time metadata，
  controller 也必须遵守它，不能重新按任意速度执行。

这一阶段是算法级改造，不应与 D0–D4 混在一个 PR 中。

### D6：高级预测 / human-aware（非近期主线）

真实数据证明 CV 不够后，再按问题选择：

- IMM/CTRV：车辆转弯和启停；
- 地图/车道/过道约束预测：仓库车辆；
- 多模态预测：路口左右分叉；
- ILIAD 风格的人体朝向、交互区和社会代价：有人机共融需求时；
- 受控机器人之间的 trajectory envelope / reservation：多车调度时。

高级模型上线必须保留 CV fallback，并用 calibration、miss rate 和安全指标证明收益，不能只比较平均位置误差。

## 6. 参数与安全原则

- 所有预测阈值、TTC、保护距离最终由“最大实车速度 + 感知/通信延迟 + 制动响应 + 制动距离 + 余量”推导。
- 分类只改变舒适距离/行为策略，不能让低置信度类别缩小硬安全区。
- uncertainty 越大只能更保守，不能因为模型不确定而忽略目标。
- 预测消息 stale 时立即停止使用其“可通行”结论，但保留 `/scan` 当前占据和 safety 保护。
- 动态 costmap 必须有 clearing/TTL，避免残影；短时遮挡的 track memory 与永久残影要区分。
- 记录每次 SLOW/WAIT/REPLAN/BLOCKED 的触发目标、TTC、最小间距、输入 age 和决策原因。

## 7. 建议交付顺序

```text
D0 tracked msg 落地 + 数据回放
 -> D1 CV predictor
 -> D2 动态碰撞检查库
 -> D3 controller 时间碰撞 + safety imminent stop
 -> D4 wait/release/replan 状态机（完成 P8.3）
 -> D5a 动态 costmap 保守绕行
 -> 真车数据复盘
 -> D5b x/y/theta/time ORU（只有证据表明需要时）
 -> D6 human-aware / 多模态 / 多车预约
```

每阶段都必须回归：普通前进、倒车、pivot、静态障碍、动态停车/放行、急停、costmap/TF/预测超时。
