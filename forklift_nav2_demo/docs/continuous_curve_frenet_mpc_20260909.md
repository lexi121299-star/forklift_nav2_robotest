# 连续曲线规划与 Frenet MPC 实施记录

日期：2026-09-09  
分支：`foxy-real`

## 已完成

### Frenet 轨迹

- `MpcTrajectoryPoint` 增加切线方向、车身参考方向、曲率导数、参考转角、速度上限和停车边界。
- 轨迹统一按 `0.05 m` 重采样。
- 投影输出弧长、带符号横向误差、航向误差、纵向误差、曲率和参考转角。
- 前进车身方向使用轨迹切线，倒车车身方向使用切线加 `pi`。
- 换向点和终点记录零速停车边界。

### Frenet MPC

- 主候选搜索和 sampled solver 均改为 Frenet 误差评分。
- 默认权重：横向 `12.0`、航向 `6.0`、纵向 `0.2`、参考转角 `3.0`、转角变化 `120.0`。
- 参考转角由轨迹曲率计算，控制器只叠加横向和航向修正。
- 直线速度调度使用 Frenet 切线航向误差，投影不可用时才回退到离散前视点角度。
- 保留转向反馈、转角及转角速率限制、碰撞检查、安全门和速度连续限制。
- 普通 Nav2 controller 默认拒绝旧式 pivot path。
- 新目标开始时不再强制方向盘回零；车辆保持停车并预转到当前轨迹曲率对应的参考转角，反馈进入 `0.08 rad` 容差并稳定 `0.2 s` 后才开始行驶。

### 连续曲线和可倒车兜底

- 2D A* 继续提供障碍物拓扑通道。
- 同向路径首选 clamped cubic B-spline，按完整 footprint、扫掠和 `0.60 m` 最小半径验证。footprint 失败时只在失败区域补原始 A* anchor；曲率失败不再全局加密 anchor。
- 普通导航优先连续曲线；连续曲线和 Reeds-Shepp 都失败时，允许经过完整旋转 footprint 验证的 segmented pivot 作为最终可达性兜底。
- B-spline 全部失败后，沿 A* 通道按 `1.25 m` 提取局部 anchor。
- 相邻或隔一个 anchor 尝试 Reeds-Shepp，并尝试切线、`+/-15 deg`、`+/-30 deg` 航向候选。
- 倒车代价系数为 `1.5`，换挡等效代价为 `2.0 m`，整条路径最多两次换挡。
- Reeds-Shepp 失败路段使用局部 lattice：先限制在 `8 x 8 m`，再扩大到 `12 x 12 m`。
- 局部 lattice 使用 `0.10 m` 搜索分辨率、16 个航向和 `0.20 m` primitive 步长。
- lattice 改为稀疏状态存储，不再按整张地图分配位置、航向和方向的稠密数组。
- 单次 lattice 限制 `30000` 次扩展和 `1.5 s`，整个局部修补限制 `5 s`。
- lattice 开启前进和倒车，关闭 pivot primitive，半径为 `0.60/0.90/1.20 m`。

### 托盘靠近

执行路线简化为：

```text
连续曲线导航到 staging
-> 单次 PivotRelative 对准托盘
-> 转向回正
-> 激活目标托盘豁免
-> 0.10 m/s 最终直线靠近
-> 叉尖距托盘 0.30 m 停车
```

- staging 只选择托盘法线上的候选，`lateral_m` 固定为零。
- 删除 staging runup、infeed pivot 和 infeed straight。
- 全局可达性检查目标由 runup 改为 staging。
- 托盘在 staging 导航和 pivot 阶段仍作为障碍物，只有最终直线靠近阶段启用已有豁免。
- `PivotRelative` 固定按请求方向旋转，最后 20 度降到 `0.06 m/s`。
- 达到目标后停车并等待 yaw rate 稳定；最终误差不超过 `0.05 rad` 才成功，超限直接失败，不再反向校准。

## 配置入口

- Nav2 与 MPC：`forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml`
- 托盘任务：`forklift_task_manager/config/pallet_approach.yaml`
- 精调动作：`forklift_vehicle_interface/config/fine_motion_adapter.yaml`

## 验证结果

Foxy Docker 构建通过：

- `forklift_msgs`
- `forklift_oru_planner`
- `forklift_nav2_plugins`
- `forklift_vehicle_interface`
- `forklift_task_manager`
- `forklift_nav2_demo`

自动测试结果：`281 tests, 0 errors, 0 failures, 0 skipped`。

### 2026-09-11 910 规划与持续接单修复

- 规划请求增加共享 `planner_total_timeout_sec: 10.0`，该预算覆盖 B-spline、Reeds-Shepp、segmented 和 departure 全流程。
- departure 只尝试远、中、近三个候选，不再按 `0.25 m` 对每个候选重复执行全图 A*。
- Reeds-Shepp 输出删除同一运动段内小于 `0.02 m` 的重复点；换向 cusp 仍保留，避免跨换向点计算虚假曲率。
- segmented fallback 对小于 pivot 门限的拐点先做直线 shortcut 和完整安全验证；能合并则不 pivot，不能合并才验证原地旋转。
- curve-exit 仅在同一路径已经经过高曲率参考后触发。新目标的初始舵角差不再误触发 `0.15 m/s` 恢复状态。
- curve-exit 恢复速度由横向、航向误差连续调度，初始上限 `0.20 m/s`；超过 `8 s` 只记录告警并继续低速 Frenet 修正，不让节点或任务退出。
- 当时新增的 `straight_cruise_speed_mps: 1.10` 已在 2026-09-14 方案中废弃为兼容参数；当前低曲率速度直接使用 `runtime_velocity_limit_mps`。
- Frenet 跟踪预瞄下限改为 `tracking_preview_min_distance_m: 3.80`；速度规划使用独立 `profile_preview_design_speed_mps: 1.90`，约对应 `4.91 m` 的前方制动预瞄。
- Task Manager 的 RViz 入口拆为 `/forklift/navigation_goal` 和 `/forklift/pallet_goal`，旧 `/goal_pose` 默认关闭。普通目标可替换普通目标，托盘任务期间拒绝其他目标。
- pallet staging 的 `ComputePathToPose` 单次超时为 `12 s`，候选选择总超时为 `20 s`；超时上报当前任务失败后，节点继续接收下一目标。
- real launch 增加 `use_task_manager:=true`。轻量 RViz 配置内的两个 Goal 工具分别发布普通导航和托盘目标，不再直接调用 Nav2 action。

### 2026-09-09 实车起步修正

- `13:18:04` 测试中，前 `1 m` 曲线切线由约 `17.5 deg` 增至 `51.1 deg`，起点参考转角约 `61 deg`；旧控制先回零，横向误差随后增长到约 `0.67 m`。
- `13:33:02` 测试中出现相反方向的同类现象，横向误差增长到约 `0.62 m`，排除单侧方向符号错误。
- 修正后新增起步预转测试，确认参考转角未到位时驱动速度保持为零。

## 实车验证顺序

1. 先以 `0.20 m/s` 测直线、90 度连续弯、S 弯和倒车。
2. 再以 `0.30 m/s` 重复测试，确认方向盘反馈和参考转角连续。
3. 最后恢复 `0.47 m/s`，统计横向误差 RMS 和 P95。
4. 分别测试前进转倒车、倒车转前进，确认换向点完全停车。
5. 托盘任务确认只有一次 staging pivot，且超调时失败而不是反向修正。
6. 狭窄通道确认完整 footprint 不碰 lethal cell，局部兜底在 5 秒内返回成功或明确失败。

本次未修改 MK320 CAN 报文、底盘速度换算或转向 CAN 编解码。

## 2026-09-14 分层 Anchor 与速度 MPC

- `planner_server` 同时注册 `GridBased` 和 `TopologyOnly`。前者生成可跟踪轨迹；后者只向 Task Manager 返回经过完整 footprint 搜索的 A* 拓扑通道。
- 普通目标先尝试完整连续轨迹；失败后 Task Manager 自动选择 `1.5-5.0 m` 间隔的安全 anchor，并在每个中间点到达后按最新 costmap 重算剩余路线。
- 单段最多重试两次并递归细分四级。任意相邻目标超过 `5.0 m` 且找不到安全停车 anchor 时拒绝执行，不以关闭 footprint 检查换取成功。
- 新普通目标会取消旧目标、拓扑请求和 anchor 队列；单次失败只结束当前任务，Task Manager 保持运行并继续接收目标。
- departure fallback 同时尝试前进和倒车。起点 footprint 已占用时，只接受占用集合单调减少且最终完全离开的低速逃离路径。
- `runtime_velocity_limit_mps` 是低曲率路段的直接速度参考，不再由 `straight_cruise_speed_mps` 二次限制。曲率、转向速率、弯前制动、终点停车和 Safety Gate 仍可降低速度。
- MPC 主 solver、采样 fallback 和碰撞预测统一使用 `acceleration + steering_rate` 控制模型，Frenet 代价包含横向、航向、速度、参考转角、转角速率和加速度。
- 选中托盘豁免框改为 `1.8 x 1.6 m`，只在 staging 后最终低速直线靠近阶段生效；staging 导航、pivot 及框外障碍仍严格检查。
- 新增 `/forklift/controller_debug` 与 `/forklift/navigation_anchor_index`，可记录分段序号、Frenet 误差、速度、转角、曲率、预瞄和限速原因。
- 新增 `plot_controller_debug`，用于从 rosbag 导出 CSV 并绘制误差、速度和转角曲线。

Foxy Docker 回归结果：Messages `9`、ORU `14`、Nav2 plugin `128`、Safety `29`、Vehicle Interface `53`、Task Manager `48`，合计 `281` 项全部通过。
