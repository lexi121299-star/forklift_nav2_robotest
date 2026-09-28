# 909 路径规划与控制问题离线分析

## 数据范围

- `pallet_debug_20260909_151339`
- `pallet_debug_20260909_151549`
- `pallet_debug_20260909_151637`
- 对应目录：`ros2bag/909/909log/909`

## 151339：前段跟随正常，随后冲出轨迹

首条路径长度约 10.57 m，共 227 个点，无 pivot。车辆起步后前约 1.5 m
跟踪正常，但横向误差随后持续增大：约从 0.02 m 增长到 0.20 m、0.54 m，
最终达到 0.756 m，触发原有 0.75 m path-deviation fail-safe。

误差已经增大时，控制器仍随着道路曲率减小把实际速度从约 0.11 m/s 提升到
0.40 m/s。转角反馈虽然跟随命令，但回正速度赶不上车辆沿轨迹前进的速度，
导致车辆继续向曲线外侧漂移。Nav2 随后重新规划了一条 5.94 m 路径，并最终
到达目标，因此重规划保护本身有效，不应简单放宽或删除。

修复：

- 增加 Frenet 横向误差速度调度。误差超过 0.10 m 开始连续降速，达到
  0.30 m 时速度上限为 0.15 m/s。
- 保留 0.75 m 最终偏离保护。
- 增加当前 Frenet 投影点参考转角的即时权重，避免预测窗口过早要求回轮。

## 151549：可执行的掉头路线被 controller 拒绝

连续 B-spline 因最小转弯半径约束失败后，局部 Reeds-Shepp/lattice fallback
成功生成 16.25 m 路径，其中包含一个经过 swept-footprint 校验的 pivot。
车辆没有运动，controller 在 `setPlan()` 中立即报错：

```text
ForkliftMpcController rejects legacy pivot path
```

后续两次重规划也重复相同结果，最终 Navigation failed。这不是底盘、定位或
Safety Gate 问题，而是 planner fallback 与 controller 配置互相矛盾。

修复：

- 将 `reject_pivot_paths` 改为 `false`。
- 仅允许 planner 已完成 footprint sweep 校验的 fallback 路径进入现有
  stop-pivot-go 状态机；pivot 的停车、预测制动、回正和低速恢复保护保持不变。

## 151637：起步转向摆动，随后无可行控制候选

首条路径约 5.89 m，无 pivot，但起始连续曲线曲率较大。controller 先将转角
预置到约 -0.898 rad，开始运动后又提前回到 -0.72/-0.51 rad，随后再次加深到
约 -0.65 rad，出现明显不连续。车辆每次只前进约 0.1~0.4 m，就报：

```text
ForkliftMpcController found no collision-free command
```

根因是当前 sampled MPC 用一个固定转角外推完整 1.8 s。连续曲线的转角参考会
随弧长变化，但候选碰撞检查假设该转角在整个预测窗口内不再变化，因此在狭窄
区域会把本来下一控制周期能够继续调整的命令提前判死。

修复：

- 0.60 s 内的预测碰撞继续作为硬拒绝。
- 0.60 s 以后的恒定转角外推碰撞改为最大障碍代价，由下一次 10 Hz 控制周期
  重新求解；当前和近期碰撞不会放行。
- 增加当前 Frenet 切点参考转角权重，避免预测窗口因看到远处曲率下降而过早
  回轮。
- 独立 Safety Gate、原始 scan 动态制动和 costmap 检查保持启用。

## 新增参数

```yaml
reject_pivot_paths: false
cross_track_slowdown_threshold_m: 0.10
cross_track_slowdown_full_error_m: 0.30
cross_track_recovery_max_speed_mps: 0.15
hard_collision_prediction_horizon_sec: 0.60
immediate_steering_reference_weight: 30.0
```

## 验证

- Foxy 干净构建：`forklift_oru_planner`、`forklift_nav2_plugins` 通过。
- ORU 与 Nav2 plugin：133 tests，0 errors，0 failures。
- 新增测试覆盖横向误差连续降速、远期恒定转角碰撞软代价、近期碰撞硬拒绝。

实车建议按以下顺序复测：

1. 复测 151637 附近短曲线，确认起步转角不再大幅来回切换。
2. 复测 151339 长曲线，观察 `MPC cross-track recovery slowdown`，确认误差
   超过 0.10 m 后不再继续加速。
3. 复测 151549 掉头路线，确认日志出现 pivot maneuver latched，而不是
   rejects legacy pivot path。

## 161324 / 161413：Frenet 跟踪从起始曲线向两侧发散

两个包分别是左、右方向的镜像故障：

- `161324` 的横向误差由 0 增长到 `+0.73 m`，航向误差达到约 `+37.6°`。
- `161413` 的横向误差由 0 增长到 `-0.81 m`，航向误差达到约 `-36.5°`。
- 控制器日志中的 preview 长度始终为 `0.700 m`。将
  `lookahead_distance` 改成 `2.0 m` 仍会被固定的 15 个、间距 0.05 m 的点截断。
- 两侧都表现为轨迹曲率已经迅速减小，但转向反馈仍保留较大角度。方向盘回正
  速度赶不上车辆沿曲线前进，车辆持续向弯道外侧偏离。

Frenet 投影使用轨迹切线左法向计算有符号横向误差，前进航向为切线，倒车航向
为切线加 pi；左右镜像结果和单元测试都表明符号定义正确。横向权重原先也存在，
只是参数名为 `path_distance_weight`，容易误认为是普通欧氏距离。

本轮修改：

- 新增明确参数 `lateral_error_weight: 24.0`，作为旧
  `path_distance_weight` 的规范别名。
- preview 改为按弧长动态选择：根据当前速度、加速度和预测时域计算，限制在
  `0.75~2.0 m`，另加 `0.30 m` 前视余量。
- `preview_window_points: 15` 仅保留为种子求解器的计算量上限；最终候选评分
  使用完整动态窗口。
- 当反馈转角和当前 Frenet 参考转角相差超过 `0.05 rad` 时开始降速，达到
  `0.20 rad` 时限速到 `0.10 m/s`，等待真实转向机构追上轨迹曲率。
- 启动日志增加完整 Q-like / R-like 权重、动态 preview 和转角跟踪参数。

对应参数：

```yaml
lookahead_distance: 2.0
preview_min_distance: 0.75
preview_distance_margin: 0.30
lateral_error_weight: 24.0
steering_reference_tracking_tolerance: 0.05
steering_reference_tracking_full_error: 0.20
steering_reference_tracking_max_speed: 0.10
```

Foxy 重建 `forklift_oru_planner` 与 `forklift_nav2_plugins` 后，共 256 个测试通过。
