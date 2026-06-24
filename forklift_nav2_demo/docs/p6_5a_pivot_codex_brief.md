# P6.5a rear-axle pivot primitive — Codex 实施任务书

收尾 P6.5a：stop-pivot-go（绕后轴近原地 90° 转向）闭环。**这不是从零写**——planner 侧
`buildDirectPivotPath` / lattice pivot primitives / terminal pivot regime / pivot_segments 已实现，
config 里 `lattice_pivot_enabled: true`、`lattice_rear_axle_x_offset: -0.34` 已就位。你的任务是
对齐、补缺口、加验收、跑回归。

## 唯一真相来源（先读，按它做）
- 细则与验收门槛：`forklift_nav2_demo/docs/oru_migration_execution_plan.md`
  → 搜索 “P6.5a”：**“P6.5a 待做清单(收尾...)” 5 条** + “P6.5a 的目标/planner 侧/controller 侧/safety 侧/acceptance/终点姿态策略(v1)/不要求”。
- 权威配置（唯一）：`forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml`。
- 既有验收脚本风格照抄：`forklift_nav2_demo/scripts/forklift_ab_acceptance.py`、
  `forklift_p6_reverse_acceptance.py`、`forklift_ab_dynamic_obstacle_acceptance.py`。
- 相关代码：planner `forklift_nav2_plugins/src/oru_global_planner.cpp` +
  `forklift_oru_planner/vendor/src/oru_lattice_core.cpp`；controller
  `forklift_nav2_plugins/src/forklift_mpc_controller.cpp`；safety
  `forklift_safety/forklift_safety/safety_command_gate.py`（`predicted_poses_for_command` 已含 pivot 扫掠）。

## 待办（按 plan 的 5 条收尾清单）
1. **配置对齐**：确认 `lattice_pivot_enabled` / `lattice_rear_axle_x_offset` / `pivot_steering_angle`
   与 MPC controller、safety gate 三处一致，都指向驱动轴中心（`rear_axle_x_offset = -0.34`）。不一致就改到一致。
2. **controller stop-pivot-go**：确认 `forklift_mpc_controller` 能识别 pivot intent 段 →
   先 brake/降速到≈0 → 发 pivot command（`allow_pivot_turn=true`、`pivot_steering_angle≈90°`、低速 `velocity_mps` 定 yaw rate）
   → yaw 到位恢复前进。**缺则补**，不要把 pivot 当普通前进弧线追踪。
3. **safety gate pivot 扫掠**：确认/补齐 pivot 执行前与执行中的 swept-footprint 检查（车尾/配重旋转弧），
   障碍进入旋转包络即停。
4. **加验收脚本 + 跑**：新建 `forklift_nav2_demo/scripts/forklift_p6_5a_pivot_acceptance.py`，
   覆盖 plan acceptance 表全部场景及门槛：
   `pivot_90_left_in_place` / `pivot_90_right_in_place` / `pivot_90_then_forward_ab` /
   `pivot_blocked_stop` / `l_shaped_corridor_ab` / `sparse_90_turn_ab`（起点 (-2.0,-0.5,0°)→终点 (-1.3,0.2,90°)），
   并跑 `forward_ab / reverse_ab / dynamic_stop_release_ab` 回归不退化。
5. **不动**真车换算参数与 `pivot_turn_radius`（台架标定项）；全程 Foxy docker。

## 终点姿态策略（必须遵守）
- `use_final_approach_orientation=true` 保持，终点朝向是硬约束。
- 终点 90° 姿态**由终端 pivot 执行**，**不靠 reverse 弧**贴姿态；`lattice_reverse_requires_goal_behind=true` 保持。

## 不做（范围外，别扩）
完整 ORU primitive lookup/cache；高速连续圆弧最优平滑；窄通道三点掉头/倒车入库/复杂 docking；
把 `base_link` 移到后轴中心；`PlannerConstraints` 运行时通道。

## 验证（提交前自检）
- `bash scripts/foxy_colcon_build.sh` 构建通过；`bash scripts/foxy_colcon_test.sh` 全绿
  （含 `test_oru_lattice_core` / `test_oru_global_planner` 等 pivot 相关单测不退化）。
- 新验收脚本本地能跑出 plan 门槛（pivot_control_samples / reverse_control_samples / SUCCEEDED 等）。
- 不要实跑真车。

## 完成后
- 勾掉 §1 清单 `[ ] P6.5a` 与 §「P6 分版推进」`[ ] P6.5a`，在 plan 的 P6.5a 段追加
  “执行记录（日期 + 各 acceptance 场景实测结果 + 遗留项）”，风格对齐 P2.3 / P8.2 记录。
- 不清楚的设计点先在本文件追加 “待确认”，不要擅自扩大范围。
