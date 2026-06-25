# Lattice 全局规划器改进 — codex brief

目标：让 `OruGlobalPlanner`（lattice 模式）在窄口 / 大转角 / 任意目标位姿下都能稳定找到**运动学可行**的路，
从而可以关掉 `lattice_fallback_to_astar`（v1 fail-safe）而不再频繁规划失败。

负责分工：codex 实现；人类做计划与最终验收；assistant 审查 + 在仿真里实测。

## 现状与文件

- 核心算法：`forklift_oru_planner/vendor/src/oru_lattice_core.cpp`（`LatticeCore`，hybrid-A* 风格）
  + 头文件 `.../include/forklift_oru_planner/oru_lattice_core.hpp`
- nav2 插件：`forklift_nav2_plugins/src/oru_global_planner.cpp`（`createPlan` / lattice→fallback→fail）
- 配置：`forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml` 的 `planner_server.GridBased`
- 单测：`forklift_oru_planner/test/test_oru_lattice_core.cpp`、`forklift_nav2_plugins/test/test_oru_global_planner.cpp`

## 根因（按影响排序）

1. **基元集只有单一转弯半径 0.6m**（直行 + ±22.5°弧 + ±22.5° pivot，前进/倒车）。曲率单一，
   窄口/大转角缺少可行基元。
2. **没有解析扩展（Reeds-Shepp shot）收尾**。`isGoal` 要求 位置≤`goal_tolerance` 且 朝向误差≤π/bins，
   靠粗基元精确命中终点位姿极脆 → 最常见的“no kinodynamic path”。
3. 朝向离散粗（16 bins=22.5°），窄走廊里 footprint 拒绝多、易死路。
4. 无路径平滑/优化，输出阶梯状、贴墙余量差。

## 交付物（按优先级）

### D1（必做，收益最大）Reeds-Shepp 解析扩展到目标
- 在 `LatticeCore::plan` 的搜索循环里，对被扩展的节点（或满足距离阈值的节点）尝试一次
  **Reeds-Shepp 曲线**直连目标位姿（最小转弯半径用与基元一致的 `arc_radius`）。
- 对解析曲线**逐采样点做 footprint + costmap 碰撞检查**（复用 `GridAdapter.footprint_traversable` /
  `cell_traversable`）；通过则把这段拼到路径上、视为到达，立即 `reconstruct` 返回。
- 频率可控：每 N 次扩展或当 `goalDistance < analytic_expansion_radius` 时尝试一次（加参数）。
- 倒车规则要尊重：RS 含倒车段时，遵守现有 `reverse_enabled` / `reverse_requires_goal_behind`。

### D2（必做）丰富运动基元目录（多曲率）
- 用**多个转弯半径**（如 {tight, 默认0.6, wide}，可配 `lattice_arc_radii: [..]`）和合适弧长，
  生成 lattice 对齐（end heading 落在 bin 上）的基元集合。
- 真正启用 `PrimitiveCatalog`：预生成 origin 处的基元集合，按 heading bin 旋转复用（性能 + 一致性），
  让 `plan()` 消费目录而不是每次硬生成那 5 个。
- 保留 pivot（绕后轴 `rear_axle_x_offset`）和倒车基元。

### D3（必做）终点连接 / 容差稳健性
- 复查 `isGoal` 与 `goal_tolerance` / 朝向容差的组合；与 D1 配合，避免“差一点点 bin”导致整体失败。
- `max_iterations` 给足预算（参数化，默认足够大）；搜索耗尽时输出 `SearchStats`
  （expanded/generated/rejected_footprint/best_goal_distance）到日志，便于诊断。

### D4（可选）路径平滑
- 对 lattice 路径做一遍**保碰撞约束的平滑**（梯度下降/shortcut），缓解阶梯、改善贴墙余量。
  必须每步做 footprint 碰撞检查，平滑后仍可行才采用。

## 约束（务必遵守）

- **不改 nav2 插件对外接口**（`createPlan` 签名、参数命名风格 `GridBased.lattice_*`）。
- **footprint 碰撞检查不能弱化**：所有新基元/解析段/平滑结果都要过 footprint+costmap 检查。
- **保留 `lattice_fallback_to_astar` 参数与 fail-safe 语义**（默认 false）；本任务目标是让 lattice
  自身足够强，而不是依赖 fallback。
- 尊重车辆运动学：`rear_axle_x_offset=-0.34`、pivot 绕后轴、倒车成本/规则不变。
- 新增参数都要在插件里 declare + 透传到 `PlannerOptions`，并在 yaml 注释默认值。

## 验收标准

- 之前实测失败的目标（仿真大图里如 (10.32,-2.93)、(8.60,-1.34)、(8.16,2.42) 这类一般空旷点，
  以及窄口/90°拐角点）在 **fallback 关闭** 下能规划成功且 footprint 不撞。
- `sparse_corner` 路线三段（含 (-0.3,-0.5) 拐点）全部可规划。
- 现有单测全过；新增针对 D1/D2 的单测：
  - RS 解析扩展在空地上能直连目标并通过碰撞检查；有障碍挡住时拒绝该段。
  - 多曲率基元目录生成正确（end heading 落在 bin、长度/成本正确、紧/松半径都在）。
  - 一个“窄通道 + 大转角”栅格用例：旧基元集失败、新实现成功。
- assistant 在仿真里复跑：fallback=false 下点几个点 + sparse_corner 能跑通、不贴墙撞角。

## 不做（明确排除）

- 不整套移植 ROS1 的 SBPL/orunav 代码栈；只把“多曲率基元 + 解析收尾 + 平滑”的思想做进现有 hybrid-A*。
- 不改控制器 / 安全门 / sim 桥 / 真车 Curtis 路径。

---

## 完成情况（codex 实现 / assistant 审查，2026-06-25）

代码审查 + 编译 + 单测均通过；两个包 7 个测试可执行 100% 通过。仿真实测进行中。

| 交付物 | 状态 | 实现要点与证据 |
|---|---|---|
| **D1 Reeds-Shepp 解析扩展** | ✅ 完成 | 新增 `vendor/src/reeds_shepp.cpp` + `vendor/include/.../reeds_shepp.hpp`（移植 OMPL 实现，附 `vendor/licenses/ompl_BSD_LICENSE`）。`plan()` 在 `goalDistance ≤ analytic_expansion_radius`(3.0) 或每 `analytic_expansion_interval`(20) 次扩展时尝试 RS 直连；`analyticExpansion()` 对每个 RS 段调 `rejectReason`（footprint+costmap+越界），任一采样碰撞即整段放弃；含倒车段时尊重 `reverse_enabled` / `reverseAllowedTowardGoal`。新增 stats：`analytic_attempted/succeeded/rejected`。单测：`ReedsSheppAnalyticExpansionConnectsExactGoalPose`、`...RejectsBlockedSweptPath`、`AnalyticExpansionUsesForwardOnlyPathWhenReverseIsDisabled`、`ReedsSheppExpansionReachesArbitraryGoalPoses`。 |
| **D2 多曲率基元目录** | ✅ 完成 | `PlannerOptions.arc_radii`（配置 `[0.45, 0.60, 0.90]`）；`buildPrimitiveCatalog` 按多半径生成 lattice 对齐基元，`LatticeCore` 构造时预生成 `primitive_catalog_`，`generatePrimitives` 按 heading bin 旋转复用目录（不再硬生成）；`sanitize()` 对空 `arc_radii` 兜底（{r, 1.5r, 2r} 并并入 `arc_radius`、排序去重）。保留 pivot（绕后轴）与倒车基元。单测：`MultiCurvatureCatalogIsHeadingAlignedAndCostedByRadius`、`TightRadiusCatalogSolvesConstrainedNinetyDegreeTurn`、`CatalogLoadProvidesReversePivotAndForwardHeadings`。 |
| **D3 终点连接 / 容差 / 预算** | ✅ 完成 | `max_iterations` 默认 0→250000；新增 `goal_heading_tolerance` 参数；analytic 诊断计数随搜索统计一起可日志化。 |
| **D4 路径平滑（可选）** | ✅ 完成 | `smoothPath` 用 `analyticExpansion` 做 collision-checked shortcut（复用 D1 的碰撞检查），且 shortcut 长度不超过原长 1.05 倍、段数必须更少才采用；代码默认 `shortcut_smoothing_enabled=false`，committed 配置里打开为 true。单测：`CollisionCheckedReedsSheppShortcutReducesSegments`、`ReedsSheppShortcutKeepsOriginalWhenObstacleBlocksIt`。 |

**约束符合性**：nav2 插件 `searchLattice` 确实委托 `LatticeCore::plan`（新能力真正生效，非死代码）；所有新参数在插件 declare 并透传到 `PlannerOptions`；footprint 碰撞检查在新基元/RS 段/平滑结果上均强制；`lattice_fallback_to_astar` 默认 false 的 fail-safe 语义保留；`rear_axle_x_offset`、倒车规则、控制器/安全门/sim 桥/真车路径均未改。

**待办**：assistant 在仿真里保持 `lattice_fallback_to_astar: false`，复跑之前失败的点 + `sparse_corner`，确认能规划通且不贴墙撞角。
