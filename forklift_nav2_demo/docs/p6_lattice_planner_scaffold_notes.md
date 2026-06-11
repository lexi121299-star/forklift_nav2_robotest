# P6 Minimal Lattice Planner Scaffold Notes

## 1. 这一步要解决什么

P5 已经能把一条 `nav_msgs/Path` 处理成更适合 controller 跟踪的 trajectory：

```text
插密 -> 平滑 -> 曲率估计 -> 转角参考 -> 限速
```

但 P5 仍然是在“修一条已经给出来的路径”。如果 global planner 本身给出的路径包含车辆做不到的形状，例如：

```text
原地横移
贴障碍急拐
在很窄区域里突然变向
只检查路径点、不检查转弯扫掠区域
```

controller 再努力也只能补救一部分。

P6 的目标是让 global planner 一开始就生成更像叉车能执行的路径。

## 2. 什么是 lattice planner

当前 `OruGlobalPlanner` 是 costmap-aware A*：

```text
状态: x, y
动作: 到相邻 grid cell
碰撞: 检查 cell 和该 cell 姿态下的 footprint
输出: 一串 2D 路径点
```

lattice planner 的核心变化是把搜索空间从二维格子升级为：

```text
状态: x, y, theta_index
动作: motion primitive
```

其中：

- `x, y` 是 costmap cell。
- `theta_index` 是离散化后的车体朝向，例如 16 或 32 个方向。
- `motion primitive` 是一小段车辆能实际执行的运动，例如直行、左转圆弧、右转圆弧。

因此 planner 搜索的不是“从这个格子跳到旁边格子”，而是：

```text
从当前 pose 出发，尝试几段车辆可执行的小运动；
每段小运动沿途都不能碰撞；
把合法的小运动终点放进 A* open set。
```

这样输出的 path 天然带有车辆朝向和转弯约束，controller 不需要再把几何上不可行的折线硬掰成可行轨迹。

## 3. 什么是“全量 planner”

这里说的全量 planner，指接近 ORU motion planner 完整能力的一套系统，不只是一个小 A* 改造。

它通常会包含：

- 完整的 motion primitive 集合：
  - 前进直行
  - 前进左/右转
  - 后退直行
  - 后退左/右转
  - 原地或近似原地姿态调整
  - 不同曲率、不同长度、不同速度档位的 primitives
- heading 和位置的多分辨率搜索。
- 根据车辆几何、转弯半径、转向角速度生成或加载 primitive lookup table。
- 启发函数 lookup table，例如 Reeds-Shepp / Dubins / grid distance 混合启发。
- 倒车代价、换向代价、转向变化代价、贴障碍代价。
- primitive 沿途 footprint sweep collision。
- goal pose 对齐和最后一段 approach 处理。
- 路径后处理、压缩、平滑、速度/方向标注。
- 大量参数、缓存、debug marker、失败诊断。

这些能力很有价值，但一次性搬进来风险很高：

- 很难知道失败发生在 primitive、collision、heuristic、costmap 还是 controller。
- 参数很多，调试成本大。
- 一旦替换掉当前 planner，回退成本高。

所以 P6 第一刀要做的是“最小可回退 scaffold”，不是全量 planner。

## 4. 最小可回退 scaffold 是什么

最小 scaffold 的目标是先把 global planner 的搜索骨架改成 lattice 形状，但只打开最少动作。

建议第一版：

```text
保留现有 costmap-aware A* fallback
新增 x/y/theta_index 状态
只开 forward straight / forward left arc / forward right arc
primitive 沿途采样做 footprint collision
输出仍然是 nav_msgs/Path
```

这一步重点不是找到所有复杂场景的最优路径，而是证明：

```text
Nav2 costmap -> lattice search -> vehicle-feasible path -> P5/P4 controller
```

这条链路能闭环。

## 5. 每一步是什么意思

### 5.1 保留现有 costmap-aware A* 作为 fallback

当前 `OruGlobalPlanner` 已经能在 costmap 上找路径，它是一个能跑的基线。

保留 fallback 的意思是：

```text
新增参数 use_lattice_planner
false: 使用当前 2D A*
true: 尝试 lattice planner
```

或者更保守：

```text
先尝试 lattice
lattice 找不到路径时回退到当前 2D A*
```

这样做的价值：

- P6 开发过程中不会把已有 NavigateToPose 基线打断。
- 可以 A/B 对比同一 start/goal 下 2D A* 和 lattice 的路径差异。
- 如果 lattice 参数不成熟，仍能用旧 planner 做 smoke test。

### 5.2 加 `x/y/theta_index` heading state

当前 A* 只知道位置：

```text
cell(10, 20)
```

但叉车在同一个位置、不同朝向下，可执行动作完全不同：

```text
cell(10, 20), 朝东
cell(10, 20), 朝北
```

所以 P6 要把搜索状态改成：

```text
LatticeState:
  x
  y
  theta_index
```

`theta_index` 是离散朝向。例如 16 个 heading bin：

```text
0: 0 deg
1: 22.5 deg
2: 45 deg
...
```

这样 planner 才能表达：

```text
到达同一个 cell，但车头方向不同，是两个不同状态。
```

验收重点：

- 同一位置不同朝向可以有不同 parent。
- 输出 path 的 orientation 来自 lattice state，而不是简单用相邻点差分估计。

### 5.3 第一版 primitives 只开 forward straight / left arc / right arc

motion primitive 是车辆能执行的一小段动作。

第一版只开三种：

```text
forward straight
forward left arc
forward right arc
```

含义：

- `forward straight`：保持当前 heading 前进一小段。
- `forward left arc`：按固定曲率向左前方走一小段，heading 随之增加。
- `forward right arc`：按固定曲率向右前方走一小段，heading 随之减少。

暂时不开 reverse 的原因：

- 倒车会引入换向代价、倒车安全、局部 controller 方向控制等问题。
- 当前 controller/config 里 `max_reverse_velocity` 仍是 `0.0`。
- 先证明前进-only lattice 能生成更连续、可跟踪的路径，再加倒车更稳。

暂时不做大量曲率档位的原因：

- 第一版只需要证明 scaffold 正确。
- 曲率越多，branching factor 越大，搜索和调参都会变复杂。

验收重点：

- 生成路径不会出现相邻点朝向突变 90 度但中间没有可执行圆弧。
- 90 度转向应该表现为若干段前进圆弧，或在无法前进圆弧完成时失败/回退，而不是输出不可执行折线。

### 5.4 对 primitive 沿途采样做 footprint collision

普通 2D A* 容易只检查终点 cell：

```text
终点没撞 -> 认为这一步合法
```

但叉车转弯时车身和前叉会扫过一片区域。终点不撞，不代表中间过程不撞。

所以每个 primitive 要采样多个 pose：

```text
sample 0: 起点 pose
sample 1: primitive 25%
sample 2: primitive 50%
sample 3: primitive 75%
sample 4: 终点 pose
```

每个 sample 都调用 footprint collision checker：

```text
footprintCostAtPose(x, y, theta, footprint)
```

只有所有 sample 都安全，这个 primitive 才能加入 open set。

验收重点：

- 贴障碍转弯时，planner 不会只因为终点 cell 安全就通过。
- fork/车身扫掠区域能被 costmap footprint 检查拦住。

### 5.5 验收先看“不再出现 controller 无法执行的急转/横移”

P6 第一版不要求全局最优，也不要求解决所有仓储场景。

第一阶段验收更朴素：

```text
planner 输出的 path 是否明显更像车辆能走的轨迹？
controller 是否不再频繁收到几何上不可执行的折线？
```

具体看：

- path orientation 连续。
- 相邻 path 点不出现原地横移。
- 90 度附近不是一个尖角折线，而是带 heading 变化的连续运动。
- P5 日志里的 `sharp_turns` 数量减少，或 sharp turn 变成 lattice 可解释的连续 arc。
- FollowPath / NavigateToPose 不再因为 planner 给出过急几何而进入 controller 挣扎。

## 6. 推荐第一版参数

可以先从保守参数开始：

```text
use_lattice_planner: false
lattice_fallback_to_astar: true
lattice_heading_bins: 16
lattice_step_distance: 0.20
lattice_arc_radius: 0.60
lattice_arc_angle: 0.3926990817   # 22.5 deg
lattice_primitive_samples: 5
lattice_reverse_enabled: false
```

说明：

- `heading_bins=16` 和 `arc_angle=22.5 deg` 对齐，调试直观。
- `arc_radius=0.60` 先沿用 pivot/高曲率处理里的半径尺度。
- `step_distance=0.20` 不要太小，避免搜索爆炸；也不要太大，避免漏过窄通道。
- `primitive_samples=5` 第一版足够发现大多数扫掠碰撞。

这些不是最终参数，只是 scaffold 阶段的起点。

## 7. 推荐实现顺序

第一刀尽量小：

```text
P6.1 参数和数据结构
P6.2 2D A* fallback 显式保留
P6.3 LatticeState + indexing
P6.4 生成三种 forward primitives
P6.5 primitive sample collision
P6.6 reconstruct lattice path
P6.7 单测 + sparse NavigateToPose smoke
```

不要第一刀就做：

- reverse primitives。
- 复杂 lookup table。
- ORU primitive 文件加载。
- Reeds-Shepp 启发。
- 多分辨率 planner。
- 大规模参数调优。

## 8. 单测建议

建议先写纯 C++ 单测，不依赖完整 Nav2 bringup：

- heading bin normalize：
  - `theta=-pi`、`0`、`pi` 能映射到合法 bin。
- primitive endpoint：
  - straight primitive 终点在当前 heading 前方。
  - left arc 后 `theta_index` 增加。
  - right arc 后 `theta_index` 减少。
- fallback：
  - lattice 关闭时仍走当前 2D A*。
- collision：
  - primitive 中间 sample 碰撞时，该 primitive 被拒绝。
- path orientation：
  - 输出 path orientation 使用 lattice heading。

## 9. 完成标准

P6 scaffold 第一版完成时，应该能说清楚：

```text
这不是完整 ORU planner，
但 global planner 已经从 x/y grid search 迈到了 x/y/theta primitive search。
```

最低验收：

- 构建通过。
- `OruGlobalPlanner` 关闭 lattice 时行为保持原样。
- 打开 lattice 时能在简单空旷 start/goal 下生成 path。
- path orientation 连续。
- primitive 沿途 footprint collision 生效。
- 失败时能 fallback 或给出明确 planner error。

后续再逐步接近全量 planner：

```text
forward-only scaffold
-> reverse primitives
-> primitive cost tuning
-> better heuristic
-> ORU primitive/lookup table 参考实现
-> goal approach / docking / narrow aisle 场景
```

## 10. 2026-06-11 实现记录

本轮已完成最小可回退 lattice scaffold。

代码位置：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/oru_global_planner.hpp
forklift_nav2_plugins/src/oru_global_planner.cpp
forklift_nav2_plugins/test/test_oru_global_planner.cpp
forklift_nav2_demo/config/forklift_nav2_oru_test.yaml
forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml
```

已实现能力：

- `use_lattice_planner` 参数控制是否启用 lattice。
- `lattice_fallback_to_astar` 参数控制 lattice 失败时是否回退到原 2D A*。
- 搜索状态从 `x/y` 扩展到 `x/y/theta_index`。
- 第一版只生成 forward primitives：
  - straight
  - left arc
  - right arc
- 每个 primitive 按 `lattice_primitive_samples` 沿途采样。
- 每个 sample 同时检查 costmap cell 和 footprint collision。
- lattice 输出的 `nav_msgs/Path` 使用 lattice heading 生成 pose orientation。
- `goal_tolerance` 也用于 lattice 终点位置判定，避免 primitive 离散化必须精确命中某一个 cell。

ORU 测试配置已打开：

```yaml
use_lattice_planner: true
lattice_fallback_to_astar: true
lattice_heading_bins: 16
lattice_step_distance: 0.20
lattice_arc_radius: 0.60
lattice_arc_angle: 0.3926990817
lattice_primitive_samples: 5
lattice_reverse_enabled: false
```

保留的旧行为：

- lattice 关闭时仍走原 costmap-aware A*。
- lattice 找不到路径且 `lattice_fallback_to_astar=true` 时，仍回退到原 A*。
- reverse primitives 仍未实现；如果误开 `lattice_reverse_enabled`，planner 会打印 warning。

## 11. 验证记录

Foxy docker 构建：

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'export CCACHE_DIR=/tmp/ccache CCACHE_TEMPDIR=/tmp/ccache-tmp; ./scripts/foxy_colcon_build.sh'
```

结果：

```text
4 packages finished
```

Foxy docker 插件测试：

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'export CCACHE_DIR=/tmp/ccache CCACHE_TEMPDIR=/tmp/ccache-tmp; ./scripts/foxy_colcon_test.sh --packages-select forklift_nav2_plugins --ctest-args -R "test_oru_global_planner|test_forklift_vehicle_model|test_forklift_mpc_types|test_forklift_mpc_trajectory|test_forklift_mpc_preview_window|test_forklift_mpc_solver"'
```

结果：

```text
100% tests passed, 0 tests failed out of 6
Summary: 44 tests, 0 errors, 0 failures, 0 skipped
```

新增 planner 单测覆盖：

- heading index 正常归一化。
- forward straight / left arc / right arc 的终点和 heading 变化正确。
- primitive 中间 sample 碰到 lethal cell 时会被拒绝。

Foxy headless runtime smoke：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.3, y: 0.2, z: 0.0}, orientation: {z: 0.7071068, w: 0.7071068}}}, planner_id: GridBased}"
```

planner 配置日志：

```text
Configured GridBased ... use_lattice=true lattice_bins=16 lattice_step=0.20 lattice_arc_radius=0.60
```

planner 成功日志：

```text
Lattice planner produced 5 states
```

action 结果：

```text
Goal finished with status: SUCCEEDED
```

返回 path 共 5 个 pose，orientation 从起点朝向逐步过渡到 90 度：

```text
start yaw ~= 0 deg
intermediate yaw ~= 22.5 deg
intermediate yaw ~= 45 deg
intermediate yaw ~= 67.5 deg
goal yaw ~= 90 deg
```

这证明 P6 scaffold 已经实际由 Nav2 planner server 加载并产出 `x/y/theta_index` lattice path，而不是只停留在单元测试里。
