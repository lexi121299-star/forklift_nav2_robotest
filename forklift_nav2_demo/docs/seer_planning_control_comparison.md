# SEER SRC2000 Planning And Control Comparison
本文对比 SEER SRC2000 发布包中可观察到的规划/控制模块与当前 forklift 项目的规划/控制架构。

范围只覆盖规划和控制，不展开定位、建图、视觉模型、底盘驱动和系统服务。SEER 包主要是编译后的 `.so` 和模型文件，不是源码；因此本文将“已观察到的模块/符号”和“基于名称的工程推断”分开描述。

## 1. 观察依据

检查对象：

- `/home/pl/robotest/SRC-2000-3.4.8.6106-202606240000/Robokit-v3.4.8.6106-SRC2000.deb`
- 代表性库：
  - `usr/local/SeerRobotics/rbk/lib/libpath_plan.so`
  - `usr/local/SeerRobotics/rbk/lib/libmotion_plan.so`
  - `usr/local/SeerRobotics/rbk/lib/libdwa.so`
  - `usr/local/SeerRobotics/rbk/plugins/libMCLoc.so`
  - `usr/local/SeerRobotics/rbk/plugins/libSlaMapping.so`
  - `usr/local/SeerRobotics/rbk/plugins/libSensorFuser.so`

关键信息：

- `libpath_plan.so`、`libmotion_plan.so`、`libdwa.so` 等库带有符号表/调试信息，可以看到不少类名、函数名和源码文件名。
- `libMCLoc.so` 已 stripped，但动态符号和字符串中仍能看到定位/融合/地图相关线索。
- 未发现完整 C++ 源码，因此无法确认具体参数、状态机、代价函数细节和实际运行策略。

## 2. SEER 规划部分

从符号名看，SEER 规划侧是多算法组合架构。

| 观察到的名称 | 可确认含义 | 工程推断 |
| --- | --- | --- |
| `CQuadTreePathPlanner` / `QuadTree` | 四叉树地图/空间划分相关规划器 | 用四叉树加速障碍查询、碰撞检测或局部地图搜索 |
| `AStarAlgorithm` | A* 搜索 | 基础栅格/图搜索能力 |
| `RunDijkstra2D` | Dijkstra 2D 搜索 | 可能用于 2D fallback、距离场或无朝向路径搜索 |
| `HybridAStarAlgorithm` | Hybrid A* 搜索 | 考虑车辆朝向和运动学约束的搜索 |
| `rsGeneratePath` / `rsPathLength` | Reeds-Shepp 路径生成/长度计算 | 支持前进/倒车曲线，适合可倒车车辆 |
| `BSplineOptimizer` | B 样条优化器 | 对路径做平滑、避障或曲率优化 |
| `EdfMap` / `rebuildDistanceField` | 欧氏距离场 | 用于离障碍距离查询、路径优化和安全距离代价 |
| `isCollisionOBB2D` / `checkCollision` | OBB/footprint 碰撞检测 | 不只按点机器人处理，至少支持矩形/多边形碰撞 |
| `stg_virture_laser_point` | 虚拟激光点/射线相关 | 可能用于地图可见性、障碍检测或路径验证 |

规划侧结论：

- SEER 看起来同时具备经典 2D 搜索、运动学搜索、曲线路径连接、路径平滑和距离场优化。
- 它更像商业通用规划库，覆盖不同车型/场景的多种规划策略。
- 仅从符号名不能确定最终运行时一定启用了哪些算法，也不能判断它的重规划策略、fallback 顺序和调参水平。

## 3. SEER 控制部分

从 `libdwa.so` 和 `libmotion_plan.so` 看，SEER 控制侧不是单一 DWA，而是 DWA、LQR/MPC 类控制和碰撞/轨迹评分组合。

| 观察到的名称 | 可确认含义 | 工程推断 |
| --- | --- | --- |
| `DWAMotionControl` | DWA 动态窗口控制 | 采样速度、预测短时轨迹、按评分选择控制量 |
| `DWAMotionControlSteer` | 舵轮/转向版本 DWA | 可能适配转向角车辆，而不是只适配差速 |
| `DwaPlanner` | DWA 局部规划/控制器 | 有轨迹生成、路径评分、障碍评分、速度评分 |
| `scoreTrajPath` / `scoreTrajGoal` / `scoreTrajObstacle` | DWA 轨迹评分项 | 类似 Nav2 DWB critics，但实现是 SEER 自己的 |
| `LqrDiff` | 差速 LQR/MPC 类控制 | 面向差速底盘的预测/优化控制 |
| `LqrSteer` | 转向底盘 LQR/MPC 类控制 | 面向有转向角约束的车辆 |
| `LqrDualDiff` | 双差速控制 | 适配双差速/特殊底盘 |
| `solveMPC` | MPC 求解函数名 | 具备 MPC 风格控制求解，不一定是完整 QP-MPC |
| `move_1d_jerk` / `move_1d_step` | 一维速度/加加速度约束运动 | 有速度平滑、加速度/jerk 限制 |
| `Hashcollision` / `Costmap2D` | 碰撞检测和代价地图 | 控制器侧可能直接做局部避障/碰撞过滤 |

控制侧结论：

- SEER 不是只有 DWA；它看起来同时准备了 DWA、LQR/MPC、速度平滑和碰撞检测。
- DWA 部分更偏“采样控制”；LQR/MPC 部分更偏“模型预测/优化控制”。
- 名称显示它考虑了多种底盘：差速、转向、双差速等。

## 4. 我们当前规划部分

当前 forklift 项目的规划核心在 `forklift_nav2_plugins/OruGlobalPlanner`。

| 方向 | 当前实现 |
| --- | --- |
| 全局规划器 | `forklift_nav2_plugins/OruGlobalPlanner` |
| 搜索状态 | `x / y / theta_index` lattice |
| 运动原语 | forward / reverse / pivot primitive |
| 碰撞检测 | Nav2 costmap + swept footprint |
| 倒车语义 | 通过 primitive 和 path yaw 保留 reverse intent |
| pivot 语义 | 显式支持 rear-axle pivot / stop-pivot-go 链路 |
| fallback 策略 | 当前倾向禁用全向 A* fallback，避免生成叉车不可执行路径 |

我们的规划侧特点：

- 更窄，更针对当前叉车。
- 明确把“能不能被叉车执行”放在规划阶段约束。
- 特别关注倒车、后轴 pivot、路径朝向、运动原语和 safety gate 的一致性。
- 目前还不是完整 ORU 商业级 motion primitive planner，部分能力仍属于 scaffold。

## 5. 我们当前控制部分

当前 forklift 项目的控制核心在 `forklift_nav2_plugins/ForkliftMpcController`。

| 方向 | 当前实现 |
| --- | --- |
| 控制器 | `ForkliftMpcController` |
| 控制思想 | sampled predictive controller / MPC scaffold |
| 输入 | Nav2 `nav_msgs/Path`，预处理成 tracking trajectory / preview window |
| 状态 | 车辆位置、航向、转向角等 |
| 采样量 | 速度、转向角/转向变化 |
| 输出 | Nav2 `TwistStamped`，可选 `ForkliftControlCommand` |
| 倒车限制 | 只有 preview window 显示 reverse intent 时允许负速度 |
| pivot | controller 显式识别 pivot intent，执行 stop-pivot-go |
| 安全 | controller safety gate + 独立 `forklift_safety` command gate |

我们的控制侧特点：

- 它不是标准 Nav2 DWB/DWA。
- 它比传统 DWA 更理解叉车控制语义：转向角、倒车、pivot、限速、安全门。
- 当前求解方式偏“采样式预测控制”，还不是完整 QP-MPC。
- 控制逻辑源码可改、可测、可按真车继续收敛。

## 6. 核心差异

| 对比项 | SEER 推测架构 | 我们当前架构 |
| --- | --- | --- |
| 总体定位 | 商业通用机器人规划控制栈 | 当前叉车项目定制栈 |
| 算法覆盖 | A*、Dijkstra、Hybrid A*、Reeds-Shepp、B-spline、EDF、DWA、LQR/MPC | ORU lattice planner + sampled predictive controller |
| 车型适配 | 看起来支持多车型/多底盘 | 聚焦当前叉车 |
| 路径规划 | 多策略组合，覆盖较全 | 强调运动学可执行性和 primitive 语义 |
| 局部控制 | DWA + LQR/MPC 类控制并存 | 单一自定义预测控制器 |
| 倒车 | 有 Reeds-Shepp/LQR/DWA 线索，应该支持 | 显式用 reverse primitive/path yaw 控制倒车 |
| pivot | 从名字无法确认是否显式支持后轴 pivot | 明确支持 rear-axle pivot 和 stop-pivot-go |
| 路径平滑 | 有 B-spline 和 EDF 线索，可能更完整 | 当前只有轻量预处理和平滑 |
| 安全链路 | 黑盒内可能包含控制侧避障 | controller safety gate + 独立 command gate，链路清楚 |
| 可维护性 | `.so` 黑盒，难改策略 | 源码可控，易加测试和真车约束 |
| 成熟度 | 发布包形态，看起来更完整 | 仍在 ORU/真车适配推进中 |

## 7. 结论

SEER 的规划控制部分看起来更完整、更通用，像一套成熟商业底盘算法库：全局搜索、运动学路径、曲线连接、平滑优化、DWA、LQR/MPC 和碰撞检测都有覆盖。

我们当前方案更专门，目标不是覆盖所有机器人形态，而是把当前叉车在 Nav2/ORU 框架下稳定跑起来。我们的重点是：

- 规划路径必须符合叉车运动学；
- 倒车不能被普通路径误触发；
- pivot 要按后轴语义贯通 planner/controller/safety；
- 不把全向 A* 这类叉车跟不上的路径交给控制器；
- 控制命令必须经过 safety gate 统一约束。

如果追求“算法覆盖面”，SEER 更全。如果追求“当前叉车可解释、可修改、可验证、可上车调参”，我们当前架构更可控。

## 8. 后续可借鉴方向

从 SEER 名称看，后续可以优先借鉴这些方向，而不是直接照搬黑盒：

1. 路径平滑：补强 B-spline / distance field 类的轨迹平滑和离障优化。
2. 运动学连接：完善 Reeds-Shepp 或等价 analytic expansion，提高倒车/调头场景成功率。
3. 控制求解：把当前 sampled predictive controller 逐步升级到更严格的 QP/MPC 或 LQR-MPC 混合求解。
4. 多级 fallback：建立“运动学合法 fallback”，避免退回普通 2D A*。
5. 碰撞距离场：在 controller scoring 中更系统地使用距离场，而不是只做阈值式安全门。
