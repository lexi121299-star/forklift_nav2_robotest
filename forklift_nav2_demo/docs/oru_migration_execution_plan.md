# ORU Migration Execution Plan

这份文档是后续把 `navigation_oru-release` 逐步移植到当前 ROS2 forklift 仿真，并最终能上真车运行的执行清单。

目标不是一次性把 ORU 全套搬过来，而是按最小可验证闭环推进：

```text
地图/任务 -> 路径 -> 轨迹 -> 控制 -> 底盘协议 -> 车辆反馈 -> 定位/TF
```

每一步都必须能单独测试、能回退、能解释问题发生在哪一层。

## 0. 当前基线

当前已经具备：

- `forklift_nav2_demo`
  - Gazebo world、forklift URDF、Nav2 bringup、RViz。
  - 清理后的地图：`/home/pl/robotest/forklift_factory_big_map_clean.yaml`。
  - 手画路径工具：`forklift_manual_path_follower`，通过 `/follow_path` 让 controller 跟踪手画 path。

- `forklift_nav2_plugins`
  - `forklift_nav2_plugins/OruGlobalPlanner`
    - 当前已经完成 P6.4a 最小倒车执行验证：`x/y/theta_index`、forward/reverse primitives、primitive 方向元数据、沿途 footprint collision、2D A* fallback、lattice cost/diagnostics，以及 reverse path 到 controller/bridge 的低速执行链路。
    - 还不是完整 ORU lattice/motion primitive planner；倒车场景调优、lookup/cache 和场景化 acceptance 仍待做。
  - `forklift_nav2_plugins/ForkliftMpcController`
    - 当前是 sampled predictive controller scaffold，不是完整 ORU QP-MPC。

当前主要问题：

- 仿真时间、`/clock`、TF、`/odom` 必须稳定，否则 2D Pose Estimate、costmap、FollowPath 都会失败。
- 当前 controller 已经有 ORU State / Control 概念、trajectory preview、最小 MPC 求解、path preprocessing 和 rear-axle pivot-turn 预测，但还不是完整 ORU QP-MPC。
- 当前 global planner 已经开始理解 heading、forward/reverse motion primitives、前进/倒车方向语义和换向代价；P6.4a 已在 runtime test 配置中低速打开 reverse，并通过 controller preview gating 避免普通 forward path 被倒车候选扰乱。

## 1. 总体迁移顺序

按这个顺序做，不跳步：

```text
P0  稳定仿真基线和 TF
P1  建 forklift_msgs
P2  建 forklift_vehicle_interface
P3  定义真实车控制模型和命令接口
P4  把 ORU MPC 核心移进 ForkliftMpcController
P5  接入轨迹处理和平滑
P6  把 ORU motion planner / primitive 思路移进 GlobalPlanner
P8  加 safety，做急停、限速、防撞、掉边保护
P7  加 task_manager，统一 A-B 导航、手画 path、调度任务
P9  真车低速联调
```

编号仍保留 P7/P8，但 2026-06-12 起近期执行优先级调整为：

```text
[x] P6.3  reverse primitives + direction metadata
[x] P6.4a 最小倒车执行验证：controller/vehicle_interface 能执行倒车段
[x] P8.1  最小 safety gate：动态障碍停车/限速、急停、watchdog
[x] A-B acceptance 快速验证：空旷 NavigateToPose 简单 A-B
[x] A-B 动态障碍停车/放行快速验证
[x] P6.4b 倒车 acceptance 调优：当前最小 lattice scaffold 范围内，普通前进不乱倒、后方目标能倒车、90 度和动态障碍回归不退化
[x] A-B acceptance 正式验收：普通路线、倒车/换向路线、障碍停车/放行
[ ] P6.5a 上车前 rear-axle pivot primitive：停下后绕后轴近原地 90 度转向，再继续前进
[ ] P8.2  独立 safety package / 命令闸门
[ ] P8.3  动态障碍等待、重新规划、简单绕行
[ ] P8.4  真车低速 safety acceptance 包
[ ] P2.3  真车 vehicle_interface 实际 I/O：底盘控制、反馈、急停、watchdog（代码侧已补齐，待台架/真车验收）
[ ] P7.1  task_manager 最小任务入口
```

调整原因：

- 当前目标是先让车能稳定从简单 A 到 B，并且遇到动态障碍物能安全停下。
- 倒车是叉车运动能力的一部分，应该先在 planner/controller 闭环里打通。
- 第一版真车点位有大量“原地/近原地 90 度后再走”的动作；真车已确认是双驱差速，90 度绕后轴旋转，所以进入独立真车 safety gate 前需要先补最小 rear-axle pivot primitive，而不是回到 Ackermann 或靠普通 arc 硬凑。
- 动态障碍停车/限速是上车安全底线，应在最小倒车执行闭环后尽早完成，不等所有倒车场景调优结束。
- `forklift_task_manager` 负责站点、路线、任务暂停/恢复/取消和调度接口，可以等 A-B + safety gate 稳定后再做。
- `iliad/` 里的 human-aware navigation、HRSI、安全指标和 actor 仿真不属于当前 A-B + safety 主线，暂不作为 ORU 核心移植前置条件。

原则：

- 不直接改 `/opt/ros/humble` 的 Nav2 源码。
- 不把底盘协议写进 planner/controller。
- 不把调度任务写进 controller。
- 所有算法通过 Nav2 plugin 或独立 node 接入。
- 每一步都保留一个能回退的配置文件。
- **构建 / 测试 / 运行一律在 Foxy docker（`forklift-nav2:foxy`）里进行**，不在宿主机原生环境编译或跑验收：`colcon build --build-base build_foxy --install-base install_foxy --symlink-install`，跑 `./scripts/foxy_colcon_test.sh`。
- **配置以 `forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml`（Foxy 版）为唯一权威**，现在是 `config/` 下唯一的 Nav2 参数档，`forklift_navigation.launch.py` 默认值也指向它。原 Humble/原生并行档（`forklift_nav2_oru_test.yaml`、stock 基线 `forklift_nav2.yaml`，含 `goal_checker_plugins` 复数、`DifferentialMotionModel` 等 Foxy 加载不了的 API）已删除，避免 Foxy/Humble 混淆；历史阶段笔记中残留的文件名只作历史记录。

## 2. P0: 稳定仿真基线

目标：

- Gazebo、Nav2、RViz、AMCL、TF、costmap 稳定。
- 能用 `2D Pose Estimate` 定位。
- 能用 `NavigateToPose` 或 `/follow_path` 触发 controller。

当前要检查：

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

验收标准：

- `/clock` 有数据。
- `/odom` 有数据。
- `odom -> base_link` 连续可查。
- `map -> odom` 在 2D Pose Estimate 后可查。
- `/controller_server` 和 `/planner_server` 是 `active`。
- RViz 里能显示 `/map`、global costmap、local costmap、robot footprint。

不要进入 P1/P2 的条件：

- `odom frame does not exist` 还在出现。
- `2D Pose Estimate` 点了之后 AMCL 没有响应。
- Gazebo 车没有 spawn 或 `/odom` 不发布。

## 3. P1: 建 forklift_msgs

目标：

定义真车和上层任务需要的消息，不直接复用 ORU 消息作为对外接口。

新 package：

```text
forklift_msgs
```

建议消息：

```text
msg/ForkliftControlCommand.msg
msg/ForkliftVehicleState.msg
msg/ForkliftFaultState.msg
msg/ForkliftTask.msg
msg/ForkliftTaskState.msg
srv/SetControlMode.srv
srv/SetEmergencyStop.srv
```

第一版 `ForkliftControlCommand` 建议包含：

```text
std_msgs/Header header
float64 velocity
float64 steering_angle
float64 steering_angle_velocity
float64 fork_height
bool enable
```

第一版 `ForkliftVehicleState` 建议包含：

```text
std_msgs/Header header
float64 velocity
float64 steering_angle
float64 battery_percent
bool enabled
bool emergency_stopped
string mode
```

ORU 参考：

- `orunav_msgs`
  - 只作为字段设计参考，不作为最终对外协议。

验收标准：

- `colcon build --packages-select forklift_msgs` 通过。
- `ros2 interface show forklift_msgs/msg/ForkliftControlCommand` 正常。

## 4. P2: 建 forklift_vehicle_interface

目标：

把 Nav2/controller 输出转换成真实底盘协议，同时把底盘反馈转换成 ROS 标准 `/odom` 和 TF。

新 package：

```text
forklift_vehicle_interface
```

职责：

```text
订阅:
  /cmd_vel
  /forklift/control_cmd
  /forklift/emergency_stop

发布:
  /odom
  odom -> base_link TF
  /forklift/vehicle_state
  /forklift/fault_state
```

第一阶段先做仿真版接口：

```text
/forklift/control_cmd -> /cmd_vel
/odom passthrough 或 fake odom monitor
```

第二阶段接真实协议：

```text
/forklift/control_cmd -> CAN/串口/TCP -> 底盘控制器
底盘反馈 -> /odom + /forklift/vehicle_state
```

ORU 参考：

- `orunav_mpc/src/commandSender.*`
- `orunav_mpc/src/canlibWrapper.*`
- `orunav_mpc/src/canSensorReader.*`

不要直接照搬：

- ORU 的 CAN 代码可以参考，但不要把它塞进 Nav2 controller。
- 真车协议必须独立在 `forklift_vehicle_interface`，这样后面换 controller 不影响底盘通信。

验收标准：

- 不启动 Nav2，只用测试命令也能驱动车：

```bash
ros2 topic pub /forklift/control_cmd forklift_msgs/msg/ForkliftControlCommand ...
```

- 真车或仿真能发布稳定 `/odom`。
- `tf2_echo odom base_link` 连续可查。
- 命令超时后车辆自动停车。

P2.3 执行记录（2026-06-17）：

- `curtis_vehicle_interface` 已从 dry-run 日志骨架补成真实 I/O 节点；默认仍可 `dry_run:=true`
  离线验证，`dry_run:=false can_interface:=can0` 时打开 SocketCAN。
- 新增标准库 SocketCAN 传输层，周期发送 Curtis 控制帧 `0x203`、`0x303`、`0x403`。
- 接收并解析 Curtis 反馈帧 `0x183`、`0x283`、`0x383`、`0x483`，发布 `/forklift/vehicle_state`、
  `/forklift/fault_state`、`/forklift/io_state`。
- 根据左右驱动 RPM、`drive_wheel_radius_m`、`drive_gear_ratio`、`drive_track_width_m`
  积分发布 `/odom` 和 `odom -> base_link` TF。
- `/forklift/set_emergency_stop` 会立即发 brake/zero 命令；`command_timeout_sec` 超时、
  `manual/standby` 模式、非法前后退方向都会进入 brake/zero 输出。
- 新增 `curtis_vehicle_interface.launch.py`，暴露 `dry_run`、`can_interface`、watchdog、
  feedback timeout、odom/TF frame、轮径/齿比/轮距等真车参数。
- 验证：Foxy docker `colcon test --packages-select forklift_vehicle_interface` 通过，14 个 pytest
  全部通过；`colcon build --packages-select forklift_vehicle_interface --symlink-install` 通过；
  `ros2 launch forklift_vehicle_interface curtis_vehicle_interface.launch.py --show-args` 可发现启动参数。
- 尚未完成台架/真车实际验收：需要接真实 `can0`，确认 CAN ID/字节定义与实车一致，标定轮径、
  齿比和驱动轮距，验证 `/odom` 与 `tf2_echo odom base_link` 连续稳定，并实测急停/watchdog
  能让底盘停车。

### P2.3a velocity → drive_rpm 换算（双驱差速外侧轮语义）

底盘确认：**两个驱动轮在前轴（双电驱差速），后面是单个万向轮**。叉臂在前。

协议依据（MK320 CANopen「注意事项 5」）：

> 双驱车型自动模式下，上位机下发的指令速度在转弯时，**弯心外侧轮速度与下发指令一致，内侧轮速由 CURTIS 控制器算**。

即 `0x203` 的 `行驶速度`（BYTE1-2，0–4000 rpm）= **转弯外侧驱动轮的电机转速**，配合 `转向角度`（BYTE6-7）和前进/后退位；没有独立的“原地自转/角速度”指令。要让车转，唯一动力来源是驱动轮转动，所以 **pivot 也必须发非零 `drive_rpm`，`drive_rpm=0` 则车完全不动**。

当前缺口（必须修，否则真车一步都走不了）：

- `ForkliftMpcController::publishControlCommand` 只填 `velocity_mps` + `steering_angle`，`drive_rpm` 留默认 0；plugins 全链路 `grep drive_rpm` 零命中。
- safety gate 只对 `drive_rpm` 做 clamp 后透传（恒为 0 → clamp 无效）。
- `encode_0x203` **只读 `drive_rpm`、完全忽略 `velocity_mps`**，直接写进行驶速度字节。
- 结果：仿真靠 `velocity_mps`→`/cmd_vel` 能动；真车 `0x203` 行驶速度恒为 0，**车不走也不转**。

pivot 旋转中心已核对（URDF 实锤，MPC 不用改）：

- `forklift_diff_drive.urdf.xacro`：左右驱动轮 joint 在 `x=-0.34`（`±wheel_separation/2=±0.38`），`front_caster` 在 `x=+0.42`；建模 `+x` 朝万向轮一侧。
- 配置 `rear_axle_x_offset = -0.34` ⇒ **MPC pivot 点精确落在两驱动轮的差速轴中心**（命名 “rear_axle” 只是标签，几何上就是驱动轴）。差速驱动物理上唯一能实现的旋转中心就在两轮连线上，所以 MPC 绕这个点 pivot 是对的，**P2.3 不动 MPC pivot 几何**。
- 因此差速分解就是绕这个 `-0.34` 驱动轴：pivot 时 `v_x=0`、外侧轮 `=|ω|·T/2`，与 odom 自洽。`velocity` 参考点是 base_link(0,0)，在驱动轴前方 0.34m，转弯时有小的二阶偏差，台架标定即可。
- 注意 sim/real 前后布局相反：sim URDF 万向轮在前(+0.42)、驱动轮在后(-0.34)；真车是驱动轮在前、万向轮在后。这只影响 footprint 扫掠方向与标定，不影响差速换算本身——列为真车 bring-up 标定项，不在 P2.3 改。

换算公式（落在 `curtis_vehicle_interface`，gate 下游，用已限好的 `velocity_mps`）：

```text
# 把 (velocity_mps, steering_angle) 还原成车体 twist（与 ForkliftVehicleModel 一致）
if |steering| >= pivot_steering_angle - 1e-3 and speed > 0:   # pivot 分支
    v_x   = 0
    omega = speed / pivot_turn_radius
else:                                                          # 自行车模型
    v_x   = speed
    omega = speed * tan(steering) / wheel_base

# 差速分解（绕 x=-0.34 的驱动轴中心，即 MPC rear_axle_x_offset）
v_left  = v_x - omega * track/2
v_right = v_x + omega * track/2
v_outer = max(|v_left|, |v_right|) = |v_x| + |omega|*track/2   # 外侧轮线速度

# 线速度 -> 电机转速（与 odom 的 _rpm_to_mps 严格互逆）
wheel_rpm = v_outer * 60 / (2*pi*wheel_radius)
drive_rpm = clamp(wheel_rpm * gear_ratio, 0, max_drive_rpm)    # 方向由 forward/reverse 位给
```

落点与参数：

- 纯函数 `drive_rpm_from_command(...)` 放 `curtis_command_kinematics.py`，单测覆盖；`curtis_vehicle_interface` 在 `encode_0x203` 前用它**覆盖** `drive_rpm`（上游 `drive_rpm` 不可信，恒 0）。
- 因为用的是 gate 已限好的 `velocity_mps`，gate 限速不会被绕过；再用 `max_drive_rpm`（默认对齐 gate 的 2500）兜底，确保不超 gate 的 rpm 包络。
- 新增接口参数：`drive_wheel_base_m`(1.2)、`pivot_steering_angle_rad`(π/2)、`pivot_turn_radius_m`(0.6)、`max_drive_rpm`(2500)；必须与 MPC/gate 同名参数保持一致。

验收：

- [ ] 单测：直行 `v=0.2, δ=0` → `drive_rpm = 0.2*60/(2π*0.1) ≈ 19.1`；pivot `v, δ=90°` → `drive_rpm ≈ |omega|*track/2` 对应的非零 rpm；`v=0` → `drive_rpm=0`。
- [ ] 台架：`dry_run` 日志里转弯时 `0x203` 行驶速度非零且随转向变化；pivot 命令下行驶速度非零。
- [ ] 与 odom 自洽：发已知 `drive_rpm` 反馈，`/odom` 速度与下发线速度量级一致。

P2.3a 执行记录（2026-06-18）：公式与 `curtis_vehicle_state._rpm_to_mps` 严格互逆，pivot 按绕 `-0.34` 驱动轴的差速分解，已加单测；经 URDF 核对 MPC pivot 点本就在驱动轴中心，无需改 MPC。

- 落地文件：新增 `curtis_command_kinematics.py`（纯函数 `drive_rpm_from_command`）+ `test/test_curtis_command_kinematics.py`（6 例）；`curtis_vehicle_interface` 在 `encode_0x203` 前覆盖 `drive_rpm`，新增 `drive_wheel_base_m`/`pivot_steering_angle_rad`/`pivot_turn_radius_m`/`max_drive_rpm` 参数并在 launch 暴露。
- 验证：Foxy docker（py3.8 / pytest-4.6.9）`colcon build` + `colcon test --packages-select forklift_vehicle_interface` 通过，20 测试全过（含新增 6 例）；并把 `forklift_vehicle_interface` 加进 `scripts/foxy_colcon_test.sh` 的 `--packages-select`。
- 遗留真车标定项：sim/real 前后布局相反（footprint 扫掠方向）、`drive_track_width_m`(real 0.70 vs sim 0.76)、`drive_wheel_radius_m`(real 0.10 vs sim 0.16) 需按实车标定。

P2.3b 整车参数标定（2026-06-18，厂家口述 + 规格图纸 `2MKC20M30LV205 平衡重式叉车20250122.PDF`）：

- 拿到真值,占位默认全部替换并通过单测:
  - `drive_gear_ratio = 26.75`（采埃孚减速比,主要部件规格表）
  - `drive_wheel_radius_m = 0.2285`（驱动轮 φ457×178,空载名义,带载滚动半径仍需上车实测微调）
  - `drive_track_width_m = 0.937`（图纸「轮距,驱动侧」937mm,原占位 0.70/0.76 偏小）
  - `drive_wheel_base_m = 1.4`（图纸「轴距」1400mm,原占位 1.2）
  - `max_drive_rpm = 2485`（厂家给运行软限速 8 km/h ⇒ 2.222 m/s × 1118 rpm/(m/s)）
  - `min_drive_rpm = 100`（厂家:100rpm 车能稳定转,低于此电机转速会波动;`(0,floor)→floor`,0 仍为 0）
- 交叉验证:0x203 字段满量程 4000 rpm 用 gear 26.75 + 轮径 0.2285 算 ⇒ 12.9 km/h,与铭牌「行驶速度 满/空载 12/13 km/h」吻合,反证 gear/轮径正确。8 km/h 是 AGV 运行软限速(2485 rpm),12–13 km/h 是硬件极限(≈4000 rpm)。
- 换算系数 k = 60·gear/(2π·r) ≈ 1118 rpm/(m/s);command 与 odom `_rpm_to_mps` 仍严格互逆。
- 落地:`curtis_command_kinematics.drive_rpm_from_command` 新增 `min_drive_rpm` 抬升逻辑 + 默认值改真值;`CurtisFeedbackState` 默认值;`curtis_vehicle_interface` 节点+launch 新增 `min_drive_rpm` 参数;`safety_command_gate` 节点+launch `max_drive_rpm 2485`、`wheel_base 1.4`。
- 验证:Foxy docker `colcon build` + `colcon test` 通过,forklift_vehicle_interface 24 例(新增 4 例:8km/h↔2485、min_rpm 抬升/保零/不扰动高值)、forklift_safety 15 例全过。
- 仍遗留真车标定项:带载滚动半径(图纸 φ457 是空载名义)、`min_drive_rpm` 上车下探微调、sim URDF 几何与真车不一致(见下「URDF 建模 vs 真车」)。

### URDF 建模 vs 真车（2026-06-18 核对）

sim URDF `forklift_nav2_demo/urdf/forklift_diff_drive.urdf.xacro` 是粗略替身,几何与真车 2MKC20M30LV205 不一致:

| 量 | sim URDF | 真车规格 |
| --- | --- | --- |
| 驱动轮半径 | 0.16 | 0.2285（φ457） |
| 驱动轮距 `wheel_separation` | 0.76 | 0.937 |
| 轴距（驱动轴↔front_caster） | 0.76（-0.34↔+0.42） | 1.4（轴距） |
| 车体宽 | 1.16 | 1.22 |
| 整车长 | ~2.89（叉尖 -2.043↔+0.85） | 3.299 |

- Gazebo diff_drive 只用 `wheel_separation`+`wheel_diameter`,所以 sim 自洽、历史 acceptance 作为**仿真**仍有效;但 controller/gate 现在按真车 0.937 轮距 / 1.4 轴距算,sim 车并没有这套几何,转向行为不会 1:1 迁移。若要用 sim 验真车参数,需同步 URDF 并重跑 acceptance。
- safety gate footprint `[[0.843,0.58],...,[-2.043,0.58]]`（2.886×1.16）也是 sim 几何;真车 3.299×1.22,上真车前(P8.4)必须放大。
- **转向方式已确认(2026-06-24,厂家)**:真车是**后轮转向**(规格图纸 转向电机 YDZ48400A-G45 / 齿轮箱速比 45 / 转向齿轮 110/25 / 电转向 / 转弯半径 1760mm)。原地回转的实现方式 = **把转向角打到 90°,再发驱动转速**,Curtis 内部协调两驱动轮+后轮,车**绕两个驱动轮所在轴的中心**旋转。
  - 与现有模型一致,**无需改代码**:MPC `forklift_vehicle_model.cpp` pivot 时绕 `rear_axle_x_offset`(=驱动轴中心)旋转;`curtis_command_kinematics` pivot 时 `v_outer=ω·track/2`(驱动轮在中心 ±track/2);两边触发条件都是「转向角≥90°+给驱动速度」,且共用同一 `pivot_turn_radius`,彼此自洽。
  - **留台架标定项**:① `pivot_turn_radius`(现 0.6,是"速度→转速"增益,非旋转中心;MPC+gate+接口三处同值,标定时一起改)—— 90° + 已知 drive rpm 量实际 °/s 反推;② 非 pivot 正常转弯走自行车模型,真车后轮转向的转向方向语义可能与前轮转向相反,台架一并验。

## 5. P3: 固定控制接口和车辆模型

目标：

明确三支点叉车 controller 到底输出什么。

推荐内部控制模型：

```text
state:
  x, y, theta, phi

control:
  v, steering_angle 或 steering_angle_velocity
```

其中：

- `x, y`: 地图/odom 平面位置。
- `theta`: 车体朝向。
- `phi`: 转向角。
- `v`: 车辆纵向速度。

当前仿真还是 diff drive，所以短期转换为：

```text
omega = v * tan(phi) / wheel_base
/cmd_vel.linear.x = v
/cmd_vel.angular.z = omega
```

真车上车时不要只用 `/cmd_vel.angular.z`，应该优先发：

```text
velocity + steering_angle
```

要做的文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_vehicle_model.hpp
forklift_nav2_plugins/src/forklift_vehicle_model.cpp
```

当前落地方式：

- `forklift_vehicle_model` 统一保存 `wheel_base`、最大转角、最大转角速度、最大速度、最大加速度等车辆参数。
- `ForkliftMpcController` 内部继续返回 Nav2 必须的 `TwistStamped`，但可通过 `publish_control_cmd` 参数同步发布 `/forklift/control_cmd`。
- 仿真阶段可以用 `sim_command_bridge` 把 `/forklift/control_cmd` 转成 Gazebo 的 `/cmd_vel`；默认不开，避免和 Nav2 默认 `/cmd_vel` 链路抢同一 topic。

验收标准：

- 同一条 path 下，仿真 controller 和真实车接口使用同一个车辆参数：

```text
wheel_base
max_steering_angle
max_steering_angle_velocity
max_velocity
max_acceleration
```

## 6. P4: 移植 ORU MPC 到 ForkliftMpcController

目标：

把 `orunav_mpc` 的核心控制能力移植进当前 Nav2 controller 插件，而不是单独运行 ORU controller 节点。

目标文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
```

ORU 源码参考：

```text
navigation_oru-release/orunav_mpc/src/state.*
navigation_oru-release/orunav_mpc/src/control.h
navigation_oru-release/orunav_mpc/src/trajectory.*
navigation_oru-release/orunav_mpc/src/previewWindow.*
navigation_oru-release/orunav_mpc/src/qpProblem.*
navigation_oru-release/orunav_mpc/src/qpConstraints.*
navigation_oru-release/orunav_mpc/src/controller.*
navigation_oru-release/orunav_mpc/qpOASES/
```

迁移顺序：

### P4.1 状态和控制结构

先移植或重写：

```text
State:
  x, y, theta, phi

Control:
  v, w 或 v, steering_rate
```

要求：

- 不依赖 ORU 全局变量。
- 不依赖 ORU 自己的 node/thread 框架。
- 可单元测试。

验收：

- 给定当前 pose、速度、转角，能构造 MPC 初始状态。

### P4.2 Path -> Trajectory

把 Nav2 的 `nav_msgs/Path` 转成 controller 内部 trajectory。

要求：

- 支持 `NavigateToPose` 生成的 path。
- 支持手画 `/follow_path` path。
- 支持未来 task_manager 下发的固定路线 path。

验收：

- 同一条 path 能输出带方向、曲率或转角估计的轨迹点。

### P4.3 预瞄窗口

参考 ORU：

```text
previewWindow
trajectoryCache
```

目标：

- 从当前车辆位置截取未来 N 个轨迹点。
- MPC 每次只优化有限时间窗口。

验收：

- RViz 或日志能显示当前 preview window 的起点、终点、长度。

### P4.4 QP/MPC 求解

参考 ORU：

```text
qpProblem
qpConstraints
qpOASES
```

先做最小版本：

- 只约束速度。
- 只约束转角。
- 只约束转角速度。
- 暂时不做复杂多机器人/任务状态。

验收：

- controller 每个周期输出稳定 `v + steering_angle`。
- 不出现剧烈振荡。
- 停车时速度收敛到 0。

### P4.5 接 Nav2 Controller API

保持接口：

```cpp
computeVelocityCommands()
setPlan()
setSpeedLimit()
```

输出策略：

- 仿真：输出 `geometry_msgs/TwistStamped` 给 Nav2 `/cmd_vel` 链路。
- 真车：通过 `forklift_vehicle_interface` 或后续控制命令 bridge 输出 `ForkliftControlCommand`。

验收：

- `/follow_path` 可以跟踪手画路径。
- `NavigateToPose` 可以使用同一个 controller 跟踪 global planner 输出路径。

## 7. P5: 接入轨迹处理和平滑

目标：

让 global planner 或手画 path 不直接变成生硬控制输入，而是先变成适合叉车的平滑轨迹。

ORU 参考：

```text
navigation_oru-release/orunav_path_smoother
navigation_oru-release/orunav_trajectory_processor
```

Nav2 接入方式：

```text
方案 A:
  写 nav2_core::Smoother 插件

方案 B:
  在 ForkliftMpcController 内部做 path preprocessing

方案 C:
  写独立 forklift_trajectory_processor node
```

推荐先做 B，再做 A。

第一版能力：

- 路径插密。
- 路径方向估计。
- 最小转弯半径检查。
- 曲率过大处降速。

验收：

- 手画稀疏 path 会自动变成密集、连续、方向合理的轨迹。
- 90 度急转会被提示或自动平滑。

## 8. P6: 移植 ORU motion planner / primitives

目标：

把 global planner 从普通 grid A* 升级成叉车可行驶路径规划。

当前：

```text
forklift_nav2_plugins/OruGlobalPlanner
  lattice scaffold with x/y/theta_index, forward/reverse primitive metadata, A* fallback
```

目标：

```text
forklift_nav2_plugins/OruGlobalPlanner
  motion primitive / lattice / 车辆运动学约束
```

ORU 参考：

```text
navigation_oru-release/orunav_motion_planner
navigation_oru-release/orunav_geometry
navigation_oru-release/orunav_constraint_extract
navigation_oru-release/orunav_motion_planner/Primitives
navigation_oru-release/orunav_motion_planner/LookupTables
```

迁移顺序：

### P6.1 保留 Nav2 costmap 输入

不要绕开 Nav2 costmap。

输入仍然是：

```text
global_costmap
footprint
map frame
start pose
goal pose
```

验收：

- RViz 的 global costmap 和 planner 看到的障碍一致。

### P6.2 加 heading state

从 2D cell：

```text
x, y
```

升级到：

```text
x, y, theta_index
```

验收：

- 同一个位置不同朝向可以有不同可行性。

### P6.3 加 motion primitives

primitive 类型：

```text
forward straight
forward left arc
forward right arc
reverse straight
reverse left arc
reverse right arc
```

第一版可以先禁用 reverse，稳定后再开。

验收：

- 全局路径不会出现车辆无法执行的原地横移或过急转弯。

### P6.4 footprint collision along primitive

不是只检查终点，而是检查 primitive 轨迹上的多个采样点。

验收：

- 叉臂不会在转弯中扫到障碍。

### P6.5 当前分版路线

P6 不一次性做完整 ORU planner，按分版推进：

```text
[x] 第一版：最小 lattice scaffold
[x] 第二版：cost / diagnostics / goal approach tightening
[x] 第三版：reverse primitives + direction metadata
[x] 第四版 A：最小倒车执行验证
[x] P8.1：最小 safety gate
[x] 第四版 B：倒车 acceptance 调优
[ ] P6.5a：rear-axle pivot primitive + stop-pivot-go acceptance
[ ] 第五版：multi-curvature / multi-length primitives + better heuristic + lookup/cache
[ ] 第六版：narrow aisle / docking / A-B scenario acceptance
```

第三版已完成 planner 侧最小闭环：

- 已加 `reverse straight / reverse left arc / reverse right arc`。
- 已为 primitive/transition 保留方向语义：
  - `forward`
  - `reverse`
  - `straight / left_arc / right_arc`
  - length / heading_delta
- 已加 `lattice_reverse_cost_multiplier` 和 `lattice_gear_switch_cost`。
- 已保留 A* fallback；P6.4a 之后 ORU test runtime 配置已低速打开 `lattice_reverse_enabled`，并通过 controller preview gating 只在 reverse-intent path 段允许负速度。

当前倒车链路说明：

- planner 搜索状态区分 `x/y/theta_index + arrival_direction`，前进到达和倒车到达不会被合并成同一种状态。
- `lattice_reverse_enabled=true` 时，planner 会在 forward straight / left arc / right arc 外，再加入 reverse straight / left arc / right arc。
- reverse primitive 会被 `lattice_reverse_cost_multiplier` 加价；forward/reverse 换向会被 `lattice_gear_switch_cost` 加价，所以 planner 不会无成本地乱用倒车。
- `nav_msgs/Path` 没有 gear 字段，当前用 path pose yaw 表达车体朝向；当 path yaw 与几何运动切线相差超过 90 度时，trajectory preprocessing 会标记 `reverse_motion`。
- controller 只有在 `allow_reverse=true`、`max_reverse_velocity>0`，并且当前 preview window 含 `reverse_motion` 点时，才允许负速度候选。
- 因此普通 forward path 即使全局打开 reverse，也不会在前进路线里随意倒车；P6.4a 的 `sparse_90_turn` 回归已验证 `forward=710 reverse=0`。

第三版之后不要直接等完整倒车场景都调好再做 P8。P6.4 拆成两段：

```text
P6.4a 最小倒车执行验证
P6.4b 倒车 acceptance 调优
```

P6.4a 在 P8.1 前完成，最低标准是：

- planner 可以输出倒车段。
- controller 不会把倒车段当成前进路径硬追。
- vehicle_interface / sim bridge 能收到正确方向的控制命令。
- 简单倒车 path 能低速执行，或至少能在仿真中正确发负速度。

P6.4a 不要求：

- 复杂窄通道倒车入库。
- 最优换向。
- 所有倒车场景稳定。

P6.4a 完成后，优先进入 P8.1 最小 safety gate。P6.4b 的倒车 acceptance 调优可以在 P8.1 后继续。

P6.4b 不从大改算法开始，而是先把倒车能力场景化验收，再根据失败现象调 planner/controller 参数和少量逻辑。

P6.4b 推荐 acceptance 场景：

```text
reverse_straight
  目标在车后方，验证 planner 输出 reverse，controller 输出负速度。

forward_straight
  目标在车前方，验证普通前进路线不乱倒车。

forward_with_goal_heading
  前方目标但终点姿态不同，验证不会为了贴终点姿态突然倒一下。

three_point_turn / narrow_turn
  空间不够直接掉头时，允许 forward + reverse 换向。

blocked_forward_reverse_escape
  前方被挡但后方可退，验证 planner 可以选择短倒车再前进。
```

每个场景至少记录：

```text
planner forward/reverse segment count
planner gear_switches
controller forward/reverse samples
/forklift/sim_cmd_vel 正负速度样本
action status
是否触发 Failed to make progress
路径是否出现横移、急转、不连续 heading
```

调优顺序：

1. 先补 acceptance 脚本和固定场景，不靠手工看 RViz 判断。
2. 先保证普通前进路线不随便倒车：
   - `forward_straight`、`sparse_90_turn`、A-B 普通路线应保持 `reverse_segments=0`、`reverse_samples=0`。
   - 只有目标在后方、空间受限或明确需要换向时，才允许 reverse。
3. 再稳定倒车段执行：
   - 倒车速度低速可控。
   - preview window 不频繁跳变。
   - reverse path heading 连续。
   - `/forklift/control_cmd` 和 `/forklift/sim_cmd_vel` 方向一致。
4. 再优化换向质量：
   - 避免短距离内 `forward/reverse/forward/reverse` 抖动。
   - 优先形成少量、明确的 `forward arc -> reverse arc -> forward arc`。
   - 必要时提高 `lattice_gear_switch_cost` 或加入 minimum segment length / suppress tiny gear changes。

优先检查和可调项：

```text
lattice_reverse_enabled
lattice_reverse_cost_multiplier
lattice_gear_switch_cost
lattice_goal_heading_weight / goal-heading heuristic
lattice_goal_tolerance / yaw tolerance
primitive length / arc radius
allow_reverse
max_reverse_velocity
respect_reverse_path_orientation
preview window 长度和 progress checker 参数
```

P6.4b 验收表：

```text
[x] forward route does not reverse
[x] rear goal uses reverse
[x] reverse segment reaches controller/bridge
[x] forward_with_goal_heading 不因为终点姿态引入倒车
[x] sparse_90_turn 回归不退化（FollowPath SUCCEEDED，reverse=0）
[x] A-B dynamic obstacle 回归不退化
[>] three-point / narrow aisle / docking 正式场景放到 P10/P6.5
```

> **v1 能力边界（故意设计，不是遗漏）：**
> - 当前 `lattice_reverse_requires_goal_behind=true` 作为 transit 阶段默认约束，goal 在前方时不生成 reverse primitive。
> - `PlannerConstraints` 运行时通道（transit/maneuver/dock 语义）推迟到 P7/P10。
> - **v1 明确不支持**：窄道三点掉头、倒车入库 docking（这些需要 task_manager 在运行时 relax 掉 goal-behind guard）。

### P6.5a 上车前 rear-axle pivot primitive

> **状态(2026-06-24):待做,暂缓。** 先把 P8.2(独立 safety gate)和 P8.4(真车低速 safety acceptance)收尾,再回来做 P6.5a。pivot 机制已被厂家确认(后轮转向、绕驱动轮轴中心,见 P2.3b 后「转向方式已确认」),前置不再有疑点。
>
> **P6.5a 待做清单(收尾,不是从零写——planner 侧 `buildDirectPivotPath`/lattice pivot primitives/terminal pivot regime/pivot_segments 已实现):**
> 1. 唯一权威配置 `forklift_nav2_oru_test_foxy.yaml` 打开 `lattice_pivot_enabled`,并让 `lattice_rear_axle_x_offset`/`pivot_steering_angle` 与 MPC、gate 一致(指向驱动轴中心)。
> 2. controller stop-pivot-go:识别 pivot intent → 先停/降速 → 发 pivot command → yaw 到位恢复前进(确认 forklift_mpc_controller 已覆盖,缺则补)。
> 3. safety gate 补 pivot 扫掠 footprint 检查(车尾/配重扫掠弧)。
> 4. 跑 acceptance(下方场景表)+ 回归 forward/reverse/dynamic;勾选 P6.4b 验收表里 `[>]` 的 three-point/narrow 项按需。
> 5. 不动真车换算参数与 `pivot_turn_radius`(台架标定项);全程 Foxy docker。

新增优先级背景：

- 第一版真车测试有很多点位需要“原地/近原地转 90 度后再走”。
- 真车信息已确认：双驱差速，90 度绕后轴旋转。
- 因此不要回到 Ackermann，也不要只靠普通 forward arc / reverse arc 调参硬凑。
- 当前 `sparse_90_turn_ab` 诊断失败不是简单 A-B blocker，但它暴露了短距离贴 90 度终点姿态时，完整 Nav2 重规划/controller/recovery 链路还不稳；如果真车点位大量依赖近原地 90 度转向，上车前必须补这一层。

P6.5a 的目标是一个最小、可回退的 stop-pivot-go 闭环：

```text
前进到转向点
-> 停车或降到接近 0
-> 绕后轴 pivot left/right
-> yaw 到位后继续前进
```

planner 侧最小改动：

- 在 lattice primitive 中新增 `pivot_left` / `pivot_right`，由参数开关控制，例如 `lattice_pivot_enabled`，默认可保持关闭，ORU test 配置再打开。
- pivot 几何以“后轴中心不动、车身 yaw 改变”为准。
- 当前不急着移动 `base_link` 到后轴中心；继续使用 `rear_axle_x_offset` 计算后轴点和旋转后的 base pose，减少对 AMCL、costmap、传感器 TF 和 footprint 的牵动。
- 每个 primitive 先按 heading bin 小步旋转，例如 16 个 heading bins 时每步 22.5 度；90 度由 4 个 pivot primitive 组成，不直接一次跳 90 度。
- pivot 沿途按多个 yaw sample 做 footprint collision，覆盖车体和叉臂扫掠范围；任意采样碰撞就拒绝该 primitive。
- 保留现有 costmap-aware A* / 非 pivot lattice 路径作为 fallback，确保新 primitive 可开关、可回退。

controller / bridge 侧最小改动：

- controller 需要识别 pivot-intent 段，不能把它当成普通前进弧线追踪。
- 执行策略优先用低速、可解释的 stop-pivot-go：先 brake 或降速到接近 0，再发 pivot command，再在 yaw 到位后恢复普通前进/倒车控制。
- pivot command 使用当前已有语义：`allow_pivot_turn=true`、`pivot_steering_angle` 接近 90 度、低速 `velocity_mps` 决定 yaw rate，`steering_angle` 和 forward/reverse 方向共同决定旋转方向。
- 如果 `nav_msgs/Path` 不能稳定表达 pivot intent，就在 trajectory preprocessing 中通过“后轴点近似不动 + path yaw 连续变化”识别，或增加内部 transition metadata；不要靠横移假路径伪装 pivot。

safety 侧必须补的边界：

- P8.1 当前主要检查 forward/reverse 方向保护区；pivot 时需要检查旋转扫掠 footprint。
- pivot 执行前和执行中都要检查 swept footprint，障碍进入旋转包络时必须停车，不允许继续硬转。
- P8.2 独立 safety gate 要把 pivot/backoff 这类 recovery 或 planner command 纳入统一命令闸门，不能让 Nav2 默认 `/cmd_vel` 或裸 pivot command 直接进底盘。

P6.5a acceptance：

```text
pivot_90_left_in_place
  同一后轴中心附近，yaw 从 0 转到 90 度。
  门槛：pivot_control_samples >= 1，abs_sim_angular_z >= 0.05，NavigateToPose SUCCEEDED。

pivot_90_right_in_place
  同一后轴中心附近，yaw 从 0 转到 -90 度。
  门槛：pivot_control_samples >= 1，abs_sim_angular_z >= 0.05，NavigateToPose SUCCEEDED。

pivot_90_then_forward_ab
  先近原地转 90 度，再前进到 B 点。
  门槛：pivot_control_samples >= 1，reverse_control_samples = 0，NavigateToPose SUCCEEDED。

pivot_blocked_stop
  在 pivot 扫掠范围内放障碍，planner 或 safety gate 必须拒绝/停车。
  门槛：NavigateToPose ABORTED 或 controller 输出 zero command。

l_shaped_corridor_ab（真车典型场景）
  起点面朝过道方向，先沿过道前进约 1.5 m，到路口后 pivot 90°，再进侧道前进。
  场景比 sparse_90_turn_ab 的腿更长（接近真车仓库点位），验证 replanning 不退化。
  门槛：pivot_control_samples >= 1，reverse_control_samples = 0，NavigateToPose SUCCEEDED。

sparse_90_turn_ab（从「诊断项」升为「硬验收门」）
  起点 (-2.0, -0.5, 0°)，终点 (-1.3, 0.2, 90°)。
  终点 90° 姿态由终端 pivot 贴近，不靠 reverse 弧。
  门槛：reverse_control_samples = 0，NavigateToPose SUCCEEDED。
  说明：v1 上车前此场景必须 SUCCEEDED。pivot primitive 上线后不再是诊断项。

forward_ab / reverse_ab / dynamic_stop_release_ab 回归
  确认普通 A-B、倒车、动态障碍停车/放行不被 pivot primitive 破坏。
```

终点姿态策略（v1）：

- 保留 `use_final_approach_orientation=true`，终点朝向是硬约束。
- **终点姿态由终端 pivot 执行**：前进或弧线到达终点附近 → planner 输出 pivot primitive 修正剩余 yaw 偏差 → controller stop-pivot-go 贴近目标 yaw。
- **不靠 reverse 弧贴终点姿态**，`lattice_reverse_requires_goal_behind=true` 阻止 goal 在前方时生成 reverse primitive。

P6.5a 不要求：

- 完整 ORU primitive lookup/cache。
- 高速连续圆弧转弯的最优平滑。
- 窄通道三点掉头、倒车入库、复杂 docking 全部稳定。
- 立即移动 `base_link` 到后轴中心。
- `PlannerConstraints` 运行时约束通道（transit/maneuver/dock 语义推迟到 P7/P10）。

## 9. P7: 建 forklift_task_manager

当前优先级：

```text
P7 暂缓，放到 P6.4a、P8.1、A-B acceptance 和 P8.3 之后。
```

原因：

- 简单 A 到 B 可以先直接用 Nav2 `NavigateToPose` / `FollowPath` 验收，不需要 task_manager。
- 任务层不会直接提升车辆运动能力，也不会替代动态障碍安全停车。
- 等 planner/controller/safety 闭环稳定后，再做 task_manager 的暂停、恢复、重试和调度接口更稳。
- P7.1 进入前，至少要证明简单 A-B 能跑、动态障碍能停/放行、持续阻挡能等待或重新规划；否则 task_manager 只会把底层不稳定性包装成任务失败。

目标：

统一三种任务入口：

```text
入口 A:
  A 点到 B 点自动规划
  NavigateToPose

入口 B:
  多点任务
  NavigateThroughPoses / FollowWaypoints

入口 C:
  固定路线或手画路线
  FollowPath
```

新 package：

```text
forklift_task_manager
```

职责：

- 接调度系统。
- 保存站点。
- 保存固定路线。
- 调用 Nav2 action。
- 记录任务状态。
- 管理等待、装卸货、充电等流程。

验收：

- 通过一个简单 service/action 下发：

```text
go_to_station("A")
follow_route("dock_to_truck")
cancel_task()
```

- task_manager 不直接发布 `/cmd_vel`。

### P7 设计（边界 / 组件 / 落地节奏）

> 2026-06-18 补充。结论：**做最小版 P7.1，而且 v1 现在就该做**——v1 走 Option A（路线拆成已验证原子段顺序执行），这个“按段下发 Nav2 goal 的序列器”本质就是 v1 路点序列器硬门，不是可选项。巡逻 / 充电 / 重定位是同一组件的自然扩展，后续增量加。

**核心边界原则（先立规矩）**

task_manager = **大脑 / 编排层**，只决定“做什么、什么顺序、失败怎么办”，向下调用别人干活：

```text
task_manager  (编排: 什么 / 何时 / 重试)
   ├─→ Nav2 action (NavigateToPose / FollowPath)   ← 运动
   ├─→ 安全层 (P8.2 command gate)                   ← 可随时抢占 / 急停 task_manager
   ├─→ vehicle_interface 服务                        ← 底盘 / 充电物理握手
   └─→ amcl 服务 / initialpose                       ← 重定位
```

硬规矩（三不）：

1. **不**直接发 `/cmd_vel` 或 `ForkliftControlCommand`（运动只经 Nav2 + 安全闸）。
2. **不**写底盘 / 充电 / CAN 协议（属 vehicle_interface）。
3. **不**做即时急停、**不**管节点 lifecycle 重启（急停在安全层；重启在 supervisor/launch 层）。

**能力归属（含本轮讨论的扩展想法）**

| 能力 | 放 task_manager？ | 处理方式 |
|---|---|---|
| 路线分解 / 路点序列 | ✅ **核心（v1）** | task_manager 本体。route = 有序 segment 列表，每段 `单次规划 + FollowPath`，关在线重规划 |
| 绕场循环巡逻 | ✅ | 一条 `loop: true` 的 route，数据驱动，几乎零额外代码 |
| 充电 | ⚠️ **拆开** | 编排在此（电量低 → 去充电桩 → 对位 → **触发**充电 → 充满 → 恢复任务）；**物理充电握手**（继电器 / 充电机协议）放 vehicle_interface |
| 重新定位 | ✅ 作为 recovery 动作 | 调 amcl `/reinitialize_global_localization` 或重发 `/initialpose`、走一小段已知图案再收敛。**“重启节点”不放这**——属 supervisor/launch |
| 紧急停靠 | ❗ **拆开** | **即时急停（E-stop）必须在安全层 / watchdog**，独立、即时、不依赖 task_manager 活着；**“开去指定车位安全停靠”**（可控停车）可做成 task_manager 任务 |

> 为什么急停不能放 task_manager：急停要在 task_manager 崩了也能切断运动，它是反射不是决策。task_manager 只能“观察到 E-stop → 任务转 PAUSED”，不能是急停的执行者。

**组件设计**

1. 数据（纯配置，不写死代码）：
   - `stations.yaml`：命名位姿（`A`、`charger`、`park_bay`…）。
   - `routes.yaml`：有序 segment + 元数据，例如：
     ```yaml
     patrol_loop:
       loop: true
       segments:
         - {to: corner_1, type: drive}
         - {to: corner_1, type: pivot, yaw: 90}
         - {to: corner_2, type: drive}
     ```
2. 状态机：`IDLE → RUNNING → (PAUSED) → SUCCEEDED / FAILED / RECOVERING`。E-stop / 安全层抢占 → 自动进 PAUSED，解除后需显式 resume。
3. 对外接口（只发任务，不发运动指令）：action `ExecuteRoute(name, loop)`；service `pause` / `resume` / `cancel` / `go_charge` / `go_park`；topic `/task_status`（当前段、进度、状态、失败原因）。
4. 执行器（每段）：取下一段 → 下发 Nav2 goal（单次规划 + FollowPath）→ 等结果 → 成功则下一段（loop 回首段），失败进 recovery。
5. recovery 策略（对齐 P8.3 顺序，且都受安全闸约束）：`重试 N 次 → 重定位 → 等待 → 升级 FAILED / 请求人工`；低速 pivot/backoff 只有安全闸允许才执行。
6. 后台监控：订 `/battery_state` 低电 → 自动排入 `go_charge` 路线；订安全层 E-stop 状态 → 任务 PAUSED。

**落地节奏**

- **v1 现在做（最小 P7.1）**：route 序列器 + 状态机 + start/pause/resume/cancel + 重试/失败。这就是 v1 硬门，L 路线靠它跑。
- **紧接着**：巡逻 loop（几乎免费）、重定位 recovery。
- **再后面**：充电编排（配合 vehicle_interface 充电握手）、可控停靠车位。
- **永远不进 task_manager**：即时急停、CAN / 充电协议、节点重启。

## 10. P8: 建 forklift_safety

当前优先级：

```text
P8.1 已提前到 P7.1 之前完成；P8.2 再抽成独立 forklift_safety package。
```

第一阶段目标不是复杂动态绕行，而是安全停车/限速：

```text
动态障碍进入保护区 -> 停车
障碍物离开 -> 放行或允许 Nav2 继续
障碍物持续存在 -> 触发等待/后续重新规划策略
控制命令超时 -> 停车
急停输入 -> 停车
```

P8.1 当前落地：

- `ForkliftMpcController` 增加最小 safety gate，沿当前运动方向在 local costmap 上采样 footprint。
- 障碍进入 `safety_stop_distance` 时直接输出 brake/zero command。
- 障碍进入 `safety_slowdown_distance` 但还没到 stop zone 时压低当前方向速度上限。
- reverse-intent preview 时检查车后方保护区；普通 forward preview 时检查车前方保护区。
- `safety_emergency_stop_active` 参数可运行时置 true，让 controller 直接停车。
- `sim_command_bridge` 已有 `/forklift/set_emergency_stop` 和 `command_timeout_sec` watchdog，继续作为仿真/vehicle_interface 侧最后一道命令闸门。

P8.1 参数：

```yaml
safety_gate_enabled: true
safety_emergency_stop_active: false
safety_stop_distance: 0.55
safety_slowdown_distance: 1.25
safety_min_speed: 0.05
safety_sample_spacing: 0.10
```

P8.1 不是复杂动态绕行，也不是最终独立 safety 架构。它先给 A-B 运行加上可回退的最小停车/限速闸门；后续 P8.2 再把安全逻辑抽成独立 `forklift_safety` package 或接入 `nav2_collision_monitor`。

长期目标：

上车必须有独立安全层，不依赖 planner/controller 自觉避障。

后续新 package：

```text
forklift_safety
```

职责：

- 急停。
- 命令 watchdog。
- 限速区。
- 掉边保护。
- 防撞监控。
- 车辆状态异常停车。
- 动态障碍物进入保护区时，对 controller 输出做安全闸门处理。

可结合 Nav2：

```text
nav2_collision_monitor
costmap filter / keepout mask
speed zone
local/global costmap obstacle observation
```

验收：

- 任意时候急停都能切断运动命令。
- `/cmd_vel` 超时自动停车。
- 障碍物进入保护区时停车或限速。
- 障碍物离开后，不需要重启 Nav2 即可继续执行或重新下发目标。
- 地图边缘/掉落风险区域不会允许继续行驶。

P8.2 最低标准：

- 把 controller 内的最小 safety gate 抽象成独立 `forklift_safety` package 或明确的命令闸门节点。
- controller、task_manager、手动控制等所有运动命令都必须经过 safety gate。
- 真车 recovery 命令也必须经过 safety gate；不允许 Nav2 默认 `/cmd_vel` 直接控制底盘。
- 明确 recovery command adapter：把允许的 recovery 动作转换成受限的 `ForkliftControlCommand` 或等价安全命令。
- 第一版只白名单低速、短时、可解释的 recovery primitive，例如 wait、clear-costmap 后重试、受限 pivot/backoff；所有 recovery 都要受急停、watchdog、速度上限、footprint collision 和车辆状态约束。
- 急停输入能锁住运动命令，解除后需要明确状态恢复。
- command timeout、vehicle fault、localization lost、costmap 数据异常时停车。
- 继续保留 controller-side 限速/停车作为可回退保护，不把安全完全交给 planner。

P8.2 剩余缺口（2026-06-18 已补完 8.2-4/5/6，下列为已闭环状态 + 真车前遗留项）：

`forklift_safety` package + `safety_command_gate` 已提交，命令链路、急停服务、watchdog、命令超时/车辆故障/定位丢失即停、速度/转角限幅、recovery 白名单 adapter、以及下面三项都已具备并在 Foxy docker 内单测通过（`forklift_safety` 14 pytest）：

- **缺口 1（已补完）— costmap 数据异常/过期即停**：gate 订阅 `/local_costmap/costmap`（可选 `costmap_message_type:=costmap_raw`），costmap 缺失/超时/空/截断/无效尺寸都会输出停车命令并在 `/forklift/safety_gate/status` 说明原因。
- **缺口 2（已补完）— gate 内扫掠 footprint 碰撞检查**：gate 按 Foxy ORU footprint 参数，沿当前位姿和短时预测位姿采样 footprint 边界查 costmap，遇 unknown/出图/lethal 即停，覆盖 raw 命令与 recovery wait/backoff/pivot。
- **缺口 3（部分）— Foxy docker 验收**：build（6 包）+ `foxy_colcon_test.sh`（83 tests / 0 failures）为实跑结果；但 footprint 碰撞停车、costmap timeout 等 live 行为目前是 `forklift_safety/P8_2_FOXY_ACCEPTANCE.md` 里的「可复现命令 + 预期输出」，**尚未实起节点抓日志**。真车前需补一次 live 端到端抓取。

真车前遗留（不阻塞 P8.2 标记，但进 P8.4 前必须闭环）：

- **drive_rpm 限幅**：gate 已对 `drive_rpm` 做 `|rpm|<=max_drive_rpm`（默认 2500）限幅，并把 `accel_time_sec`/`decel_time_sec` 缺省补成厂家建议值（5s/3s）。注意真车 0x203 驱动帧用的是 `drive_rpm` 不是 `velocity_mps`，所以 velocity→rpm 转换必须放在 gate **下游**（`curtis_vehicle_interface`，用已限好的 velocity_mps 算），否则限速会被绕过——归 P2.3。

P8.3 最低标准：

- 动态障碍横穿或短时挡路时，车辆停车等待。
- 障碍离开后，Nav2 不需要重启即可继续执行，或由上层重新触发当前目标。
- 障碍持续挡住原路径时，触发重新规划。
- 定义真车 recovery 策略选择顺序：优先 wait，其次 clear/replan；只有安全闸门允许时才执行低速 pivot/backoff。
- recovery 失败时进入任务暂停/失败状态，不能反复执行可能扩大风险的动作。
- costmap 中存在足够通道时允许简单绕行；通道不足时不硬绕，保持等待或任务失败。
- 这一阶段不要求高级人群预测，也不要求复杂动态博弈避障。

P8.4 最低标准：

- 速度相关保护区：速度越快，前向/后向保护距离越大。
- 前进和倒车使用不同保护区，倒车时重点保护车尾与叉臂相关外形。
- footprint 覆盖车体、叉臂和必要安全余量。
- keepout 区、地图边缘、掉落风险区域不允许继续行驶。
- safety 日志和诊断能说明停车、限速、等待、失败的原因，便于真车低速复盘。

### P8.2 / P8.3 / P8.4 区别速查

三者在安全这条线的不同层：P8.2 是“建闸门”（架构/机制），P8.3 是“动态障碍行为”，P8.4 是“在真车上证明它安全”（验收/标定）。

| | **P8.2 独立 safety / command gate** | **P8.4 真车低速 safety acceptance** |
|---|---|---|
| 本质 | **机制层**：把命令收进一个独立安全闸 | **验收+标定层**：在真车上跑出来、调参数 |
| 交付物 | 独立 `forklift_safety` package / 命令闸节点 | 真车低速验收包 + 标定好的保护区参数 |
| 核心内容 | ① 所有运动命令（controller / task_manager / 手动 / **recovery**）都必须过闸；② **recovery command adapter**：只白名单低速、短时、可解释的 recovery（wait、清代价图重试、受限 pivot/backoff），转成受限 `ForkliftControlCommand`；③ 急停锁命令、watchdog、命令超时/车辆故障/定位丢失/costmap 异常即停；④ 不允许 Nav2 裸 `/cmd_vel` 直接进底盘 | ① **速度相关保护区**：越快前/后保护距离越大；② 前进/倒车**不同保护区**（倒车重点护车尾+叉臂）；③ footprint 覆盖车体+叉臂+安全余量；④ keepout / 地图边缘 / 掉落区禁行；⑤ safety 日志能解释每次停/限速/等待/失败 |
| 在哪验证 | 仿真/台架即可（架构正确性） | **必须在真车低速实跑**签收 |
| 一句话 | “任何命令都别想绕过安全层进底盘” | “在真车上把保护区调对、并证明它真停得住” |

> 顺序：P8.2（建闸）→ P8.3（动态障碍等待/重规划/简单绕行行为）→ P8.4（真车把保护区标定+签收）。P8.2 是 P8.4 的前提——没有统一闸门，P8.4 没法保证所有命令（尤其 recovery）都受保护。

P8.1 到 P7.1 之间的推荐顺序：

```text
P8.1 已完成
-> A-B acceptance 快速验证已完成
-> A-B 动态障碍停车/放行快速验证已完成
-> P6.4b 倒车 acceptance 调优已完成
-> A-B acceptance 正式验收已完成
-> P6.5a rear-axle pivot primitive / stop-pivot-go acceptance
-> P8.2 独立 safety gate
-> P8.3 动态障碍等待/重规划/简单绕行
-> P8.4 真车低速 safety acceptance
-> P7.1 task_manager 最小任务入口
```

快速 A-B acceptance 已证明空旷 `NavigateToPose` 链路可跑通；A-B 动态障碍停车/放行快速验证也已证明 P8.1 safety gate 在真实导航执行中能停车、放行后能继续跑完同一目标。P6.4b 已把当前最小 lattice scaffold 的倒车 acceptance 调稳，A-B 正式验收主线也已通过。由于第一版真车点位大量依赖近原地 90 度转向，下一步建议先做 P6.5a rear-axle pivot primitive，再进入 P8.2，把 pivot/backoff/recovery 等真实运动命令统一收进独立 safety gate。

真车 recovery 不是忽略项，但不要用仿真 bridge fallback 的方式直接上车。仿真 fallback 只是解决 Gazebo bridge 模式下 recovery `/cmd_vel` 到不了 `/forklift/sim_cmd_vel` 的接线问题；真车阶段应在 P8.2/P8.3 中实现受控 recovery：

```text
Nav2/controller normal command
  -> safety gate
  -> vehicle_interface
  -> 真车底盘

Nav2/custom recovery request
  -> recovery command adapter
  -> safety gate
  -> vehicle_interface
  -> 真车底盘
```

原则：

- 真车底盘不裸订阅 Nav2 默认 `/cmd_vel`。
- recovery 可以执行，但必须转换成叉车约束下的安全命令。
- 所有 recovery 命令都要经过急停、watchdog、限速、footprint collision、车辆状态和传感器健康检查。
- P8.2 解决“recovery 命令怎么安全进真车”；P8.3 解决“什么情况下执行哪一种 recovery”。

动态障碍停车/放行快速验证最低标准：

- `NavigateToPose` 正在执行时，障碍进入 slowdown 区，速度被压低。
- 障碍进入 stop 区，controller 输出停车或 brake。
- 障碍离开后，不重启 Nav2，车辆能继续执行；如果当前行为树无法自然继续，至少能重新下发同一目标继续。
- 无障碍时，P8.1 safety gate 不误停。
- 本阶段不要求复杂绕行；持续阻挡时允许等待或任务失败，但不能硬撞或继续推障碍。

如果后续 P6.5a 暴露的是 pivot 几何、primitive 表达或 controller stop-pivot-go 问题，优先在 P6.5a/P10 解决；如果暴露的是命令闸门、recovery 入口或真车安全边界问题，进入 P8.2/P8.3 解决。

## 11. P9: 真车低速联调

第一版上真车前的门槛要分清三层：

```text
台架/离地硬件联调:
  可以在 P2.3 完成后做，不启动 Nav2。

地面低速手动控制:
  P2.3 + 急停 + watchdog + /odom + TF 稳定后做。

第一版 Nav2 自主 A-B:
  必须完成到 P8.4；P7.1、P10+ 和 ILIAD 都不是前置条件。
```

第一版 Nav2 自主 A-B 上真车前必须完成：

- P2.3 真车 `forklift_vehicle_interface` 实际 I/O：不再只是 dry-run 日志，必须能向底盘发送控制，并发布真实反馈。
- A-B acceptance 快速验证和正式验收：普通路线、倒车/换向路线、障碍停车/放行都在仿真中通过。
- P6.4b 倒车 acceptance：当前最小 lattice scaffold 中普通前进路线不乱倒、后方目标能倒、90 度和动态障碍回归不退化；更复杂的窄通道/三点掉头/倒车入库放到 P10/P6.5。
- P8.2 独立 safety gate 或明确命令闸门：所有运动命令经过安全层。
- P8.3 动态障碍等待、重新规划、简单绕行和受控 recovery 策略：能停、能等、能放行，通道不足时不硬绕。
- P8.4 真车低速 safety acceptance：速度相关保护区、前进/倒车不同保护区、keepout、地图边缘、诊断日志。
- 真实地图、真实传感器、真实 vehicle interface 参数都已经替换仿真默认值。

P7.1 不作为第一版上车前置条件。第一版可以直接用 Nav2 `NavigateToPose`、`FollowPath` 和 RViz/CLI 下发目标；等底层跑稳后，再加 `forklift_task_manager` 统一站点、路线、暂停、恢复、取消和调度接口。

第一版上真车前必须通过的仿真测试：

- `forklift_nav2_plugins` 单元测试全部通过，包括 planner、controller、trajectory、preview window、vehicle model、safety gate。
- headless Nav2/Gazebo readiness：`/clock`、`/odom`、`odom -> base_link`、2D Pose Estimate 后 `map -> odom`、Nav2 lifecycle active。
- `ComputePathToPose`：普通前进目标、90 度目标、后方倒车目标都能规划成功。
- `FollowPath`：短直线、大圆弧、稀疏 90 度、简单倒车 path 都能低速执行成功。
- `NavigateToPose`：空旷 A-B、含 90 度转弯 A-B、含一次倒车/换向 A-B 都能成功或给出可解释失败。
- 动态障碍测试：障碍进入保护区停车/限速，障碍离开放行，持续阻挡触发等待或重新规划。
- safety 回归：急停 service/输入触发停车，命令超时停车，local costmap 无障碍时不误停。
- 传感器异常测试：`/scan` 丢失、`/odom` 丢失、TF 超时、costmap stale 时进入停车或任务失败状态。
- 地图边界/keepout 测试：不会规划到地图外、未知区、掉边风险区或禁止通行区。

第一版真车实际会用到的本项目 package：

```text
forklift_msgs
  共享控制、车辆状态、故障、IO、急停和模式 service。

forklift_vehicle_interface
  真车底盘接口。第一版上车前必须把 curtis_vehicle_interface 从 dry-run
  补成真实 CAN/串口/TCP I/O，并发布 /odom、TF、vehicle_state、fault_state、io_state。

forklift_nav2_plugins
  OruGlobalPlanner、ForkliftMpcController、trajectory preprocessing、
  controller-side safety gate。第一版上车必须使用。

forklift_nav2_demo
  当前仍承载 launch、Nav2 参数、地图、URDF、仿真入口。第一版可以继续用，
  但真车应新增 real bringup launch，不能启动 Gazebo。

forklift_safety
  P8.2 后新增或等价实现。第一版 Nav2 自主 A-B 前应作为安全命令闸门存在。
```

第一版上车前需要改动的 package：

- `forklift_vehicle_interface`：最大改动点。补真实 CAN/串口/TCP 传输、反馈解析、`/odom`、TF、`vehicle_state`、`fault_state`、`io_state`、急停、模式切换、watchdog。
- `forklift_nav2_demo`：新增 real launch 和 real 参数文件，真车启动不带 Gazebo，`use_sim_time=false`，地图路径、传感器 topic、速度限制、safety 参数使用真车 profile。
- `forklift_nav2_plugins`：原则上保持算法 plugin，不写底盘协议；只根据 A-B/P8 acceptance 调 planner/controller/safety 参数，必要时补真实场景暴露的碰撞、倒车、限速逻辑。
- `forklift_safety`：P8.2 新增或等价实现，作为真车运动命令闸门。
- `forklift_msgs`：尽量保持稳定；只有真实底盘反馈或 IO 状态字段不够时才扩展消息。

第一版上车前不要改：

- 不改 `/opt/ros/*` 里的 Nav2 源码。
- 不把底盘协议写进 `forklift_nav2_plugins`。
- 不把 `navigation_oru-release` 或 `iliad` 直接接到真车主启动链路。

第一版真车不需要启动或迁移：

```text
navigation_oru-release/*
iliad/*
forklift_sim
forklift_warehouse_sim
orunav_vehicle_execution
orunav_coordinator_fake
```

`forklift_sim` 和 `forklift_warehouse_sim` 只是早期轻量仿真/开发包，不作为第一版真车或正式 Nav2 acceptance 的主入口。

真车建议启动链路：

```text
1. 传感器 driver
   发布 /scan，必要时发布 rear scan 或 360 度 scan。

2. forklift_vehicle_interface
   订阅 /forklift/control_cmd。
   发布 /odom、odom -> base_link TF、/forklift/vehicle_state、
   /forklift/fault_state、/forklift/io_state。
   提供 /forklift/set_emergency_stop、/forklift/set_control_mode。

3. robot_state_publisher 或 static TF
   发布 base_link -> base_footprint、base_link -> base_scan、
   以及其他传感器安装位姿。

4. Nav2 bringup
   map_server、AMCL、planner_server、controller_server、bt_navigator、
   local/global costmap、recoveries。
   参数使用 real profile，use_sim_time=false。

5. forklift_safety
   P8.2 后作为所有运动命令的最后安全闸门。

6. RViz
   只作为调试和初始位姿/目标下发工具，不作为安全链路。
```

当前 `forklift_nav2_demo/launch/forklift_navigation.launch.py` 会启动 Gazebo，不适合真车直接使用。上车前应新增：

```text
forklift_nav2_demo/launch/forklift_real_navigation.launch.py
```

或后续拆成：

```text
forklift_bringup/launch/real_navigation.launch.py
forklift_description
```

真车 real launch 最低要求：

- 不 include `forklift_gazebo.launch.py`。
- `use_sim_time=false`。
- 启动或接入真实 `forklift_vehicle_interface`。
- 启动 robot_state_publisher/static TF。
- 启动 Nav2 bringup，并加载真实地图和 real 参数文件。
- 可选启动 RViz，但 RViz 不参与安全闭环。

仿真主入口：

```bash
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml \
  use_sim_time:=true \
  use_rviz:=false \
  gazebo_gui:=false \
  use_sim_command_bridge:=true
```

仿真会用到：

- `forklift_nav2_demo`：Gazebo world、URDF、spawn、Nav2 launch、wait_for_sim_ready。
- `forklift_vehicle_interface`：`sim_command_bridge`，把 `/forklift/control_cmd` 转成 Gazebo `/forklift/sim_cmd_vel`。
- `forklift_nav2_plugins`：planner/controller/safety gate。
- `forklift_msgs`：共享控制和状态消息。
- Nav2、Gazebo、robot_state_publisher、AMCL、map_server、costmap。

地图格式要求：

- 第一版继续使用 Nav2 标准 2D occupancy grid 地图。
- 文件格式是 `map.yaml + .pgm`，例如：

```yaml
image: forklift_factory_big_map_clean.pgm
mode: trinary
resolution: 0.05
origin: [-12.4, -9.02, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- `map.yaml` 描述图片、分辨率、原点和阈值；`.pgm` 保存占据栅格。
- 地图坐标系是 `map`，AMCL 发布 `map -> odom`。
- 地图只表达静态环境：墙、货架、固定禁行区域。动态障碍不要画进静态地图。
- keepout、限速区、掉边风险区后续用 costmap filter 或 safety layer 表达，不直接涂进普通 occupancy map。
- 上真车前需要用真实场地重新建图或校准当前地图，确认分辨率、原点、方向和 RViz 中实际位置一致。

第一版必须输入的数据和话题：

```text
/scan
  sensor_msgs/LaserScan
  2D 激光雷达输入，当前 Nav2 local/global costmap 和 AMCL 都使用 scan。
  frame 建议为 base_scan，并通过 TF 固定到 base_link。

/odom
  nav_msgs/Odometry
  来自 forklift_vehicle_interface 或可靠里程计融合。
  必须和 odom -> base_link TF 一致。

odom -> base_link TF
  真车里程计坐标变换。方向、速度符号、时间戳必须稳定。

base_link -> base_scan TF
  激光安装外参。必须和真实安装位置一致。

base_link / base_footprint
  当前 AMCL 配置使用 base_footprint，costmap 使用 base_link。
  两者需要固定 TF 或统一配置，不能缺一条。

/forklift/control_cmd
  ForkliftControlCommand，controller 或 safety gate 输出给 vehicle_interface。

/forklift/vehicle_state
/forklift/fault_state
/forklift/io_state
  真车反馈、故障、IO、安全输入状态。

/forklift/set_emergency_stop
/forklift/set_control_mode
  急停和控制模式 service。

/initialpose
  geometry_msgs/PoseWithCovarianceStamped，AMCL 初始位姿。

NavigateToPose / FollowPath action
  第一版 A-B 和固定路径测试入口。
```

第一版传感器最低要求：

- 2D 激光雷达，能覆盖前进方向的局部避障区域，发布 `sensor_msgs/LaserScan`。
- 如果真车第一版要测试倒车，必须有后向雷达、360 度雷达，或能覆盖车尾/叉臂风险区的等价传感器；否则真车倒车必须限速并限制场景。
- 轮速/编码器/底盘反馈，能生成稳定 `/odom` 和 `odom -> base_link`。
- 转向角或等价运动状态反馈。如果底盘没有真实转向角，也要在 `ForkliftVehicleState` 中提供 controller/safety 可解释的状态。
- 急停、自动/手动模式、停车制动、故障码、通信状态输入。
- IMU 不是第一版硬要求，但如果 odom yaw 漂移明显，应接入 IMU 或 robot_localization。
- 3D 点云/相机不是第一版硬要求；如果后续使用 PointCloud2，需要同步修改 Nav2 costmap observation source。

上车前需要你确认或提供的输入：

- 真车底盘通信协议：CAN ID、字节定义、比例系数、方向/刹车/使能语义、反馈帧、故障帧。
- 车辆几何：车体长宽、叉臂长度、base_link 位置、后轴/旋转中心偏移、最小转弯/原地转能力。
- 速度和加速度限制：前进、倒车、转向角速度、最大角速度、急停/减速时间。
- 传感器安装位姿：雷达相对 base_link 的 x/y/z/yaw，是否有后向或 360 度覆盖。
- 地图：真实场地 `map.yaml + pgm`，或允许重新 SLAM 建图。
- 安全区域：禁行区、掉边区域、限速区、测试区域边界。
- 上车测试流程：谁看急停、最大速度、测试路线、允许倒车的区域、失败时如何人工接管。

顺序：

```text
1. 不启动 Nav2，只测 vehicle_interface 通信
2. 测 /odom 和 TF
3. 测手动低速 /forklift/control_cmd
4. 测 FollowPath 短直线
5. 测 FollowPath 大圆弧
6. 测 NavigateToPose 空旷区域
7. 测障碍物停车
8. 测任务点和调度流程
```

低速限制：

```text
max_velocity <= 0.2 m/s
max_steering_angle_velocity 限制到保守值
必须有人看急停
```

禁止一开始测试：

- 窄门。
- 贴货架。
- 平板车边缘。
- 倒车入库。
- 动态人车混行。

## 12. 每次开发的固定验证命令

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/pl/robotest/install/setup.bash

ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml \
  use_sim_time:=true
```

检查 TF：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

手画 path：

```bash
ros2 run forklift_nav2_demo forklift_manual_path_follower \
  --ros-args -p use_sim_time:=true
```

抓日志：

```bash
grep -E "ForkliftMpcController|OruGlobalPlanner|follow_path|Failed to make progress|transform|ERROR|WARN" \
  /tmp/forklift_mpc_test.log
```

## 13. 第一轮实际执行清单

下一步从这里开始：

```text
[x] P0.1 修到 2D Pose Estimate、/odom、TF 稳定
[x] P0.2 让手画 FollowPath 能走完整短路径
[x] P1.1 创建 forklift_msgs
[x] P1.2 定义 ForkliftControlCommand / ForkliftVehicleState
[x] P2.1 创建 forklift_vehicle_interface
[x] P2.2 写仿真 bridge: ForkliftControlCommand -> cmd_vel
[ ] P2.3 真车 vehicle_interface 实际 I/O：底盘控制、反馈、急停、watchdog（代码侧已补齐，待台架/真车验收）
[x] P3.1 写 forklift_vehicle_model
[x] P4.1 把 ORU State / Control 概念移入 ForkliftMpcController
[x] P4.2 把 Path 转成内部 Trajectory
[x] P4.3 加 preview window
[x] P4.4 接最小 QP/MPC 求解
[x] P4.5 接 Nav2 Controller API，并通过 FollowPath / NavigateToPose 验收
[x] P5 接入轨迹处理和平滑
[x] P6.1 最小 lattice planner scaffold：`x/y/theta_index`、forward primitives、沿途 footprint collision、A* fallback
[x] P6.2 lattice 代价/诊断第二版：turn/obstacle/goal-heading cost、拒绝原因统计、goal tolerance 收紧
[x] P6.3 reverse primitives + direction metadata：倒车 primitive、方向语义、reverse/gear switch cost
[x] P6.4a 最小倒车执行验证：确认 planner 输出的倒车段能被 controller/vehicle interface 执行
[x] P8.1 最小 safety gate：动态障碍停车/限速、急停、watchdog
[x] A-B acceptance 快速验证：空旷 NavigateToPose 简单 A 到 B
[x] A-B 动态障碍停车/放行快速验证
[x] P6.4b 倒车 acceptance 调优：当前最小 lattice scaffold 范围内，普通前进不乱倒、后方目标能倒车、90 度和动态障碍回归不退化
[x] A-B acceptance 正式验收：普通路线、倒车/换向路线、障碍停车/放行
[ ] P6.5a 上车前 rear-axle pivot primitive：停下后绕后轴近原地 90 度转向，再继续前进
    子任务（按顺序）：
    [ ] 6.5a-1 Foxy build + 单测通过（63 tests / 0 failures）
    [ ] 6.5a-2 pivot 全链路打通确认：planner pivot_segments>0 → trajectory pivot_points>0 → controller previewHasPivotMotion=true → bridge angular.z≠0, linear.x≈0
    [ ] 6.5a-3 pivot 验收：pivot_90_left/right_in_place、pivot_90_then_forward_ab、pivot_blocked_stop、l_shaped_corridor_ab
    [ ] 6.5a-4 sparse_90_turn_ab 硬验收（reverse=0，NavigateToPose SUCCEEDED）
    [ ] 6.5a-5 三大回归：forward_ab、reverse_ab、dynamic_stop_release_ab
[x] P8.2 独立 safety package / 命令闸门
    [x] 8.2-1 独立 `forklift_safety` package + `safety_command_gate` 节点，命令链路改为 raw→gate→control_cmd
    [x] 8.2-2 急停服务 + watchdog + 命令超时/车辆故障/定位丢失即停 + 速度/转角限幅
    [x] 8.2-3 recovery command adapter：白名单低速 wait/backoff/pivot，转受限 ForkliftControlCommand
    [x] 8.2-4 costmap 数据异常/过期检查即停
    [x] 8.2-5 gate 内扫掠 footprint 碰撞检查
    [x] 8.2-6 Foxy docker 端到端验收记录（gate 起停、急停锁定/解除、recovery 白名单、超时停车、限幅、costmap timeout、footprint collision）
[ ] P8.3 动态障碍等待、重新规划、简单绕行
[ ] P8.4 真车低速 safety acceptance 包
[ ] P7.1 task_manager 最小任务入口
[ ] P10 motion planner lookup/primitives 加强
[ ] P11 path smoother + constraint_extract
[ ] P12 评估是否上完整 ORU QP-MPC
[ ] P13 需要 human-aware 能力时，再评估 ILIAD 相关包
```

建议我们下一步做：

```text
P6.5a 上车前 rear-axle pivot primitive / stop-pivot-go acceptance
```

原因：

- P6.4b 已经把当前最小 lattice scaffold 的倒车 acceptance 固化成脚本，并完成前进、倒车、90 度和动态障碍回归。
- P8.1 已经加上最小 safety gate、急停参数和 bridge watchdog 基线。
- 空旷 A-B 快速验证和动态障碍停车/放行快速验证都已经通过。
- A-B 正式验收主线已经通过：普通前进、后方目标倒车、动态障碍停车/放行都能复跑。
- 第一版真车点位大量需要“原地/近原地 90 度后再走”；真车已确认是双驱差速、绕后轴旋转，所以需要先补 rear-axle pivot primitive，不回 Ackermann，也不靠普通 arc 硬凑。
- 真车 recovery 命令闸门已经明确放入 P8.2/P8.3；但在进入真车 safety 架构前，先把 planner/controller 的 pivot 基础运动能力调稳，能减少后续 safety/recovery 层需要兜底的问题。

执行记录：

- 2026-06-09：P0.1 通过。headless Gazebo/Nav2 启动后 readiness gate 等到 `/clock`、`/odom`、`odom -> base_link`；发布 `/initialpose` 后 AMCL 响应，`map -> odom` 连续可查，`/controller_server` 和 `/planner_server` 均为 `active [3]`。
- 2026-06-09：P0.2 通过。使用 `/follow_path` 发送 `map` 坐标短直线路径 `(-2.0,-0.5) -> (-1.3,-0.5)`，`FollowPath` action 返回 `SUCCEEDED`，controller 日志显示 `Reached the goal!`。末态 `/odom` 约为 `x=-1.488, y=-0.500`，车辆速度接近 0。
- 备注：实测 `general_goal_checker.xy_goal_tolerance=0.08` 会被 AMCL/map 估计误差放大并触发 `Failed to make progress`，当前保留 Nav2 goal checker 基线 `0.25`；controller 内部 `FollowPath.xy_goal_tolerance` 仍为 `0.08`，并增加 terminal slowdown 以减少末端抢停/早停。
- 2026-06-10：P4.5 通过。`ForkliftMpcController` 保持 Nav2 `setPlan()`、`computeVelocityCommands()`、`setSpeedLimit()` 接口；bridge 模式下短直线 `/follow_path` 返回 `SUCCEEDED`，`NavigateToPose` 也返回 `SUCCEEDED`。`/forklift/control_cmd` 和 `/forklift/sim_cmd_vel` 均有连续输出并在末端停车。详见 `forklift_nav2_demo/docs/p4_5_nav2_controller_api_notes.md` 和 `log/p4_5_smoke/`。
- 2026-06-10：P5 通过。`ForkliftMpcController` 内部增加 path preprocessing：重复点过滤、0.10 m 插密、轻量 corner-cut smoothing、方向/曲率估计、最小转弯半径诊断和曲率限速。headless bridge 下短直线 `/follow_path`、温和大圆弧 `/follow_path`、稀疏 90 度 `/follow_path` 均返回 `SUCCEEDED`；90 度路径日志显示 `sharp_turns=1`，3 个输入点被处理成 15 个 trajectory 点。过急圆弧会打印曲率超限并限速，随后 progress checker abort，可作为后续 P6 feasibility 参考。详见 `forklift_nav2_demo/docs/p5_trajectory_preprocessing_notes.md` 和 `log/p5_smoke/`。
- 2026-06-11：P3/P4/P5 车辆几何补正通过。根据真车“双驱差速，90 度绕后轴旋转”的确认信息，继续沿 pivot-turn 方向而不是回到 Ackermann；`forklift_vehicle_model` 的 pivot predict 改为保持后轴点不动，并通过 `rear_axle_x_offset: -0.34` 表达当前 base reference 到后轴的偏移。Foxy docker 构建通过，`forklift_nav2_plugins` 相关 gtest 全部通过，headless bridge 下 `sparse_90_turn` acceptance 返回 `SUCCEEDED`。因此 P6 可以开始，但第一步建议做最小 `x/y/theta_index` lattice scaffold，并保留现有 costmap-aware A* fallback。详见 `forklift_nav2_demo/docs/foxy_pivot_turn_followup.md`。
- P6 最小 lattice scaffold 的范围说明见 `forklift_nav2_demo/docs/p6_lattice_planner_scaffold_notes.md`。
- 2026-06-11：P6.1 最小 lattice scaffold 通过。`OruGlobalPlanner` 增加可开关 `x/y/theta_index` lattice 搜索，保留原 2D A* fallback；第一版只启用 forward straight / left arc / right arc primitives，并对 primitive 沿途采样做 costmap + footprint collision。Foxy docker 构建通过，`forklift_nav2_plugins` 6 个 gtest 全部通过；headless `ComputePathToPose` smoke 中 planner server 加载 `use_lattice=true`，日志显示 `Lattice planner produced 5 states`，action 返回 `SUCCEEDED`，path heading 从起点逐步过渡到 90 度。详见 `forklift_nav2_demo/docs/p6_lattice_planner_scaffold_notes.md`。
- 2026-06-11：P6.2 lattice 第二版通过。保持 forward-only primitives 和 2D A* fallback，新增 lattice search 统计日志、primitive 拒绝原因分类、沿途 sample obstacle cost、turn cost、goal-heading heuristic，以及 `lattice_goal_tolerance` 收紧目标收尾。Foxy docker 构建通过；`forklift_nav2_plugins` 6 个测试目标全部通过，其中 `test_oru_global_planner` 扩展到 6 个用例；headless `ComputePathToPose` 90 度 smoke 返回 `SUCCEEDED`，planner 日志显示 `Lattice search succeeded: expanded=7 generated=18 accepted=18 improved=18 rejected_oob=0 rejected_costmap=0 rejected_footprint=0 best_goal_distance=0.050` 和 `Lattice planner produced 5 states`。第二版/第三版目标、步骤和解释见 `forklift_nav2_demo/docs/p6_lattice_planner_scaffold_notes.md`。
- 2026-06-12：迁移优先级调整。根据当前目标“简单 A 到 B 能跑起来，并且动态障碍物至少能安全停下”，P7 task_manager 暂缓；近期路线改为先做 P6.3 reverse primitives + direction metadata，再做 P6.4a 最小倒车执行验证，然后提前进入 P8.1 最小 safety gate。P6.4b 的倒车 acceptance 调优和 P7.1 task_manager 放在 safety gate 之后。编号保留 P7/P8，但实际执行顺序以本清单为准。
- 2026-06-12：P6.3 reverse primitives + direction metadata 通过。`OruGlobalPlanner` 的 lattice transition 现在携带 `direction`、`primitive_kind`、`length`、`heading_delta`；`lattice_reverse_enabled=true` 时会生成 `reverse straight / reverse left arc / reverse right arc`，搜索 key 在 `x/y/theta_index` 外区分 arrival direction，避免 gear-switch cost 错误合并不同到达方向；搜索会保留到达每个 state 的 primitive 元数据并记录 forward/reverse 段和 gear switch 数量；新增 `lattice_reverse_cost_multiplier` 和 `lattice_gear_switch_cost`。运行配置仍保持 `lattice_reverse_enabled: false`，把真实倒车执行留给 P6.4a。Foxy docker 构建通过；`forklift_nav2_plugins` 49 个 gtest 全部通过，其中 `test_oru_global_planner` 扩展到 8 个用例，覆盖 reverse primitive gating/metadata 和 reverse/gear-switch cost。
- 2026-06-12：P6.4a 最小倒车执行验证通过。`ForkliftMpcController` 新增 `respect_reverse_path_orientation`，在 path pose yaw 与运动切线相反时保留车体朝向并标记 `reverse_motion`；controller 只在当前 preview window 含 reverse-intent 点时允许负速度候选，避免普通 forward path 末端被倒车微调扰乱。ORU test 配置低速打开 `allow_reverse=true`、`max_reverse_velocity=0.15`、`lattice_reverse_enabled=true`。Foxy docker 构建通过；`forklift_nav2_plugins` 51 个 gtest 全部通过；headless `ComputePathToPose` 后方目标 smoke 返回 `SUCCEEDED`，planner 日志显示 `forward=0 reverse=1`；`reverse_straight` FollowPath 返回 `SUCCEEDED`，观测到 `control_direction_samples forward=0 reverse=10` 和 `/forklift/sim_cmd_vel.linear.x=-0.150`；`sparse_90_turn` 回归返回 `SUCCEEDED`，观测到 `forward=710 reverse=0`，说明 forward path 不再启用倒车候选。
- 2026-06-15：P8.1 最小 safety gate 通过。`ForkliftMpcController` 增加 controller-side safety gate，沿当前 forward/reverse 运动方向在 local costmap 上采样 footprint；障碍进入 `safety_stop_distance` 时直接 brake，进入 `safety_slowdown_distance` 时压低当前方向速度上限；`safety_emergency_stop_active` 可运行时置 true 触发 controller 停车。ORU test 配置打开 `safety_gate_enabled=true`，并保留 `sim_command_bridge` 既有 `/forklift/set_emergency_stop` 和 `command_timeout_sec` watchdog 作为 vehicle_interface 侧闸门。Foxy docker 构建通过；`forklift_nav2_plugins` 59 个 gtest 全部通过，其中新增 `test_forklift_safety_gate` 5 个用例；headless `sparse_90_turn` safety gate 回归返回 `SUCCEEDED`，日志确认 `safety_gate=true`，观测到 `control_direction_samples forward=560 reverse=0` 和 `max_linear_x=0.450`，说明无障碍时不会误停。详见 `forklift_nav2_demo/docs/p8_safety_gate_notes.md`。
- 2026-06-15：A-B acceptance 快速验证通过。Foxy docker headless + Gazebo + Nav2 + sim bridge + P8.1 safety gate 打开，从 `(-2.0, -0.5)` NavigateToPose 到 `(-0.9, -0.5)`，action 返回 `SUCCEEDED`；观测到 `feedback_count=397`、`control_samples=31`、`max_velocity_mps=0.395`、`control_direction_samples forward=31 reverse=0`、`sim_cmd_samples=100`、`max_linear_x=0.395`、末态 odom 约 `x=-1.066 y=-0.571`。planner 多次重规划均输出 forward-only lattice path，例如 `forward=5 reverse=0`、`forward=3 reverse=0`、`forward=1 reverse=0`；日志确认 `safety_gate=true safety_stop=0.550 safety_slowdown=1.250`。本次只覆盖空旷简单 A-B，不覆盖动态障碍停车/放行。
- 2026-06-15：A-B 动态障碍停车/放行快速验证通过。新增 `forklift_ab_dynamic_obstacle_acceptance` 脚本：下发 `NavigateToPose`，在路线中间生成临时 Gazebo box 障碍，观察 safety gate 停车，删除障碍后等待 Nav2 自动恢复并完成目标。`sim_command_bridge` 增加可选 `twist_fallback_topic`，在 `/forklift/control_cmd` 超时后把 recovery `/cmd_vel` 转发到 `/forklift/sim_cmd_vel`，解决 bridge 模式下 Nav2 recovery spin 只发 `/cmd_vel`、Gazebo 不动的问题；正常控制仍优先使用 `/forklift/control_cmd`。Foxy docker 构建通过；`forklift_nav2_plugins` 59 个 gtest 全部通过；headless 动态障碍验收从 `(-2.0, -0.5)` 到 `(1.2, -0.5)` 返回 `SUCCEEDED`，脚本打印 `dynamic_obstacle_acceptance=PASS`，观测到 `control_samples=399`、`sim_cmd_samples=1136`、`phase_control_zero blocked=4`、`phase_sim_max after=0.450`、`forward=94 reverse=0`。launch 日志确认 `P8.1 safety gate stopping: obstacle at 0.100 m in forward protection zone`，障碍释放后 planner 继续输出 forward-only lattice path，recovery 期间 bridge 日志出现 `Using fallback Twist command.`。
- 2026-06-15：P6.4b 倒车 acceptance 调优通过。新增 `forklift_p6_reverse_acceptance`：脚本先低频发布 `/initialpose` 并等待 `map -> base_link`，再调用 `ComputePathToPose`，从 path yaw 与几何切线推断 `forward_segments / reverse_segments / gear_switches`，需要执行的场景会把 planner path 交给 `FollowPath` 并记录 `/forklift/control_cmd` 与 `/forklift/sim_cmd_vel` 的方向样本。脚本在交给 controller 前清空 path stamps，避免 Foxy 中 ComputePath 返回旧 stamp 导致 `ForkliftMpcController could not transform the global plan`。`forklift_ab_dynamic_obstacle_acceptance` 也补入同样的 initial pose 初始化，减少外部 shell 对 `/initialpose` 的依赖。本轮验收结果：
  - `forward_straight`：`ComputePathToPose` 和 `FollowPath` 均 `SUCCEEDED`；`poses=6 forward_segments=5 reverse_segments=0 gear_switches=0`；`control_direction_samples forward=31 reverse=0`；`sim_cmd_vel` 无负速度。
  - `reverse_straight`：`ComputePathToPose` 和 `FollowPath` 均 `SUCCEEDED`；`poses=2 forward_segments=0 reverse_segments=1 gear_switches=0`；`control_direction_samples forward=0 reverse=17`；`min_signed_linear_x=-0.096`。
  - `sparse_90_turn`：`ComputePathToPose` 和 `FollowPath` 均 `SUCCEEDED`；`poses=5 forward_segments=4 reverse_segments=0 gear_switches=0`；`control_direction_samples forward=158 reverse=0`。
  - `forward_with_goal_heading`：planner-only `SUCCEEDED`；`poses=5 forward_segments=4 reverse_segments=0 gear_switches=0`，说明前方目标带终点 yaw 时不会为了贴姿态引入倒车。
  - A-B 动态障碍回归：`NavigateToPose` `SUCCEEDED`，`dynamic_obstacle_acceptance=PASS`；`control_samples=389 sim_cmd_samples=1169 forward=85 reverse=0`，说明 P8.1 动态障碍停车/放行未被 P6.4b acceptance 改动破坏。
  P6.4b 的完成边界是“当前最小 lattice scaffold 的倒车验收可回归”：普通前进路线不乱倒，后方目标会倒，controller/bridge 能执行倒车段，90 度和动态障碍回归不退化。窄通道、三点掉头、倒车入库等正式场景需要更多曲率/长度 primitive、场景生成和更强 heuristic，放到 P10/P6.5，不作为本次 P6.4b 的通过条件。
- 2026-06-15：A-B acceptance 正式验收通过。新增 `forklift_ab_acceptance` 统一入口，脚本支持 Gazebo `/set_entity_state` 重置 `forklift` 实体、重新发布 `/initialpose`、记录 `/forklift/control_cmd` 和 `/forklift/sim_cmd_vel`，并覆盖 `forward_ab`、`reverse_ab`、`dynamic_stop_release_ab`，另保留 `sparse_90_turn_ab` 作为诊断场景。两个 A-B 验收脚本默认启用 `use_sim_time=true`，避免 `/initialpose` 使用 wall time 造成 AMCL extrapolation；动态场景按 `/odom` 触发障碍生成，删除障碍后清 global/local costmap，并重发同一目标来验证放行后能继续到达。为抑制普通前进路线中不必要倒车，ORU test 配置把 `lattice_reverse_cost_multiplier` 从 `0.5` 提到 `3.0`，`lattice_gear_switch_cost` 从 `1.0` 提到 `4.0`。Foxy docker build 通过；正式验收结果：
  - `forward_ab`：`NavigateToPose` `SUCCEEDED`；`control_samples=82 sim_cmd_samples=241 forward=82 reverse=0`；`sim_cmd max_signed_linear_x=0.450`；`odom_final x=1.044 y=-0.546`；`ab_acceptance=PASS scenario=forward_ab`。
  - `reverse_ab`：`NavigateToPose` `SUCCEEDED`；`control_samples=18 sim_cmd_samples=108 forward=0 reverse=18`；`sim_cmd min_signed_linear_x=-0.096`；`odom_final x=-2.134 y=-0.497`；`ab_acceptance=PASS scenario=reverse_ab`。
  - `dynamic_stop_release_ab`：`NavigateToPose` `SUCCEEDED` after release/reissue；障碍在 `odom_x=-1.548` 时生成；`global/local costmap cleared`；`control_samples=151 sim_cmd_samples=455 forward=97 reverse=0`；`phase_control_zero blocked=54`；`odom_final x=1.049 y=-0.623`；`ab_acceptance=PASS scenario=dynamic_stop_release_ab`。
  - `sparse_90_turn_ab` 诊断：调参前失败时出现 `forward=115 reverse=223`，说明重规划会用倒车贴终点姿态；调参后不再出现倒车，`forward=257 reverse=0`，但 `NavigateToPose` 仍 `ABORTED`，末态 `odom_final x=-1.147 y=-1.325`。这不是本次 A-B 正式验收主线 blocker；后续作为 P10/P6.5 的 planner/controller/replanning 诊断项继续处理。
- 2026-06-16：根据第一版真车点位信息，很多站点需要“原地/近原地 90 度后再走”；真车已确认是双驱差速，90 度绕后轴旋转。路线调整为先补 P6.5a 上车前 rear-axle pivot primitive，再进入 P8.2 独立 safety gate。P6.5a 范围是最小 stop-pivot-go 闭环：global planner 增加可开关 `pivot_left` / `pivot_right` primitive，几何按后轴中心不动和 `rear_axle_x_offset` 计算 base pose，按 heading bin 小步旋转并沿途做 swept footprint collision；controller 识别 pivot intent，先停车/低速再发 pivot command，yaw 到位后恢复普通前进；safety gate 需要补 pivot 扫掠 footprint 检查。当前不回 Ackermann，也不急着移动 `base_link` 到后轴中心。P6.5a acceptance 至少覆盖 `pivot_90_left_in_place`、`pivot_90_right_in_place`、`pivot_90_then_forward_ab`、`pivot_blocked_stop`，并回归 `forward_ab`、`reverse_ab`、`dynamic_stop_release_ab`。
  当前结论：第一版 A-B 正式验收主线完成；下一步做 P6.5a，把真车需要的后轴近原地 90 度转向补成 planner/controller/safety 都能理解的可验收 primitive；随后进入 P8.2，把真车前所有运动命令收进独立 safety package / command gate。
- 2026-06-16（续）：P6.5a 初版代码已写入工作树（planner pivot primitive + goal-behind guard + controller stop-pivot-go + trajectory pivot 识别）。Foxy docker build 通过，`forklift_nav2_plugins` 63 个 gtest 全部通过，其中 `test_oru_global_planner` 扩展到 12 个用例，覆盖 `PivotPrimitivesAreGatedAndRotateAroundRearAxle`（pivot 几何 + rear_axle_x_offset）和 `SearchCanUsePivotPrimitiveForInPlaceGoalHeading`（lattice 搜索能用 pivot primitive 到达原地 90° 目标）。`test_forklift_mpc_trajectory` 和 `test_forklift_mpc_controller` 相关单测也通过。migration plan 同步三处修正：(1) `sparse_90_turn_ab` 从「诊断项」升为 P6.5a 硬验收门（`reverse=0 + pivot>=1 + NavigateToPose SUCCEEDED`）；(2) 新增 `l_shaped_corridor_ab` 场景（L 形过道，两腿各约 1.5 m，接近真车仓库点位，验证 replanning 过程中 pivot 不退化）；(3) 明确 v1 能力边界：`lattice_reverse_requires_goal_behind=true` 是 transit 阶段的静态占位约束，窄道三点掉头/docking 放到 P7/P10，不是 v1 目标。P6.5a 下一步：headless 仿真验证 pivot 全链路（`pivot_segments>0 → pivot_points>0 → previewHasPivotMotion=true → bridge linear.x≈0 angular.z≠0`），再跑 4 个 pivot 验收场景 + 3 个回归。
- 2026-06-16（P6.5a headless 验收 + v1 架构定调）：在 Foxy docker 跑了全部 pivot 场景，结论分三层。
  - **第一层 — pivot 链路本身通**：`sparse_90_turn_ab` 稳定 PASS（`forward=300 reverse=0 pivot=139`，`NavigateToPose SUCCEEDED`）。`reverse_ab` 回归 PASS（`forward=0 reverse=16 SUCCEEDED`）。64 个 gtest 全过。
  - **第二层 — 发现并修掉「终点航向倒车污染」**：`pivot_90_left/right_in_place`、`l_shaped_corridor_ab`、`pivot_90_then_forward_ab` 初次全 ABORTED。读 planner 日志定位：首条路径是好的前进路径、车也到达目标 xy，但在终点区为凑 `goal_yaw` 反复吐 `reverse+pivot` 微调；后轴 pivot 落点天然「在身后」→ `reversePrimitiveAllowedTowardGoal` 误判开倒车 → controller 执行微调时振荡、抖离目标。修复：guard 增加「终点 pivot regime」——`start` 距目标 ≤ `lattice_pivot_terminal_radius`(0.6m) 且航向差 ≥ `lattice_pivot_terminal_heading`(45°) 时禁 reverse、纯 pivot 凑航向；判别式天然放行同航向纯倒车（`reverse_ab` 不受影响）。新增单测 `TerminalPivotRegimeSuppressesReverseForGoalBehind`。修复后 `l_shaped` 的 controller 端 `reverse 117→0`，5 连跑那轮 pivot/in-place/sparse/reverse 全 PASS。
  - **第三层 — 残留 flaky 不是 guard 能修的**：`l_shaped_corridor_ab` 复跑时仍偶发 ABORTED（`odom_final` 被甩到对面角落 ~5m 外）。根因：当前是**最小 lattice scaffold**，车到目标区时 lattice 某些位姿 footprint 全拒 → 退化到**纯 2D A***（全向、无朝向/运动学）→ forklift controller 跟不动 → 发散。这是 scaffold 成熟度问题，不是 bug；完整 ORU planner（更丰富 primitive、更强 heuristic、运动学合法 fallback）才能让长双腿单次自主规划稳过。
  - **v1 架构定调（Option A，已决策）**：第一版**不依赖在线自主重规划**。L-shape 等路线**拆成已验证的原子段序列**（直行 FollowPath → 原地 pivot 90° → 直行 FollowPath），每段单次规划+验证（`reverse=0`、无 fallback）后执行；动态障碍**停车/离障继续，不自己重规划**（复用已验证的 P8.1 `dynamic_stop_release_ab`）；持续挡路 → 任务等待/失败，不自主绕行。完整自主规划（单个 `NavigateToPose` 跑通长双腿 + 精细终点机动）**不是上车前置**，归 **P10** 继续移植 ORU planner。
  - **本轮落地代码**：`lattice_fallback_to_astar` 在 foxy/test 两个配置都改为 `false`——v1 fail-safe，lattice 无解时抛 `PlannerException` 干净失败，绝不把全向 A* 路径交给运动学受限的 controller（叉车不会被甩飞）。planner 抛错信息同步说明该姿态。
  - **下一步**：① L-shape 路点分解执行（路点序列器，每段单次规划+FollowPath，关全局在线重规划）；② P8.2 独立 safety/command gate（含 recovery 命令白名单与扫掠 footprint，把「失败/发散」彻底变「安全停」）。`l_shaped` 长双腿单次自主规划与 A* 替代（运动学合法 fallback）归 P10。

- 2026-06-18（P8.2 独立 safety / command gate）：新增 `forklift_safety` package 和 `safety_command_gate` 节点，正式把运动命令链路改成 `controller/task/manual -> /forklift/control_cmd_raw -> safety gate -> /forklift/control_cmd -> vehicle_interface/sim bridge`。第一版 gate 覆盖 command watchdog、急停服务 `/forklift_safety/set_emergency_stop`、vehicle/fault/localization 可选健康检查、速度/转角限幅；同时接入 recovery command adapter，订阅 Nav2 `/cmd_vel` 后只白名单低速 wait/backoff/pivot，并转换成受限 `ForkliftControlCommand`，不再让 bridge 裸吃 `/cmd_vel`。`forklift_navigation.launch.py` 默认启动 safety gate，`bridge_twist_fallback_topic` 默认置空；ORU test 配置把 controller 输出 topic 改为 `/forklift/control_cmd_raw`，保留 controller-side P8.1 safety gate 作为回退保护。
- 2026-06-18（P8.2 补完 8.2-4/5/6）：`safety_command_gate` 默认订阅 `/local_costmap/costmap`，支持可选 `costmap_message_type:=costmap_raw`；costmap 缺失、超时或空/截断/无效数据会输出停车命令并在 `/forklift/safety_gate/status` 说明原因。gate 内新增独立 swept footprint 复核：按 Foxy ORU local footprint 参数解析 footprint，沿当前位姿和短时预测位姿采样 footprint 边界，遇到 unknown/out-of-map/lethal cost 即停，覆盖 raw command 和 recovery wait/backoff/pivot。新增 `forklift_safety/P8_2_FOXY_ACCEPTANCE.md` 作为 Foxy docker 可复现验收记录；Foxy docker build 6 packages 通过，`./scripts/foxy_colcon_test.sh` 通过（83 tests / 0 failures，其中 `forklift_safety` 14 个 pytest）。
- 2026-06-18（P8.2 drive 限幅修正）：发现 gate 原先只限 `velocity_mps`，而真车 Curtis 0x203 驱动帧实际用 `drive_rpm`（0..4000）做速度，限速会被绕过。`safety_command_gate` 新增 `apply_drive_envelope`：对所有下发的运动命令（raw + recovery）把 `|drive_rpm|` 限到 `max_drive_rpm`（默认 2500，厂家常用工作转速），并把 `accel_time_sec`/`decel_time_sec` 在上游未给时补成厂家建议值（默认 5s / 3s）；新增对应 launch 参数与单测（`forklift_safety` 升到 15 pytest，Foxy docker 内全过）。velocity→rpm 的换算仍必须放在 gate 下游 `curtis_vehicle_interface`，归 P2.3。
- 2026-06-18（P8.2 收紧 costmap 默认 + /odom 文档化）：`safety_command_gate` footprint 碰撞检查默认改为更严档——订阅 `/local_costmap/costmap_raw`（`costmap_message_type: costmap_raw`，原始 0–254 刻度）、阈值 `footprint_collision_cost_threshold: 253`，即压到 inscribed/lethal/unknown 就停，比原 OccupancyGrid+100 更早停更保守；要放宽切回 `occupancy_grid` + `/local_costmap/costmap` + 100。同时在 `real_vehicle_tuning_guide.md` 新增 §7.1 说明独立命令安全闸,并在 §8 bring-up 清单加第 7 条:真车必须确认 `/odom` 与 `/local_costmap/costmap_raw` 真进 gate（否则 footprint 检查 fail-closed 永远停车），验证用 `ros2 topic echo /forklift/safety_gate/status` 看是否出现 `collision pose missing` / `costmap missing`。

## 14. ORU 包迁移优先级

优先移植：

```text
1. orunav_mpc
2. orunav_trajectory_processor
3. orunav_path_smoother
4. orunav_motion_planner
5. orunav_constraint_extract
```

暂时不移植，只参考：

```text
orunav_vehicle_execution
orunav_coordinator_fake
orunav_rviz
orunav_debug
orunav_pallet_detection_sdf
```

理由：

- `orunav_vehicle_execution` 是 ORU 自己的任务执行框架，和我们未来的 `forklift_task_manager + Nav2 actions` 职责重叠。
- `orunav_mpc` 的控制数学最有价值。
- `orunav_motion_planner` 的 primitive/lattice 思路很有价值，但要等 controller 和 vehicle interface 稳定后再做。

P8.4 / P7.1 之后，ORU 核心算法还剩这些主要部分：

```text
1. 完整 ORU QP-MPC
2. 更完整的 ORU motion planner / primitives / lookup tables
3. trajectory processor / path smoother
4. constraint_extract / 几何约束提取
5. ORU debug / rviz / execution 框架只参考，不作为主线迁移
```

完整 ORU QP-MPC：

- 当前 `ForkliftMpcController` 已有 ORU 的 State / Control 概念、trajectory preview、path preprocessing 和 sampled predictive controller scaffold。
- 当前还不是完整 ORU `qpProblem / qpConstraints / qpOASES`。
- 是否迁移完整 QP-MPC，取决于 sampled controller 在真车低速场景是否足够稳定。
- 如果低速 A-B、倒车、换向、末端停车已经满足需求，可以继续保留 sampled controller。
- 如果速度提高、路径更复杂或末端控制不够顺，再迁移 ORU QP-MPC。

更完整的 ORU motion planner：

- 当前 P6 已经实现 ORU-inspired lattice：`x/y/theta_index`、forward/reverse primitives、gear switch cost、footprint collision。
- 还不是完整 ORU motion planner。
- 后续可补 ORU primitive 文件加载、lookup table、multi-curvature / multi-length primitives、更强 heuristic、lookup/cache 加速。
- 重点验收窄通道、倒车、换向次数、贴边距离和路径稳定性。

trajectory processor / path smoother：

- 当前 controller 内部已有轻量 path preprocessing。
- 后续如果要靠近 ORU，应把路径转轨迹、方向语义、曲率、速度约束和平滑从 controller 内部逐步抽出来。
- `orunav_path_smoother` 的约束优化思路有价值，但 ACADO 依赖成本高，不作为第一步硬移植目标。

constraint_extract / 几何约束：

- 可用于从 costmap 提取局部可行 corridor。
- 可用于更严格的 footprint、叉臂、车尾碰撞检查。
- 可给 smoother 或 MPC 提供边界约束，让路径不只是“不撞”，而是保持合理障碍余量。

`iliad/` 当前处理策略：

- `iliad/` 暂时不移植。
- `iliad_human_aware_navigation`、`iliad_hrsi`、`iliad_safety_metrics` 更偏 human-aware navigation、HRSI、人车交互和安全指标。
- `moving_actor_gazebo`、`gazebo_plugin_actor_collision`、`iliad_base_simulation` 更偏 ROS1/Gazebo actor 仿真环境。
- `iliad_goal_manager`、`iliad_init_pose_manager` 和未来 `forklift_task_manager + Nav2 actions` 有部分职责重叠，不作为当前主线入口。
- 只有当后续明确要做人群附近通行策略、人车交互安全指标或动态人行为预测时，再从 `iliad/` 里选模块参考或局部迁移。

### 和「真实完整 ORU」的差距速查

前提：P10 做的是 **clean-room ORU-style**，**不是直接拷上游 `navigation_oru` 源码**——本地那份 LICENSE 是 `CC BY-NC-SA 4.0`（非商业），不能直接用，所以按其思路重写。因此“差距”指**能力差距**，不是“还没 copy 完”。

当前已有：P10 Phase 0–4 落地的 clean-room ORU lattice **核心**（16 heading、forward/reverse/pivot primitive、swept footprint 碰撞、hybrid heuristic、ARA* 搜索）；控制器是 **sampled predictive controller**。距完整 ORU 还差五块：

| ORU 组件 | 现状 | 差距 | 计划 |
|---|---|---|---|
| **motion planner** | clean-room lattice 核心（Phase 0–4），但 Phase 5 还没过门；长双腿 L 形单次自主规划仍偶发发散 | 更丰富 primitive 集（多曲率/多长度）、primitive 生成器、lookup table/cache、更成熟搜索 | **P10 Phase 1–6**（进行中） |
| **QP-MPC 控制器** | 有 ORU State/Control 概念、preview、预处理的**采样式**控制器 | 不是完整 ORU `qpProblem/qpConstraints/qpOASES` | **P12**，且**仅当**采样控制器在真车低速不够稳才上 |
| **trajectory processor / path smoother** | controller 内部轻量预处理 | 完整 `orunav_trajectory_processor` + `orunav_path_smoother`（ACADO 约束优化） | **P11**，ACADO 依赖重，不作第一步 |
| **constraint_extract / 几何约束** | 无 | 从 costmap 提可行 corridor、更严格叉臂/车尾碰撞、给 smoother/MPC 边界约束（保持障碍余量而不只是“不撞”） | **P11** |
| vehicle_execution / coordinator / rviz / debug / iliad human-aware | 不移 | ORU 自己的任务执行/多车协调/调试，与我们 task_manager+Nav2 职责重叠；单车 v1 不需要 | 只参考，不移 |

**对 v1 的实际影响**：差的这些**都不是 v1 上车前置**。v1 已定调走 Option A（路点分解，不靠在线自主规划），所以 planner 完整度不足用“已验证原子段”绕开，QP-MPC / smoother / constraint_extract 全部 P11/P12、低速够用就不上。即：离“完整 ORU”还有 **P10（planner）+ P11（smoother/约束）+ P12（QP-MPC）** 三大步；离“v1 上车”只差上车前那几个硬门。完整 ORU 是“把第一版做扎实之后的事”。

P8.4 / P7.1 之后的建议 ORU 后续顺序：

```text
P10  motion planner lookup/primitives 加强
P11  path smoother + constraint_extract
P12  评估是否上完整 ORU QP-MPC
P13  需要 human-aware 能力时，再评估 ILIAD 相关包
```

## 15. P10: ORU 规划器完整移植（替换 lattice scaffold）

> 2026-06-16 立项。背景：当前 `oru_global_planner` 是**最小 lattice scaffold**（手写少量
> primitive + 弱 heuristic + 退化 2D-A* fallback），monolithic `l_shaped_corridor_ab`
> 单次自主规划会发散（详见 §13 执行日志「第三层」）。结论：让长双腿 L 形单次自主规划稳过，
> 需要把 ORU 真正的 state-lattice 规划器核心移过来。本章是给执行方（Codex）的正式分阶段计划。
> 路点分解（drive→pivot→drive 顺序执行）作为另一条 v1 路线已评估，本轮暂缓，优先 ORU 核心移植。

### P10.0 scope —— 移什么 / 不移什么

只移**规划器核心**，其余保留：

| ORU 组件 | 移不移 | 原因 |
|---|---|---|
| `orunav_motion_planner`（state-lattice + primitives + heuristic + ARA* 搜索） | 移（核心） | 补 scaffold 两个短板：丰富 primitive 集 + 强 heuristic |
| primitive 生成器 + 生成的 primitive 文件 | 移 | scaffold 弱在手写 primitive 太少 |
| `orunav_constraint_extract` / `path_smoother` | 可选（后期 P11） | 让路径更顺，v1 可先不要 |
| `orunav_mpc` | 不移 | 已有 `ForkliftMpcController` 当跟踪器 |
| `orunav_coordinator`（多车协调） | 不移 | 单车 v1 不需要 |

- **保留不动**：`ForkliftMpcController`（控制器）、Nav2 costmap/TF/BT、现有验收门
  （`forward_ab`/`sparse_90_turn_ab`/`pivot_90_*`/`l_shaped_corridor_ab`）。
- **替换**：`oru_global_planner`（最小 scaffold）→ 包成新插件接 ORU 核心。
- 分工：ORU 规划器是 plan-once，出一条运动学合法路径；跟踪交给 `ForkliftMpcController`。

### P10.1（Phase 0）vendor 并隔离规划器核心，先单独编译
- 从上游 `navigation_oru` 取 `orunav_motion_planner` 的纯 C++ 核心（World/占据表示、
  VehicleModel、primitive 加载器、heuristic、ARA* 搜索），放进新包 `forklift_oru_planner/vendor/`。
- 剥掉 ROS1 依赖（`ros/ros.h`、tf、roscpp 消息）；核心基本 ROS 无关。
- 普通 CMake 编静态库，先在 foxy docker 工具链跑通编译，不接 Nav2。
- **先确认上游 repo 与 license**（BSD/LGPL 系，核实并保留 LICENSE/出处）。
- 验收：vendor 库 `colcon build` 通过，带最小 main 能 load primitives + 跑一次搜索。

### P10.2（Phase 1）为本车生成 primitive 集
- 用 ORU primitive 生成器按本车运动学产出：轴距、`max_steering`、后轴 pivot
  （对应 `rear_axle_x_offset=-0.34`、`pivot_turn_radius=0.6`）、允许倒车、heading 离散（建议 16）。
- 提交生成的 primitive 文件到 repo。
- 验收：单测加载 primitive，断言存在 pivot 原语、reverse 原语、各 heading 前进原语，
  几何与现有 `pivot_steering_angle≈π/2` 一致。

### P10.3（Phase 2）costmap 碰撞适配器
- adapter 把 ORU 世界查询映射到 Nav2 `costmap_2d`（inflation 层）：对每条 primitive 的
  扫掠 footprint 做碰撞检查，定 lethal 阈值/分辨率。
- 验收：已知障碍图上单测「穿障 primitive 被拒、空地 primitive 通过」。

### P10.4（Phase 3）heuristic（治本一步）
- 启用 ORU 混合 heuristic：nonholonomic-without-obstacles（离线预计算查找表）+
  holonomic-with-obstacles（对 costmap 跑 Dijkstra）取大。
- 这正是 scaffold「到目标区 thrash」的根因解。
- 验收：对比开/关 heuristic 的搜索扩展节点数与终点收敛性。

### P10.5（Phase 4）包成 Nav2 GlobalPlanner 插件
- 实现 `nav2_core::GlobalPlanner`（`configure/activate/createPlan`）：start/goal pose →
  建 ORU mission → ARA* 搜索 → primitive 路径转 `nav_msgs/Path`（稠密带朝向）→ 返回。
- **goal yaw 当硬约束**（lattice 终点 state 含 heading），终点 pivot 由规划器原生给出，
  不再靠 terminal-pivot guard 补丁。
- 失败 `throw nav2_core::PlannerException`（沿用 v1 fail-safe，不退化到 2D A*）。
- 验收：插件被 Nav2 加载，单次 `createPlan` 对 L 形给出含终点 pivot 的运动学合法路径。

### P10.6（Phase 5）集成、切换、调参（monolithic L 形应在此一把过）
- 配置里把 `GridBased` 从 `oru_global_planner`（scaffold）换成新插件。
- 跑全部现有门：`forward_ab`/`sparse_90_turn_ab`/`pivot_90_*`/`l_shaped_corridor_ab`
  （monolithic 单次 NavigateToPose）。
- 调 primitive 代价（reverse 倍率、换挡代价）与 heuristic 权重。
- **核心验收：`l_shaped_corridor_ab` 单次自主规划 3 连跑全 PASS**
  （reverse=0 / pivot≥1 / SUCCEEDED / 末端 y≥0.6）—— scaffold 做不到、ORU 该解决的目标。

### P10.7（Phase 6，可选/后期）port constraint-extract + smoother
- 让路径更顺、更易跟踪；控制器仍是 `ForkliftMpcController`。归并到 P11。

### P10 关键提醒（给执行方）
- 接口缝在 Phase 2/4（costmap 适配、pose↔lattice state 转换），bug 多发，先写单测。
- 坐标/单位：ORU 内部用自己的世界系，注意与 map 系、分辨率、角度离散的换算。
- 不要动控制器：ORU 出路径即可，跟踪交给现有 `ForkliftMpcController`。
- 现有 terminal-pivot guard、`lattice_fallback_to_astar=false` 是 scaffold 的补丁，
  ORU 上来后大概率可移除——但先留着，等 Phase 5 验证后再删。
- 风险/工期：数周级运动学规划器移植，主要不确定性在 primitive 生成与 heuristic 调参；
  Phase 0/1 跑通即说明可行，没跑通要尽早暴露。

### P10 执行记录（2026-06-16）

本轮按 Phase 0→5 顺序推进，先落地一个 ROS/Nav2-free 的 ORU-style lattice core，再把
`OruGlobalPlanner` 从原先最小 scaffold 改成调用 core。注意：本轮没有直接拷贝上游
`navigation_oru` 源码。原因是本地 `navigation_oru-release/LICENSE` 对非 ILIAD H2020
参与者是 `CC BY-NC-SA 4.0`，与当前仓库工程落地不匹配；因此只保留 license/出处说明，
实现采用 clean-room ORU-style core。

Phase 0 gate：

- 新增 `forklift_oru_planner` 包，`vendor/` 下提供纯 C++17 静态库 `forklift_oru_lattice_core`。
- core 不依赖 ROS、Nav2、tf 或 costmap 消息，只通过 `GridAdapter` 回调查询栅格/footprint/cost。
- `vendor/README.md` 记录本地上游来源、license 核查结论和 clean-room 决策。
- 验收：Foxy docker `colcon build --packages-select forklift_oru_planner` 通过；core 单测能生成/加载 primitive 并跑一次搜索。

Phase 1 gate：

- 提交 `forklift_oru_planner/primitives/forklift_16_heading.mprim`。
- 参数采用 16 headings、`rear_axle_x_offset=-0.34`、`pivot_turn_radius=0.6`、允许 reverse/pivot。
- primitive 集包含 forward、reverse、left/right arc、`pivot_left`、`pivot_right`。
- 验收：`test_oru_lattice_core` 覆盖 forward/reverse/pivot primitive 存在性和后轴 pivot 几何。

Phase 2 gate：

- `GridAdapter` 支持 `cell_traversable`、`footprint_traversable`、`normalized_cost` 三个回调。
- core 在 primitive rollout 的每个 sample 上做 cell + swept footprint 检查，并记录
  `OUT_OF_BOUNDS`、`COSTMAP_BLOCKED`、`FOOTPRINT_BLOCKED` reject reason。
- Nav2 插件桥接到 `costmap_2d`，沿用 lethal threshold、unknown policy、footprint sweep。
- 验收：core 单测覆盖障碍图上穿障 primitive 被拒、空地 primitive 通过。

Phase 3 gate：

- core heuristic 为 nonholonomic-without-obstacles + holonomic-with-obstacles Dijkstra 取大值。
- holonomic heuristic 基于 costmap traversability 和 normalized cost 预计算到目标的 8 邻接 Dijkstra。
- 验收：单测对比开/关 obstacle heuristic，启用 hybrid heuristic 后搜索仍收敛并减少无效扩展。

Phase 4 gate：

- `forklift_nav2_plugins::OruGlobalPlanner` 已改为在 `searchLattice()` 中构建
  `forklift_oru_planner::LatticeCore`，再把 core states/transitions 转回现有 path/diagnostics。
- `forklift_nav2_plugins` 依赖 `forklift_oru_planner`，Foxy build/test 脚本同步加入新包。
- 保持 hard goal yaw、`lattice_fallback_to_astar=false` 的 fail-safe 策略：core 无 kinodynamic path 时抛
  `nav2_core::PlannerException`，不把全向 2D A* 路径交给 forklift controller。
- 验收：`forklift_oru_planner` + `forklift_nav2_plugins` Foxy docker build 通过；总计 70 个 gtest 通过。

Phase 5 gate：

- 配置仍使用 `GridBased` / `forklift_nav2_plugins/OruGlobalPlanner`，实现已切到新 core。
- 已新增 `forklift_nav2_demo/behavior_trees/forklift_oru_plan_once_with_recovery.xml`，并在
  `forklift_nav2_plugins` 中加入 BT 插件：`ForkliftPlanOnceSequence` 只允许
  `ComputePathToPose` 成功 tick 一次，随后保持 `FollowPath` RUNNING，解决 Foxy 默认
  `PipelineSequence` 持续重规划导致的 FollowPath 被取消问题。
- `forklift_navigation.launch.py` 已支持 `autostart` 参数，并从所选 YAML 读取
  `bt_navigator.default_bt_xml_filename` 后传给 `nav2_bringup`，避免 Foxy wrapper launch
  覆盖为默认 BT。
- `ForkliftMpcController` 已改为使用 latest TF 查询 path pose，并修复 pivot preview window：
  pivot 执行中选择预览窗口内最后一个 pivot target，避免围绕中间 pivot 点反复变号。
- `forward_ab` 已实跑通过：`/navigate_to_pose status: 4 SUCCEEDED`，
  `control_samples=96 sim_cmd_samples=278 forward=96 reverse=0 pivot=0`，
  `odom_final x=1.239 y=-0.288`，`ab_acceptance=PASS scenario=forward_ab`。
- 为终端原地转向加入直接 rear-axle pivot path：当起终点后轴点在容差内重合、heading
  delta 足够大且 footprint sample 全可通行时，planner 直接生成终端 pivot 路径。
- `pivot_90_right_in_place` 已跑到 Nav2 成功并产生纯仿真 pivot：
  `status: 4 SUCCEEDED`，`control_samples=71 sim_cmd_samples=196 forward=71 reverse=0 pivot=71`，
  `sim_cmd max_signed_linear_x=0.000 max_abs_angular_z=0.200`，
  `odom_final x=-2.245 y=-0.837`。这一轮失败点不是 Nav2 执行，而是验收脚本把
  `forward=true && steering≈±pi/2` 的 pivot command 同时计入 forward samples。
- `forklift_ab_acceptance.py` 已修正计数顺序：先识别 pivot command，pivot 不再累计到
  `forward_control_samples`。修正后尚未完成一次干净复跑，因此 `pivot_90_right_in_place`
  gate 目前仍标记为未最终通过。
- 试过把 `pivot_velocity` 从 `0.12` 提到 `0.30`，结论是会导致过冲/进展失败并触发 forward
  fallback；已回退到 `0.12`，后续不应沿这条方向继续调。

当前结论：

- Phase 0→4 已通过代码和单测验收，并已提交为
  `974a90d Implement P10 ORU lattice core migration`。
- Phase 5 已解决两个主要运行态问题：Foxy BT 反复重规划问题、pivot 跟踪目标点变号问题；
  `forward_ab` 已 PASS。
- Phase 5 还没有最终过门。当前最新状态是：`pivot_90_right_in_place` 在 Nav2/controller
  层已经能成功完成 pivot，但验收脚本修正后还缺一次干净复跑确认；其它场景还未完成连续验收。

剩余问题 / 复盘清单：

- 需要在修正后的 acceptance 脚本上复跑 `pivot_90_right_in_place`，确认不再因为 pivot 被误计为
  forward 而失败。
- 需要继续跑 `pivot_90_left_in_place`、`sparse_90_turn_ab`、`pivot_90_then_forward_ab`，
  最后跑核心门 `l_shaped_corridor_ab` 3 连 PASS。
- ~~Foxy `recoveries_server` 在 lifecycle configure 阶段仍有不稳定崩溃：
  `failed to send response...`。当前工作方法是 `autostart:=false` 启动……~~
  **已定位并解决（2026-06-17）**：根因不是 `recoveries_server`，而是 **FastRTPS** 在
  `autostart` 并发 `configure/activate` 服务调用里的竞态——崩的是随机 lifecycle 节点
  （实测命中 `amcl`、`bt_navigator`、`recoveries_server`）。改用 **CycloneDDS** 后消失：
  FastRTPS 2/22 崩溃 vs CycloneDDS 0/34，且 `autostart:=true` 完整拉起 + `forward_ab`
  验收 PASS。镜像默认 RMW 已切到 `rmw_cyclonedds_cpp`（`docker/foxy/Dockerfile`），
  不再需要 `autostart:=false` 手动 bringup 规避。复现脚本 `scripts/recoveries_bringup_campaign.sh`，
  实车说明见 `real_vehicle_tuning_guide.md` §10。
- 需要在最新 BT 插件、controller、planner、acceptance 脚本和配置修改后重跑
  `forklift_oru_planner` / `forklift_nav2_plugins` gtest，再跑完整
  `./scripts/foxy_colcon_test.sh`。
- `forklift_nav2_demo/docs/lattice_planner_design.docx` 是本轮发现的未跟踪文件，当前不纳入
  P10 代码提交，避免把无关二进制文档混进最终 commit。
- 最终完成 Phase 5 后，还需要补一次文档结论、整理未跟踪/待提交文件，并提交第二个 commit。
