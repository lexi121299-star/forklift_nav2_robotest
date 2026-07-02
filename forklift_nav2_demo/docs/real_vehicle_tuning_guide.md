# 叉车实车测试参数调节指南 (Real-Vehicle Tuning Guide)

面向 ROS2 Foxy + Nav2 的叉车导航栈。本文把"现场看到的现象 → 该改哪个参数 → 在哪个文件第几行 → 往哪个方向调 → 副作用"串起来,方便上车时快速调参。

- **唯一权威配置(实车 / Foxy):`forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml`** —— 现在是 `config/` 下唯一的 Nav2 参数档,launch 默认值也已指向它。
- 原来的 Humble / 原生并行档(`forklift_nav2_oru_test.yaml`、stock 基线 `forklift_nav2.yaml`,使用 `goal_checker_plugins` 复数、`DifferentialMotionModel` 等 Foxy 加载不了的 API)**已删除**,避免 Foxy/Humble 混淆;以后一律按 Foxy 那份改参数。历史阶段笔记里若仍引用这两个文件名,只作历史记录看。
- 控制器源码(部分行为只在代码里,见各节标注):`forklift_nav2_plugins/src/forklift_mpc_controller.cpp`

> **构建 / 测试 / 运行环境**:**一律在 Foxy docker(`forklift-nav2:foxy`)里进行**,不在宿主机原生环境编译或跑验收。构建 `colcon build --build-base build_foxy --install-base install_foxy --symlink-install`;测试 `./scripts/foxy_colcon_test.sh`。

> **生效方式**:这些都是 yaml 参数,**改完不用 colcon 编译**,但 costmap / 控制器在 `configure` 时读参数,所以要**重启导航栈**(或对应节点 deactivate→cleanup→configure→activate)才生效。控制器逻辑改动(`.cpp`)才需要 `colcon build`(在 Foxy docker 内)。

> **实车 vs 仿真**:实车只使用 `foxy-real` 分支的 `scripts/start_foxy_real.sh`，不要复用会启动 Gazebo 的仿真总入口。一切运动走安全链路 `/forklift/control_cmd_raw → safety_command_gate → /forklift/control_cmd → Curtis CAN`。详见第 8 节。

> **DDS 中间件**:镜像默认已切到 **CycloneDDS**(Foxy FastRTPS 在 `autostart` 启动时会间歇性崩掉随机 lifecycle 节点)。Docker 怎么改、ARM 注意事项、回退方式见 **第 10 节**——上车前必读。

---

## 1. 症状 → 参数 速查表

| 现场现象 | 先调这个 | 文件:行 | 方向 |
|---|---|---|---|
| **走廊/路口看着够宽但规划说过不去、贴边不敢走**（costmap 边界太大） | `footprint`（local 与 global 要一致且贴合真车） | 本文件 §2.1 | 缩小到真车实际尺寸 |
| 同上，离墙总是留太大空隙 | `inflation_radius` / `cost_scaling_factor` | §2.2 / §2.3 | 减小 radius / 增大 scaling |
| 直角转弯转不过去、原地磨 | `lattice_arc_radius` / pivot 系列 | §3 | 减小转弯半径、确认 pivot 打开 |
| 障碍物没进全局路径、不绕 | global `obstacle_layer` + 感知距离 | §4 | 已加层；放大 `obstacle_max_range` |
| 车太快/太慢、起步顿挫 | `max_velocity` / 加速度 / 平滑 | §5 | 按场地调 |
| 到点附近反复修正、飘 | goal 容差 + 终点锁存 latch | §6 | 放宽容差 / 调 latch |
| 离障碍物多远开始减速/急停 | safety gate 距离 | §7 | 按车速与制动距离 |
| **安全闸误停 / 明明没障碍却不动**（要临时关闸先跑通） | `safety_enabled` 等 launch 参数 | §7.2 | 现场临时关、真车保持全开 |

---

## 1.5 坐标系、base_link 与真车标定流程（footprint 之前必读）

调 footprint / 偏移 / 转弯之前,先把坐标基准定死。否则那些数值没有参照、全是错的。

### 1.5.1 坐标轴约定（ROS REP-103）

- **+x = 前进**（配重端）,**−x = 后退**（货叉端）
- **+y = 左**,−y = 右；**+z = 上**
- **yaw 绕 +z 右手旋转**:yaw=0 时车头(+x)对齐 map 的 +x;**yaw 增大 = 逆时针 = 左转**

### 1.5.2 base_link 在哪——是"基准原点",不是"几何中心"

`base_link` 不是"车的中心"这个物理概念,而是**整个系统的坐标基准点**:footprint、激光 TF、里程计、`rear_axle_x_offset` **全部相对它**来量。你**自己定它在哪**,然后一切按它对齐。

> **真车推荐:把 base_link 定在后驱动/转向轴中心。** 这样 `rear_axle_x_offset≈0`,运动学最干净。

**仿真模型当前的几何**（diff-drive,**真车不可照搬**,见 [§1.5.4]）——各部件相对 base_link 原点的 x（m）:

```
   货叉端(−x,后)                      配重端(+x,前)
 货叉尖   货叉根  桅杆   驱动轴   原点   激光 脚轮 配重  车头
 −2.043  −1.443 −0.843  −0.34    0    +0.25 +0.42 +0.73 +0.843
   │        │     │       │       ●      │    │    │     │
   └──footprint 后边界          base_link            footprint 前边界┘

       ↑+y(左)
        │
  ──────●──────→ +x(前/配重)        yaw↺ 逆时针为正
        │   base_link(此处=车身箱中心)
       ↓−y(右)
```

注意仿真里 base_link 在**车身箱几何中心**、驱动轴在 −0.34（故 `rear_axle_x_offset:-0.34`）。footprint `[[0.843,0.58],[0.843,-0.58],[-2.043,-0.58],[-2.043,0.58]]` = 前 +0.843 / 后 −2.043（货叉尖）/ 半宽 ±0.58,**原点不在中间**（货叉往 −x 伸很长）。

- `base_footprint` 是地面投影帧,**也是导航/AMCL/odom 的基准帧**（`base_frame_id: base_footprint`）;它与 `base_link` 的 **x/y 相同**,只差一个 z（车身高）。谈 footprint 时两者等价。

### 1.5.3 真车标定顺序（定了 base_link 之后照着填）

> 顺序不能反——base_link 没定死之前,footprint 和偏移都没有意义。

| 步骤 | 做什么 | 改哪里 |
|---|---|---|
| **0. 定 base_link** | 放后轴中心,定了别动 | （仅约定，下面全相对它） |
| **1. odom 对齐** | Curtis CAN / 编码器算出的位姿**必须是 base_link(后轴)这个点的**。参考点不一致 → AMCL 一动就发散。**最易忽略、最致命。** | 里程计节点（真车 odom 源） |
| **2. 激光 TF** | 量激光相对后轴的安装位置,填进 `base_link→base_scan` 的 `origin xyz/rpy`。装反/量错 = scan 对不上地图 = 发散。**必须现场实测。** | URDF / 或 static TF（见 §1.5.4） |
| **3. footprint** | 以后轴为原点量真车轮廓,前+x/后−x/半宽±y | local_costmap [`:201`](../config/forklift_nav2_oru_test_foxy.yaml#L201)（安全闸自动同步）+ global_costmap [`:255`](../config/forklift_nav2_oru_test_foxy.yaml#L255) |
| **4. 运动学** | `rear_axle_x_offset`=后轴相对 base_link 的 x（base_link 在后轴则填 **0**）；`wheel_base`/转向角/最小转弯半径按真车 | controller [`:132`](../config/forklift_nav2_oru_test_foxy.yaml#L132) + lattice [`:372`](../config/forklift_nav2_oru_test_foxy.yaml#L372)；转弯见 §3 |
| **5. 静态验证** | 摆已知点 `set_initial_pose` → RViz 看 **scan 是否贴合地图墙线**（验 base_link+激光 TF）；最低速点动确认 **前进=+x、左转=yaw 增大** | 不开车先做，过了再进 §5/§3/§7 |

### 1.5.4 上真车还需要 URDF 吗？

**不是必须有完整 URDF,但你必须提供它负责的那几个 TF。** URDF 在本导航栈里只干三件事,其中真车只关心第 1 件:

1. **发布刚体 TF 树**（经 `robot_state_publisher`）:`base_footprint→base_link→base_scan`。**AMCL/Nav2 真正消费的是这几个 TF**——尤其 `base_link→base_scan`（激光安装位置）。**这部分真车必须有。**
2. RViz 里显示车模型——纯好看,可有可无。
3. Gazebo 仿真插件（`<gazebo>` 段里的 diff_drive、ray 激光等）——**纯仿真,真车完全不用,要删掉。**

> 其余 TF 不归 URDF:`odom→base_footprint` 由**里程计源**发,`map→odom` 由 **AMCL** 发。URDF 只补 base_footprint 以下的刚体部分。

**两个选择:**

- **方案 A（推荐,标准做法）**:留一份**精简 URDF**——保留 `base_footprint`/`base_link`/`base_scan` 等 link 与 joint,**删掉所有 `<gazebo>` 段**,跑 `robot_state_publisher`。好维护、RViz 还能看模型,以后加传感器也方便。
- **方案 B（最小化）**:**完全不用 URDF**,在 launch 里用 `tf2_ros static_transform_publisher` 直接发那 2–3 个固定 TF（base_footprint→base_link、base_link→base_scan）。最省,但没车模型、传感器一多就难管。

只挂一个激光的简单车,方案 B 够用;但**方案 A 更标准、更省心**,激光 TF 写在 URDF 里和 footprint 标定也对得上。

### 1.5.5 真车改 TF 速查

**只手改 base_footprint 以下的刚体静态 TF**（车身高 + 传感器安装位置）。`map→odom`（AMCL 发）和 `odom→base_footprint`（里程计源发）不在这里改。

```
map ─(AMCL)→ odom ─(里程计源)→ base_footprint ─┬→ base_link ─→ base_scan(激光)
 不碰            不碰(算错是odom问题)            └→ 其它传感器
                                              ↑──── 这一段才是"改TF" ────↑
```

**改在哪**

- 方式 A（URDF,推荐）:改对应 joint 的 `<origin>`——
  - 车身高 `base_footprint→base_link`:[base_joint :15](../urdf/forklift_diff_drive.urdf.xacro#L15) `xyz="0 0 后轴离地高"`
  - 激光位置 `base_link→base_scan`:[base_scan_joint :212](../urdf/forklift_diff_drive.urdf.xacro#L212) `xyz="前后 左右 高" rpy="roll pitch yaw"`（米 / 弧度）
- 方式 B（无 URDF）:launch 里 `tf2_ros static_transform_publisher`,参数序 **`x y z yaw pitch roll`(先 yaw,和 URDF 的 rpy 相反——常见填错点)**。

**必须盯的 6 点**

1. **帧名对齐激光驱动**:URDF 里激光 link 名要 = `/scan` 的 `header.frame_id`（常见 `laser`/`laser_link`）,否则 AMCL 连不上、costmap 没激光。
2. 以 base_link（后轴）为原点、REP-103:+x 前 / **+y 左**（最易反）/ +z 上,米。
3. **激光朝向 rpy 最关键**:0° 没朝正前或倒装要写进 rpy;**yaw 错一点 = 整片 scan 旋转 = 像定位漂移**,倒装 = `roll=π`。AMCL 发散头号元凶。
4. **真车 `use_sim_time:=false`**:否则 TF 时间戳对不上,报 "extrapolation into the future",全栈不动。
5. **同一条边只能一个发布者**:别 URDF 和 static_transform_publisher 同时发同一段。
6. z 高度对 2D 导航基本不影响,但仍按实际填。

**改完怎么验（开车前）**

```bash
ros2 run tf2_tools view_frames                 # 看树连通、单根
ros2 run tf2_ros tf2_echo base_link base_scan  # 数值对不对得上尺子
```
RViz:Fixed Frame=map + LaserScan,**看点云贴不贴墙线**。贴=对;错位=回查第 2/3 点。

### 1.5.6 分工 / 接口约定（谁负责哪段）

| 这段 | 谁负责 |
|---|---|
| 激光/传感器 TF 标定、`map→odom`（AMCL）、`odom→base_footprint`（里程计） | **定位 / 视觉同事** |
| footprint、`rear_axle_x_offset`、运动学/转弯/控制参数 | **导航 / 控制（你）** |

**两边的唯一接口契约 = base_link 的定义。** 你要跟定位同事讲清、并**互相确认**:

1. **原点 = 后驱动/转向轴中心,+x 前、+y 左**（告诉他们这一句是主干）。
2. ⚠️ **关键确认**:他们发的**里程计必须以这个后轴点为参考**。若他们 odom 实际算的是别的点,AMCL 会发散,现象像"定位坏了"其实是**原点不一致**——所以不能只通知,要确认一致。
3. 即便 TF 归他们,**footprint 按后轴量、`rear_axle_x_offset=0` 仍是你的活**——这两个跟着同一个 base_link 走,别落下。

---

## 2. 「costmap 边界太大 / 走廊太窄」——footprint 与膨胀（重点）

车实际能过，但规划器认为过不去，几乎都是 **footprint 太大** 或 **inflation 太厚**。两者叠加决定了"车在代价图里占多大"。

### 2.1 footprint（**第一优先级**）

代价图用 footprint 多边形做碰撞检查；footprint 越大，可行走廊越窄。

**全栈里有 3 处 footprint，但只对应 2 个"概念"：**

| 概念 | 谁在用 | 在哪 | 现值 |
|---|---|---|---|
| **物理 footprint**（真车实际轮廓，决定避障/碰撞/安全闸） | local_costmap + 控制器碰撞检查 + **安全闸** | `forklift_nav2_oru_test_foxy.yaml:201`（local_costmap） | `[[0.843,0.58],[0.843,-0.58],[-2.043,-0.58],[-2.043,0.58]]` ≈ **2.89 m 长 × 1.16 m 宽** |
| **规划 footprint**（全局规划用，可比物理略小以敢走窄口） | global_costmap（=lattice 规划器） | `forklift_nav2_oru_test_foxy.yaml:255`（global_costmap） | `[[0.50,0.35],[0.50,-0.35],[-0.70,-0.35],[-0.70,0.35]]` ≈ **1.2 m 长 × 0.7 m 宽** |

> ✅ **安全闸 footprint 已自动同步**：`forklift_navigation.launch.py` 在启动时从 yaml 的 **local_costmap footprint** 读出来传给 `safety_command_gate`（见该文件 `footprint_from_params` / `launch_safety_gate`）。**所以改物理 footprint 只改 yaml local_costmap 这一行**，安全闸自动跟随，不会再出现"local 改了、安全闸还是旧值 → 误停"那个坑（上次同时缩 footprint 卡死就是这个）。
> 需要单独给安全闸一个不同 footprint（一般不需要）：启动加 `safety_footprint:="[[...]]"` 覆盖。

#### 一次性调"所有 footprint"的标准做法

1. **量真车物理轮廓**（含货叉前伸、护顶架、车尾），以 `base_link`（后轴中心）为原点：
   - 车头方向（+x）最大伸出 = 前边界
   - 车尾方向（-x）最大伸出 = 后边界（负值）
   - 半宽 = 侧边界
   - 顶点顺序：前左 → 前右 → 后右 → 后左。
2. **改物理 footprint → 只动 yaml local_costmap 这一行**（`:201`）。控制器碰撞检查 + 安全闸**自动用同一份**。例如真车 1.2 m × 0.7 m、后轴略偏后：
   ```yaml
   footprint: "[[0.50, 0.35], [0.50, -0.35], [-0.70, -0.35], [-0.70, 0.35]]"
   ```
3. **改规划 footprint → 动 yaml global_costmap 这一行**（`:255`）。
   - 想最省事、最安全：**两处填同一个真车物理值**（规划=物理，规划出来的路控制器/安全闸一定能过）。
   - 想在窄口更敢规划：global 可比 local 略小（如各边各收 5–10 cm），但**别小过车实际能过的极限**，否则规划出的路安全闸会拦（fail-closed）。
4. footprint 坐标系要和 `rear_axle_x_offset: -0.34`（控制器/规划器，[`:132`](../config/forklift_nav2_oru_test_foxy.yaml#L132) / [`:372`](../config/forklift_nav2_oru_test_foxy.yaml#L372)）一致——都以 `base_link` 为基准。
5. 改完**重启导航栈**生效（yaml 参数在 `configure` 时读；安全闸 footprint 在 launch 时从 yaml 读，也必须重启整条 launch）。

**副作用**：footprint 调太小会真的蹭墙/蹭货架，留 5–10 cm 安全余量即可，剩下的安全裕度交给 inflation。

### 2.2 inflation_radius（第二优先级）

在 footprint 外再"膨胀"一圈高代价区，让路径离障碍物有余量。半径越大，离墙越远、窄通道越容易被判死。

膨胀系数只在 yaml，就 **2 处**（local + global），**一次性调就是这两行一起改、保持一致**：

- local `inflation_radius`：[`forklift_nav2_oru_test_foxy.yaml:205`](../config/forklift_nav2_oru_test_foxy.yaml#L205) → `0.65`
- global `inflation_radius`：[`forklift_nav2_oru_test_foxy.yaml:303`](../config/forklift_nav2_oru_test_foxy.yaml#L303) → `0.65`

**经验值**：`inflation_radius ≈ 车体内切半径 + 期望离墙余量`。叉车半宽约 0.35 m，想离墙 ~0.1–0.2 m，则 `0.45~0.55` 往往就够；当前 0.65 偏保守，窄通道里会显得"边界太大"。

**怎么改**：窄通道过不去 → 先把 `inflation_radius` 从 0.65 往下调到 0.45–0.50，local/global 一起改、保持一致。

### 2.3 cost_scaling_factor

膨胀区内代价的衰减速度。值越大，高代价集中在贴近障碍物处，路径更敢靠近；值越小，代价"摊得更平更远"，路径更躲。

- local [`:206`](../config/forklift_nav2_oru_test_foxy.yaml#L206) / global [`:302`](../config/forklift_nav2_oru_test_foxy.yaml#L302) → `5.0`

**怎么改**：想让车更敢贴近通过窄口，**增大** `cost_scaling_factor`（如 5→8）；想更躲着走则减小。配合 `inflation_radius` 一起看。

### 2.4 costmap 分辨率

[`:199`](../config/forklift_nav2_oru_test_foxy.yaml#L199)（local）/ §global 同名 → `0.05`（5 cm/格）。

分辨率越细，转角/窄口判得越准（少把能过的地方判死），但 CPU 占用上升。0.05 一般够用；实车 CPU 紧张可临时用 0.075，精度要求高可降到 0.025。

---

## 3. 转弯能力（直角转弯 / pivot）

车"能转但规划不让转"或"原地磨"，看这几个：

**规划器（lattice，[`planner_server` 段](../config/forklift_nav2_oru_test_foxy.yaml#L329)）**
- `lattice_arc_radius: 0.60`（[`:352`](../config/forklift_nav2_oru_test_foxy.yaml#L352)）——lattice 弧线基元半径。真车最小转弯半径更小就可减小它，转弯更紧。
- `lattice_pivot_enabled: true`（[`:364`](../config/forklift_nav2_oru_test_foxy.yaml#L364)）、`lattice_pivot_angle`（[`:365`](../config/forklift_nav2_oru_test_foxy.yaml#L365)）——原地/小半径转向基元开关与步进角，直角转弯依赖它。
- `lattice_turn_cost_multiplier` / `lattice_pivot_turn_cost`——转弯代价，调高则规划器更不爱转（偏直），调低更爱转。

**控制器（pivot 执行，[`FollowPath` 段](../config/forklift_nav2_oru_test_foxy.yaml#L118)）**
- `allow_pivot_turn: true`（[`:128`](../config/forklift_nav2_oru_test_foxy.yaml#L128)）——必须开。
- `pivot_turn_radius: 0.6`（[`:131`](../config/forklift_nav2_oru_test_foxy.yaml#L131)）、`pivot_velocity: 0.12`（[`:133`](../config/forklift_nav2_oru_test_foxy.yaml#L133)）、`pivot_yaw_tolerance: 0.05`（[`:134`](../config/forklift_nav2_oru_test_foxy.yaml#L134)）——pivot 半径、速度、停止航向误差。
- `max_steering_angle: 1.5708`（[`:124`](../config/forklift_nav2_oru_test_foxy.yaml#L124)，即 90°）——后轴 pivot 靠打满转向实现。
- `minimum_turning_radius: 0.0`（[`:161`](../config/forklift_nav2_oru_test_foxy.yaml#L161)）——0 表示不额外限制，靠 pivot/转向上限决定。

> 注意：`lattice_arc_radius`（规划）与 `pivot_turn_radius`（执行）要物理自洽——规划出的弧/原地转，控制器得能真转出来。两边一起调。

---

## 4. 障碍物检测范围（plan-once 的限制）

global_costmap 现已加回 `obstacle_layer`（[`:257`](../config/forklift_nav2_oru_test_foxy.yaml#L257)），全局规划会绕开**发 goal 那一刻、且在激光近距离内**被看到的障碍物。

- `obstacle_max_range: 2.5`、`raytrace_max_range: 3.0`（local voxel：[`:232`](../config/forklift_nav2_oru_test_foxy.yaml#L232)；global obstacle：[`:268`](../config/forklift_nav2_oru_test_foxy.yaml#L268)）

**当前为 plan-once（只规划一次、不重规划）**：
- 只有发 goal 时障碍物在 ~2.5 m 内才会进全局路径被绕开；
- 路途中途冒出来的障碍物不会改全局路径——局部代价图能看到它，控制器有碰撞检查（`use_collision_check`）会**停住但不绕**。

**想更早看到障碍物**：放大 `obstacle_max_range`（如 2.5→4.0）和 `raytrace_max_range`（如 3.0→5.0），并确保激光本身量程够。
**想要真正动态绕障**：需要换成"周期重规划"的 BT（本次未做，属设计取舍）。

---

## 5. 速度 / 加速度 / 平滑（[`FollowPath` 段](../config/forklift_nav2_oru_test_foxy.yaml#L118)）

实车第一次跑务必**先把速度压低**再逐步放开。

- `max_velocity: 0.45`（[`:121`](../config/forklift_nav2_oru_test_foxy.yaml#L121)）——前进上限 m/s，首测建议先降到 0.2–0.3。
- `max_reverse_velocity: 0.15`（[`:123`](../config/forklift_nav2_oru_test_foxy.yaml#L123)）、`allow_reverse: true`（[`:151`](../config/forklift_nav2_oru_test_foxy.yaml#L151)）。
- `min_velocity: 0.06`（[`:122`](../config/forklift_nav2_oru_test_foxy.yaml#L122)）——最低爬行速度。
- `max_acceleration: 0.5`（[`:126`](../config/forklift_nav2_oru_test_foxy.yaml#L126)）——起步/刹车顿挫就减小。
- `curvature_slowdown_enabled: true`（[`:162`](../config/forklift_nav2_oru_test_foxy.yaml#L162)）+ `curvature_slowdown_lateral_accel: 0.12`（[`:163`](../config/forklift_nav2_oru_test_foxy.yaml#L163)）——弯道自动减速，过弯发飘就调小 lateral_accel。
- `control_cmd_accel_time` / `control_cmd_decel_time: 0.3`（[`:181`](../config/forklift_nav2_oru_test_foxy.yaml#L181)）——下发给车的加/减速时间常数，决定指令平顺度。

---

## 6. 到点行为（容差 + 终点锁存 latch）

- 全局到点判定 `general_goal_checker`：`xy_goal_tolerance: 0.25`（[`:112`](../config/forklift_nav2_oru_test_foxy.yaml#L112)）、`yaw_goal_tolerance: 0.30`（[`:115`](../config/forklift_nav2_oru_test_foxy.yaml#L115)）。
- 控制器侧 `xy_goal_tolerance: 0.08` / `yaw_goal_tolerance: 0.25`（[`:138`](../config/forklift_nav2_oru_test_foxy.yaml#L138)）。
- **终点锁存（防止到点后反复修正/飘）**：[`:145`](../config/forklift_nav2_oru_test_foxy.yaml#L145)
  - `goal_latch_enabled: true`
  - `goal_latch_xy_tolerance: 0.25`、`goal_latch_yaw_tolerance: 0.30`
  - 一旦进入容差就锁死停车，直到来新 goal；新 goal 目标明显移开（>2×容差）会自动解锁（自愈逻辑见 `forklift_mpc_controller.cpp` `computeVelocityCommands`）。

**怎么改**：到点还在小幅磨 → 适当放宽 `goal_latch_*` / `general_goal_checker` 容差；停得太早不到位 → 收紧。注意 `goal_latch_yaw_tolerance` 与 `general_goal_checker.yaw_goal_tolerance` 要保持一致，否则会出现"锁停了但 BT 不判成功"。

---

## 7. 安全门 Safety Gate（[`FollowPath` 段](../config/forklift_nav2_oru_test_foxy.yaml#L165)）

按真车制动距离设置，宁可保守。

- `safety_gate_enabled: true`（[`:165`](../config/forklift_nav2_oru_test_foxy.yaml#L165)）
- `safety_stop_distance: 0.55`（[`:167`](../config/forklift_nav2_oru_test_foxy.yaml#L167)）——前方障碍到此距离急停。
- `safety_slowdown_distance: 1.25`（[`:168`](../config/forklift_nav2_oru_test_foxy.yaml#L168)）——开始减速距离。
- `safety_min_speed: 0.05`（[`:169`](../config/forklift_nav2_oru_test_foxy.yaml#L169)）。
- `safety_emergency_stop_active: false`（[`:166`](../config/forklift_nav2_oru_test_foxy.yaml#L166)）——置 true 可强制急停（调试/急停联动用）。

车重/速度大 → 增大 stop/slowdown 距离。

> §7 是 controller 内置的 P8.1 限速/急停（FollowPath 段）。它之外还有一个**独立的命令安全闸节点** `safety_command_gate`（P8.2），见 §7.1。

### 7.1 独立命令安全闸 `safety_command_gate`（P8.2，`forklift_safety` 包）

命令链路:`controller/recovery/手动 -> /forklift/control_cmd_raw -> safety_command_gate -> /forklift/control_cmd -> curtis_vehicle_interface`。所有运动命令都先过这个闸,做:急停服务、命令超时/车辆故障/定位丢失即停、速度/转角限幅、`drive_rpm` 限幅（`max_drive_rpm` 默认 2500）、recovery 白名单、costmap 异常即停、footprint 扫掠碰撞即停。

- **真车必须保证 `/odom` 真正发进 gate**。footprint 碰撞检查要靠 `/odom`（`localization_topic`，默认 `/odom`）拿当前位姿把 footprint 摆到 costmap 上。**拿不到位姿 → gate 报 `collision pose missing` → 一直停车**（fail-closed,安全,但车会"明明没障碍却不动")。验证方法见 §8 第 7 条。
- **costmap 默认走严档:`costmap_message_type: costmap_raw` + `costmap_topic: /local_costmap/costmap_raw` + `footprint_collision_cost_threshold: 253`**。原始 0–254 刻度下,阈值 253 表示 footprint 压到 inscribed(253)/lethal(254)/unknown(255) 就停,比之前 OccupancyGrid+100（只在真障碍格才停）**更早停、更保守**,适合上车。
  - 要更宽松（只在真撞上才停,误停少）:`costmap_message_type:=occupancy_grid costmap_topic:=/local_costmap/costmap footprint_collision_cost_threshold:=100`。
  - 阈值越低越严越早停;太低会在远处膨胀圈就误停。
- costmap 缺失/超时/空/截断 → 输出停车并在 `/forklift/safety_gate/status` 写原因（`costmap missing` / `costmap timeout` / `costmap invalid: ...`）。

### 7.2 安全闸"挂了"的临时关闭方案（车照常能跑）

> ⚠️ **仅用于现场调试**：安全闸误停（明明没障碍却一直停车 / costmap 偶发超时把车卡住 / footprint 没标定好导致误碰撞）时,用它先把车跑起来定位问题。**真车正式运行务必保持全开。**

#### 默认状态 & 怎么确认当前开/关

- **默认全开**：真车入口 `forklift_real_navigation.launch.py` 明确把 `enabled` / `collision_check_enabled` / `costmap_monitor_enabled` 都设为 `true`（碰撞检查 + costmap 兜底 + 限幅全在）。当前 `start_foxy_real.sh` **不暴露关闭参数**，避免现场误操作。
- **怎么看运行中到底开没开**——看状态话题：
  ```bash
  ros2 topic echo /forklift/safety_gate/status
  ```
  | 显示 | 含义 |
  |---|---|
  | `raw command` | 闸**全开**且放行(正常行驶中) |
  | `bypass` | 被 `safety_enabled:=false` **旁路**(否决全关、只剩转发+限幅) |
  | `collision ...` / `costmap timeout` / `collision pose missing` | 闸**开着且正在拦车**(在停车) |
  | `waiting for first command` | 闸开着、还没收到运动命令(空闲) |

  另:`ros2 node info /safety_command_gate` 能看到节点在不在、订了哪些话题。

以下开关是安全门节点本身支持的调试能力，但真车顶层 launch 当前故意不透传。若确需定位误停，应先保持 `VEHICLE_DRY_RUN=true`，单独启动/检查安全门；不要为了旁路安全门改用仿真 launch。

**全套档位（从"留兜底"到"全关"）：**

| 现象 | 加这个 launch 参数 | 效果 |
|---|---|---|
| footprint 误判碰撞、贴边就停 | `safety_collision_check_enabled:=false` | 只关 footprint 扫掠碰撞否决,**其余兜底保留**（超时/急停/限幅仍在）。最克制,**首选**。 |
| costmap 偶发超时/缺失把车卡死 | `safety_costmap_monitor_enabled:=false` | 只关 costmap 缺失/超时停车 |
| 安全闸整体逻辑可疑、要先跑通 | `safety_enabled:=false` | **旁路模式**：闸仍在线、仍转发 `control_cmd_raw→control_cmd`、**仍做速度/rpm 限幅**,但跳过所有否决（碰撞/超时/方向）。车能正常跑,限幅保命。 |

> 真车入口目前没有“一条命令全关安全门”的通道。需要这种调试时先停真车输出、切回 `VEHICLE_DRY_RUN=true`，确认问题和风险后再修改专用真车 launch；禁止用 `forklift_navigation.launch.py` 代替，因为它是仿真入口并会启动 Gazebo。

> **不要用 `use_safety_command_gate:=false` 来"关安全闸"**：那会**整个不启动闸节点**,于是没人把 `/forklift/control_cmd_raw` 转成 `/forklift/control_cmd`,**车反而完全不动**。要"关闭但能跑"一定用上面的 `safety_enabled:=false`（闸在线、只旁路否决）。

**验证关掉生效**：`ros2 topic echo /forklift/safety_gate/status` 应显示 `bypass`（`safety_enabled:=false` 时）或 `raw command`（只关单项时）,不再是 `collision ...` / `costmap timeout`。

**恢复**：去掉这些参数（默认全 `true`）重启即恢复满档安全。

---

## 8. 实车 bring-up 检查清单（一切走 vehicle command）

### 8.1 `dry_run` 到底做什么

真车脚本默认 `VEHICLE_DRY_RUN=true`，这是**接口演练模式，不操作真车**：

- 不打开 `can0`，不发送、接收任何 CAN 报文；
- 仍订阅安全门输出 `/forklift/control_cmd`；
- 仍把指令编码成准备发送的 Curtis `0x203` / `0x303` / `0x403`，但只打印 `Curtis TX dry-run` 日志；
- 仍发布 `/odom` 和 TF，但没有真实 CAN feedback，里程计保持初始化的静止零值，**不能拿来验证真实里程计或定位闭环**。

`VEHICLE_DRY_RUN=false` 时，`curtis_vehicle_interface` 在节点构造阶段立即打开指定的 SocketCAN 口，并开始：

- 20 Hz 发送 `0x203` / `0x303` / `0x403`；
- 50 Hz 轮询 Curtis feedback；
- 从 feedback 发布 `/odom`、TF、车辆状态、IO 和故障状态。

> ⚠️ **不会等待导航 goal 才开始发 CAN。** 还没收到第一条 `/forklift/control_cmd` 时，接口也会以 20 Hz 周期发送制动/停止帧；命令超过 0.5 s、急停或非 auto 模式时同样发送停止帧。

### 8.2 vehicle interface 与 Curtis interface 的关系

它们不是两个串联节点。`forklift_vehicle_interface` 是软件包，包内提供两个不同运行环境的适配节点：

```text
forklift_vehicle_interface（软件包）
├── curtis_vehicle_interface（真车：ForkliftControlCommand ↔ SocketCAN/Curtis）
└── sim_command_bridge（仿真：ForkliftControlCommand → Gazebo Twist）
```

真车实际命令链路：

```text
Nav2 controller
  → /forklift/control_cmd_raw
  → safety_command_gate
  → /forklift/control_cmd
  → curtis_vehicle_interface
  → SocketCAN can0
  → Curtis 控制器
```

真车入口只启动 `curtis_vehicle_interface`，不会启动 `sim_command_bridge`。

### 8.3 真车脚本启动哪些组件

`scripts/start_foxy_real.sh` 启动：

- `robot_state_publisher`；
- `curtis_vehicle_interface`；
- `safety_command_gate`；
- Nav2（AMCL、地图服务器、planner、controller、BT navigator 等）；
- RViz（默认开启，可用 `USE_RVIZ=false` 关闭）。

它**不启动** Gazebo、`sim_command_bridge`、仿真激光自过滤节点，也**暂不启动** `forklift_task_manager`。

`forklift_task_manager` 是可选的站点/路线任务调度层，只向 Nav2 的 `NavigateToPose` action 发稀疏任务点，不直接发布底盘命令。手动在 RViz 发 A→B goal 不需要它；执行 `stations.yaml` / `routes.yaml` 的多段任务时才需要。主栈运行后，可在另一终端启动：

```bash
docker exec -it forklift-foxy-real bash -lc '
  source /opt/ros/foxy/setup.bash
  source /workspace/install_foxy/setup.bash
  ros2 launch forklift_task_manager task_manager.launch.py use_sim_time:=false
'
```

### 8.4 什么时候允许真实发送 CAN

先在 `dry_run=true` 下完成以下检查：

- [ ] `can0` 波特率、接线、终端电阻正确，`ip -details link show can0` 显示接口已 UP；
- [ ] `candump can0` 能稳定看到符合协议的 Curtis feedback；
- [ ] 硬件急停、驻车/制动和人工接管有效，第一次测试最好架空驱动轮；
- [ ] dry-run 日志中的方向、转角、RPM、使能和停止帧编码正确；
- [ ] `/scan`、真实 `/odom`、TF、local costmap 和安全门状态正常；
- [ ] 首测场地清空，控制器速度/加速度已压低。

全部通过后才进入 live CAN：

```bash
git switch foxy-real
VEHICLE_DRY_RUN=false CAN_INTERFACE=can0 ./scripts/start_foxy_real.sh
```

启动日志必须出现 `curtis_vehicle_interface opened SocketCAN can0`。若打不开接口，节点应直接启动失败，不要绕过错误继续测试。停止整套真车栈：

```bash
./scripts/stop_foxy_real.sh
```

### 8.5 系统级检查清单

0. **DDS 必须是 CycloneDDS**（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）——否则 FastRTPS 会在 `autostart` 启动时间歇性崩掉随机 lifecycle 节点（amcl/bt_navigator/recoveries）。镜像与改法见 §10。
0.5. ⚠️ **坐标原点一致性——上车前必须和定位同事【确认而非通知】**（详见 §1.5.6）：
   - [ ] 双方约定 **base_link 原点 = 后驱动/转向轴中心,+x 前、+y 左**。
   - [ ] **定位同事发的里程计 `odom→base_footprint` 确实以这个后轴点为参考**——不一致会让 AMCL 发散,现象像"定位坏了"实则原点错位。**这一条必须当面对齐,不能默认。**
   - [ ] 你这边:footprint 已按后轴量（§2.1）、`rear_axle_x_offset` 已设（后轴上=0,controller [`:132`](../config/forklift_nav2_oru_test_foxy.yaml#L132) + lattice [`:372`](../config/forklift_nav2_oru_test_foxy.yaml#L372)）。
   - [ ] `use_sim_time:=false`、TF 树 `view_frames` 连通单根、RViz 里 scan 贴墙（§1.5.5 验证）。
1. 使用 `foxy-real` 的 `scripts/start_foxy_real.sh`，不要用仿真 `forklift_navigation.launch.py`；真车 launch 不包含 Gazebo 和 sim bridge。
2. 第一次只用默认 `VEHICLE_DRY_RUN=true`，确认编码和全 ROS 链路后才按 §8.4 切 live CAN。
3. `publish_control_cmd: true`，controller 输出话题必须是 `/forklift/control_cmd_raw`，不能绕过安全门直接写 `/forklift/control_cmd`。
4. `curtis_vehicle_interface` 只消费安全门放行后的 `/forklift/control_cmd`，并负责 Curtis CAN 编解码、feedback、`/odom` 和 TF。
5. 所有运动（含 pivot 直角转弯）都以 `ForkliftControlCommand` 下发，**BT 里没有 Spin/BackUp 类指令**，恢复行为只清代价图。
6. 真车上 `/cmd_vel` 无人订阅（Nav2 接口要求控制器返回 `TwistStamped`，那是死端口，不影响）。
7. **确认 `/odom` 和 `/local_costmap/costmap_raw` 真进了 `safety_command_gate`**，否则 footprint 检查 fail-closed 永远停车（车"明明没障碍却不动"）。三种判断方法（Foxy docker 内，从最直接到最底层）:

   **① 最直接 —— 看 gate 自己报的状态**（一边发着 raw 命令时):
   ```bash
   ros2 topic echo /forklift/safety_gate/status
   ```
   正常应为 `raw command`。出现 **`collision pose missing`** = `/odom` 没进来；出现 **`costmap missing` / `costmap timeout`** = costmap 没进来。

   **② 看话题订阅者里有没有 gate:**
   ```bash
   ros2 topic info /odom
   ros2 topic info /local_costmap/costmap_raw
   ```
   订阅者计数/列表里应能看到 `safety_command_gate`。

   **③ 看 gate 节点订了哪些话题:**
   ```bash
   ros2 node info /safety_command_gate
   ```
   Subscribers 里应有 `/odom`、`/local_costmap/costmap_raw`、`/forklift/control_cmd_raw`。

   > 常见坑:话题名/命名空间对不上（如 odom 实际发在 `/forklift/odom`，需用 `localization_topic:=` 指过去）；odom 节点没起；`use_sim_time` 不一致导致时间戳老（表现为状态在 `collision pose missing` 与正常之间跳）。

---

## 9. 调参流程建议

1. **先标定 footprint**（§2.1）——量真车、local/global 统一。这一步解决大多数"边界太大"。
2. **再调 inflation**（§2.2/2.3）——窄通道过不去就减 `inflation_radius`、增 `cost_scaling_factor`。
3. **压低速度**（§5）跑通整条路线，确认不蹭不撞。
4. **调转弯**（§3）——直角/路口实测，规划与执行的转弯半径自洽。
5. **调到点与 latch**（§6）——终点不飘、判定成功。
6. **设安全门**（§7）按制动距离。
7. 逐步放开速度到目标值。

> 每次只改一类参数、改完重启栈、单点验证，避免多变量纠缠。local/global 同名参数记得一起改、保持一致。两份 config（foxy 与 base）也要同步。

---

## 10. DDS 中间件：必须用 CycloneDDS（Docker 镜像怎么改）

**这是上车前的硬约束，不是调优项。**

### 10.1 为什么

Foxy 默认的 **FastRTPS** 在 Nav2 `autostart` 启动那一波并发 `configure/activate` 服务调用里有一个**间歇性竞态**：某个 lifecycle 节点回复生命周期服务时崩溃，报

```
what(): failed to send response: client will not receive response,
  at .../rmw-fastrtps-shared-cpp-1.3.2/src/rmw_response.cpp:127
```

崩的是**随机节点**（实测命中过 `amcl`、`bt_navigator`、`recoveries_server`），谁在那一刻输掉竞争就崩谁。`amcl` 崩 = 定位没了，`bt_navigator` 崩 = 整个导航不可用——**在实车上是安全/可用性事故**。

> 这跟仿真无关：崩在 DDS 服务握手层，跟 Gazebo/传感器数据完全无关，**仿真和实车都会出现**。换 **CycloneDDS** 后该竞态消失。

本机实测（同一镜像、同一 `forklift_nav2_oru_test_foxy.yaml`、反复 `autostart:=true` 拉起整套 Nav2）：

| RMW | 崩溃 / 总次数 |
|---|---|
| FastRTPS | 2 / 22（~9%，间歇） |
| **CycloneDDS** | **0 / 34** |

且 CycloneDDS 下 `autostart:=true` 完整拉起（含 `recoveries_server`）+ `forward_ab` A-B 验收 `ab_acceptance=PASS`，导航行为不受影响。复现脚本：[`scripts/recoveries_bringup_campaign.sh`](../../scripts/recoveries_bringup_campaign.sh)。

### 10.2 Docker 镜像怎么改（[`docker/foxy/Dockerfile`](../../docker/foxy/Dockerfile)）

仓库里的镜像**已经改好**，新建/重建镜像即生效。改了两处：

1. **装 CycloneDDS 的 rmw 包**——在 apt 安装列表里加一行（FastRTPS 那行保留，留作一行回退）：

   ```dockerfile
   ros-foxy-rmw-cyclonedds-cpp \
   ros-foxy-rmw-fastrtps-cpp \
   ```

2. **把默认 RMW 设为 CycloneDDS**——改 `ENV`：

   ```dockerfile
   ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

重建镜像：`bash scripts/foxy_docker_build.sh`（或 `docker build -f docker/foxy/Dockerfile -t forklift-nav2:foxy .`）。

> **临时验证**（不重建镜像、只在运行中的容器里试）：
> `apt-get update && apt-get install -y ros-foxy-rmw-cyclonedds-cpp`，
> 然后启动前 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。
> 注意容器重启/重建会丢，**正式落地必须改 Dockerfile**。

### 10.3 ARM 实车注意

- `ros-foxy-rmw-cyclonedds-cpp` **有 arm64（aarch64）官方 deb**，官方 `ros:foxy` 基础镜像本身是 multi-arch，arm64 上同样一行 apt 即可，无需源码编译。
- 车上确认 `uname -m` 是 `aarch64`（ROS 2 不支持 32 位 `armv7l`），且镜像架构与车一致。
- Foxy 已 EOL，若 apt 报找不到包，切到 ROS snapshot 源，arm64 的包仍在。
- **多容器跑 ROS 节点时**（如导航、传感器驱动分容器）：CycloneDDS 默认走共享内存+多播，跨容器要么 `--network host`，要么用 `CYCLONEDDS_URI` 指定网卡/走单播。单容器跑整套栈无此问题。

### 10.4 切换 / 回退

RMW 是**运行时**经环境变量选的，**不用重新 colcon 编译**：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 默认（推荐）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp     # 一行回退
```

`forklift_navigation.launch.py` 还提供 `rmw_implementation:=...` 参数，会一并传给 Gazebo / Nav2 / RViz / 辅助节点，保证整条链路用同一个 RMW（**全栈必须统一，混用 RMW 节点之间不通信**）。

> 用了 CycloneDDS 后，就**不再需要**之前 `autostart:=false` + 手动逐个 `configure/activate` 的临时规避，可以直接 `autostart:=true`，并恢复完整 recovery 链。
