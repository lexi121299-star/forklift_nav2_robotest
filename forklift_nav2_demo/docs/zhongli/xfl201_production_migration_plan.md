# XFL201 量产车型适配修改计划

## 1. 背景

XFL201 是后续准备量产的新车型，车型资料位于：

- `forklift_nav2_demo/docs/zhongli/Weixin Image_2026-07-28_141906_190.png`
- `forklift_nav2_demo/docs/zhongli/Weixin Image_2026-07-28_141956_526.png`
- `forklift_nav2_demo/docs/zhongli/Weixin Image_2026-07-28_142020_614.png`
- `forklift_nav2_demo/docs/zhongli/XFL201  CAN通讯协议V1.0  20250607.xlsx`

当前工程主要按现有 Curtis/MK320 车型打通，真实车接口、CAN codec、运动学参数、footprint、URDF 和安全门参数都带有旧车型假设。XFL201 虽然同样是三支点叉车，但协议和控制方式不同，后续还要量产，因此不能只做临时补丁，应该从一开始按“车型平台化”方式改。

当前已建立专用开发分支：

```bash
xfl201-zhongli-diff-drive
```

## 2. 已知车型参数

从尺寸图读取到的关键参数：

| 项目 | 数值 |
| --- | --- |
| 型号 | XFL201 |
| 操作方式 | 驾驶式 |
| 自重 | 3717 kg |
| 额定载荷 | 2000 kg |
| 导航方式 | 3D 激光 SLAM |
| 定位精度 | ±10 mm |
| 电池 | 80 V / 280 Ah |
| 整车长 | 3226 mm |
| 整车宽 | 1076 mm |
| 整车高 | 2306 mm |
| 载荷中心距 | 500 mm |
| 轴距 | 1470 mm |
| 到货叉垂直面的长度 | 2100 mm |
| 货叉尺寸 | 40 / 122 / 1070 mm |
| 货叉外宽 | 244-770 mm |
| 货叉下降后高度 | 40 mm |
| 起升高度 | 3300 mm |
| 最大爬坡度 | 满载 20% / 空载 25% |
| 转弯半径 | 1743 mm |
| 前直角行驶通道宽度 | 1400 mm |
| 直角转弯通道宽度 | 2137 mm |
| 单侧取卸货通道宽度 | 3610 mm |

这些参数会影响：

- Nav2 footprint。
- safety gate swept footprint。
- URDF / robot_state_publisher 模型。
- 激光雷达安装 TF。
- 后轴/驱动轴位置。
- 最小转弯半径和路径 primitive。
- docking / 取托盘等待位距离。

## 3. 已知 CAN 协议差异

XFL201 协议：

| CAN ID | 方向 | 周期 | 作用 |
| --- | --- | --- | --- |
| `0x231` | 上位机 -> 电控 | 50 ms | 车体控制指令 |
| `0x232` | 上位机 -> 电控 | 50 ms | 心跳，`BYTE0 = 0x05` |
| `0x233` | 上位机 -> 电控 | 50 ms | 货叉控制指令 |
| `0x206` | 电控 -> 上位机 | 50 ms | 车体反馈 |
| `0x207` | 电控 -> 上位机 | 50 ms | 行走/转向故障码 |
| `0x209` | 电控 -> 上位机 | 50 ms | 左电机脉冲计数 |
| `0x20A` | 电控 -> 上位机 | 50 ms | 右电机脉冲计数 |
| `0x6DB` | 电控 -> 上位机 | 未明确 | VCM 阀控故障码 |

关键差异：

- 旧 Curtis 车型使用 `0x203 / 0x303 / 0x403`。
- XFL201 使用 `0x231 / 0x232 / 0x233`。
- 厂家确认 XFL201 不是差速底盘，舵轮角度必须给；直行舵角给 `0`。
- `0x231` 的左、右电机 RPM 是行走速度命令，正常同向同值输出，不用于差速转向。
- 车辆不能左右轮反转原地旋转；可给舵角 `±90°`，以行走 RPM 围绕转向中心旋转。
- XFL201 货叉控制是速度百分比 + 方向 bit，不是泵转速 + 比例阀电流。
- XFL201 协议波特率为 `125 kbps`，标准帧。
- 心跳检测窗口当前为 `200 ms`，`200 ms` 内收到一帧心跳即可；建议发送周期保持 `50 ms`，允许周期不超过 `150 ms`。
- 心跳丢失后电控报错并自动停车，恢复心跳后需要人工按复位按键恢复任务。
- 停车和急停均发 RPM=0；协议虽有刹车力度字段，但厂家说明当前没有刹车选项。
- 当前最大行走命令为 `3000 RPM`，最小低速 `30 RPM`，最小稳定运行速度 `100 RPM`。

因此必须新增 XFL201 专用 CAN codec 和 vehicle interface，不能复用 Curtis CAN codec。

## 4. 总体设计原则

1. **车型隔离**
   - 保留现有 Curtis 车型逻辑。
   - 新增 XFL201 专用配置、launch、codec、adapter。
   - 不在旧车型文件里硬编码新车参数。

2. **接口稳定**
   - 上层 Task Manager、Nav2、Safety Gate 尽量继续使用现有 ROS 接口。
   - 车型差异尽量收敛到 vehicle interface 和车型配置。

3. **参数化量产**
   - 所有量产可调项放入 YAML。
   - 包括尺寸、轮径、轮距、方向符号、最大 RPM、最小 RPM、货叉速度百分比、CAN ID、反馈超时等。

4. **安全优先**
   - 任何底盘运动仍经过 `/forklift/control_cmd_raw -> safety gate -> /forklift/control_cmd`。
   - XFL201 adapter 只订阅 gated command。
   - CAN 超时、反馈超时、急停、非 auto 模式都必须输出停车/停止货叉。

5. **先兼容，再优化模型**
   - 第一阶段可以用现有上层 `ForkliftControlCommand` 兼容 XFL201。
   - 车型接口按舵轮模型处理：上层速度变成左右同值 RPM，上层转角变成 `0x231` 舵轮角度。

## 5. 需要新增和修改的文件

### 5.1 新增车型配置

建议新增：

```text
forklift_nav2_demo/config/xfl201_nav2_foxy.yaml
forklift_vehicle_interface/config/xfl201_vehicle_interface.yaml
forklift_vehicle_interface/config/xfl201_fork_control_adapter.yaml
```

配置内容包括：

- 车长、车宽、前后外廓。
- local/global costmap footprint。
- wheel radius。
- drive track width。
- wheel base。
- rear axle / drive axle offset。
- max velocity。
- max angular velocity。
- max left/right motor rpm。
- min motor rpm。
- acceleration/deceleration limit。
- CAN bitrate、CAN ID、发送周期。
- 方向符号：
  - 车体控制正方向是否为货叉反方向。
  - 左电机 RPM 正方向。
  - 右电机 RPM 正方向。
  - yaw 正方向。
  - 货叉左移/右移方向。
  - 前倾/后倾方向。

### 5.2 新增 CAN codec

建议新增：

```text
forklift_vehicle_interface/forklift_vehicle_interface/zhongli_can_codec.py
```

负责纯协议转换，不依赖 ROS，便于单元测试。

需要实现：

- `encode_0x231(command)`
  - 左电机转速。
  - 右电机转速。
  - 舵轮角度字段。
  - 刹车力度。
  - 自动使能 bit。

- `encode_0x232()`
  - 心跳帧，`BYTE0 = 0x05`。

- `encode_0x233(fork_command)`
  - 货叉速度。
  - 下降速度。
  - 上升/下降/左移/右移/前倾/后倾/张/合 bit。

- `decode_0x206(data)`
  - 左电机转速。
  - 右电机转速。
  - 舵轮角度。
  - 电池电量。
  - 手动/自动模式。

- `decode_0x207(data)`
  - 行走故障码。
  - 转向故障码。

- `decode_0x209(data)`
  - 左电机脉冲计数，`int64` 小端。

- `decode_0x20A(data)`
  - 右电机脉冲计数，`int64` 小端。

- `decode_0x6DB(data)`
  - VCM 阀控故障码。

### 5.3 新增 XFL201 vehicle interface

建议新增：

```text
forklift_vehicle_interface/forklift_vehicle_interface/xfl201_vehicle_interface.py
forklift_vehicle_interface/launch/xfl201_vehicle_interface.launch.py
```

职责：

- 订阅 `/forklift/control_cmd`。
- 将上层速度/角速度意图转换成 XFL201 左右电机 RPM。
- 周期发送 `0x231` 和 `0x232`。
- 如接入货叉控制，也发送 `0x233`。
- 接收 `0x206/0x207/0x209/0x20A/0x6DB`。
- 发布 `/forklift/vehicle_state`。
- 发布 `/forklift/fault_state`。
- 发布 `/forklift/io_state`。
- 根据左右脉冲或左右 RPM 积分 `/odom`。
- 可选发布 `odom -> base_link` TF。

### 5.4 真车 launch 车型选择

当前真车 launch 固定使用 Curtis interface。建议改成可选择车型：

```bash
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py vehicle_model:=xfl201
```

建议支持：

```text
vehicle_model:=curtis
vehicle_model:=xfl201
```

原则：

- `curtis` 保持现有行为。
- `xfl201` 加载 XFL201 interface 和 XFL201 参数。
- 不通过手动改 launch 文件切车型。

### 5.5 URDF / TF 模型

建议新增：

```text
forklift_nav2_demo/urdf/xfl201_diff_drive.urdf.xacro
```

或将现有 URDF 参数化，但量产初期建议先单独建 XFL201 文件，避免影响旧车型。

需要建模：

- `base_footprint`
- `base_link`
- 左驱动轮
- 右驱动轮
- 第三支点/随动轮
- 门架
- 货叉
- 激光雷达安装位
- 货叉雷达/托盘检测雷达安装位

必须现场确认：

- `base_link` 选择在几何中心、驱动轴中心，还是现有系统约定位置。
- 激光雷达相对 `base_link` 的 xyz/rpy。
- 左右驱动轮中心距。
- 驱动轮半径。
- 轮速 RPM 是电机轴 RPM 还是轮端 RPM。
- 如果是电机轴 RPM，需要齿比。

### 5.6 Nav2 / Safety 参数

需要新增 XFL201 专用 Nav2 参数文件：

```text
forklift_nav2_demo/config/xfl201_nav2_foxy.yaml
```

重点参数：

- local costmap footprint。
- global costmap footprint。
- inflation radius。
- controller max velocity。
- controller max angular velocity。
- planner lattice arc radius / diff primitive。
- safety gate footprint。
- rear axle / drive axle offset。
- pivot / in-place rotate 能力。

XFL201 尺寸更长，footprint 必须重新计算，不能沿用当前旧值：

```yaml
footprint: "[[front_x, half_width], [front_x, -half_width], [rear_x, -half_width], [rear_x, half_width]]"
```

其中：

- `half_width` 应至少接近 `1.076 / 2 = 0.538 m`，再加安装误差和安全余量。
- `front_x` / `rear_x` 取决于 `base_link` 定义。
- 叉臂是否纳入 footprint 要按导航场景决定：空车行驶和插叉动作可能需要不同 footprint。

## 6. 底盘建模策略

### 6.1 第一阶段：兼容现有上层接口

短期目标是尽快让 XFL201 能在现有 Task Manager / Nav2 / Safety Gate 下跑起来。

现有上层命令：

```text
ForkliftControlCommand
  velocity_mps
  steering_angle_rad
  forward
  reverse
```

XFL201 adapter 内部转换：

```text
v = signed velocity_mps

travel_rpm = speed_to_motor_rpm(v)

left_rpm  = travel_rpm
right_rpm = travel_rpm
steering_angle = steering_angle_rad
```

优点：

- 上层改动少。
- 可以复用现有 safety gate。
- 可以先完成 CAN 通讯、里程计、基础导航验证。

风险：

- XFL201 不能执行左右轮反转的差速原地旋转。
- 现有 planner/controller 中的 pivot primitive 需要重新定义为“舵角 ±90° 的小半径转向”，而不是差速 counter-rotation。

### 6.2 第二阶段：XFL201 舵轮模型专用控制器

量产稳定后，建议把上层控制器显式升级为 XFL201 舵轮底盘模型：

```text
controller output:
  travel velocity v
  steering angle phi

vehicle interface:
  v -> left/right same rpm
  phi -> 0x231 steering angle
```

这样会更符合厂家确认的 XFL201 控制语义。

需要改动：

- `ForkliftMpcController` 增加 `drive_model:=curtis_pivot|xfl201_steered`。
- `ForkliftVehicleModel` 支持 XFL201 舵角模型。
- planner primitive 中禁止差速原地旋转；`±90°` 转向按小半径绕中心旋转处理。
- safety gate swept footprint 按 XFL201 舵角模型预测。
- sim bridge 支持 XFL201 舵轮模型下的真实运动。

## 7. 货叉控制策略

XFL201 的 `0x233` 货叉控制与 Curtis 不同。

协议字段：

```text
BYTE0: 货叉速度，0~255，对应 0~100%
BYTE1: 下降速度，0~255，对应 0~100%
BYTE2:
  bit0 上升
  bit1 下降
  bit2 左移
  bit3 右移
  bit4 前倾
  bit5 后倾
  bit6 张
  bit7 合
```

建议仍保留上层 action：

```text
ForkMoveTo.action
  target_height_m
  side_shift_m
  tilt_rad
```

新增 XFL201 专用 fork adapter：

```text
xfl201_fork_control_adapter
```

控制闭环：

1. 读取高度、侧移、倾角反馈。
2. 计算目标误差。
3. 将误差映射为速度百分比。
4. 选择方向 bit。
5. 通过 `0x233` 周期发送。
6. 到位后所有 bit 清零，速度清零。

必须补充传感器反馈：

- 高度传感器。
- 侧移位置反馈。
- 倾角反馈。
- 限位开关。
- 货叉故障码。

如果 XFL201 只通过 `0x233` 控制方向和速度，而不反馈高度/侧移/倾角，那么不能做精确 `ForkMoveTo.action` 闭环，必须额外接编码器或传感器。

## 8. 里程计与反馈

XFL201 有左右电机脉冲：

- `0x209` 左电机脉冲计数。
- `0x20A` 右电机脉冲计数。

优先使用脉冲计数积分里程计，而不是只用 `0x206` 的 RPM。

建议里程计逻辑：

```text
delta_left_count = left_count - last_left_count
delta_right_count = right_count - last_right_count

left_distance = delta_left_count * meter_per_pulse
right_distance = delta_right_count * meter_per_pulse

delta_s = (left_distance + right_distance) / 2
delta_yaw = delta_s * tan(steering_angle) / wheel_base

当 steering_angle 接近 ±90° 时，使用现场标定的转向半径：
delta_yaw = delta_s / pivot_turn_radius
```

需要 YAML 配置：

```yaml
encoder_counts_per_motor_rev: TBD
drive_gear_ratio: TBD
drive_wheel_radius_m: TBD
drive_wheel_base_m: TBD
pivot_turn_radius_m: TBD
left_encoder_sign: 1
right_encoder_sign: 1
odom_publish_tf: true
```

必须现场确认：

- 脉冲计数是电机轴还是轮端。
- 正转计数方向和车辆前进方向关系。
- 左右轮编码器符号是否一致。
- 计数是否上电清零。
- int64 溢出处理。

## 9. 安全策略

XFL201 量产必须保留三层安全：

1. **上层任务安全**
   - Task Manager 只编排动作。
   - 不直接发 CAN。

2. **Safety Gate**
   - 检查急停、模式、定位、costmap、障碍物、速度限幅。
   - 只允许 safe command 进入 `/forklift/control_cmd`。

3. **Vehicle Interface**
   - command timeout 停车。
   - feedback timeout 停车或报错。
   - 非自动模式停车。
   - CAN 错误报 fault。
   - 每周期发送 heartbeat。

必须避免：

- Task Manager 绕过 safety gate 发底盘 CAN。
- Nav2 直接发 CAN。
- 货叉动作和底盘运动同时争抢同一控制帧。
- CAN 通讯断开时保持上一次运动命令。

## 10. 分阶段实施计划

### Phase 0：资料确认和分支隔离

目标：

- 建立 XFL201 专用分支。
- 固化协议和尺寸文档。
- 列出待确认项。

产物：

- `xfl201-zhongli-diff-drive` 分支。
- 本计划文档。
- XFL201 参数确认表。

验收：

- 旧车型分支不受影响。
- XFL201 协议、尺寸、坐标方向有文档记录。

### Phase 1：XFL201 CAN codec

目标：

- 完成纯 Python CAN 编解码。

产物：

- `zhongli_can_codec.py`
- `test_zhongli_can_codec.py`

验收：

- `0x231` 左右 RPM 正负值编码正确。
- `0x232` heartbeat 固定输出 `05 00 00 00 00 00 00 00`。
- `0x233` 方向 bit 编码正确。
- `0x206/0x207/0x209/0x20A/0x6DB` 解码正确。

### Phase 2：XFL201 vehicle interface dry-run

目标：

- 不接真 CAN，先 dry-run 打印帧。

产物：

- `xfl201_vehicle_interface.py`
- `xfl201_vehicle_interface.launch.py`
- `xfl201_vehicle_interface.yaml`

验收：

- 收到 `/forklift/control_cmd` 后周期输出 `0x231/0x232`。
- command timeout 后输出停车帧。
- emergency stop 后输出停车帧。
- dry-run 日志能看到左右 RPM。

### Phase 3：真 CAN 通讯

目标：

- 在 XFL201 上完成 CAN 收发。

验收：

- `candump can0` 能看到 `0x231/0x232/0x233`。
- 电控能返回 `0x206/0x207/0x209/0x20A`。
- 自动模式/interlock 生效。
- 空载架空或安全区域内左右轮低速转动方向正确。

### Phase 4：XFL201 里程计

目标：

- 使用 `0x209/0x20A` 脉冲计数发布 `/odom`。

验收：

- 直行 1 m，odom 距离误差在可接受范围。
- 舵角 `±90°` 小半径转向 90 度，odom yaw 方向正确。
- 左右轮符号正确。
- 断 CAN 或反馈超时进入 fault。

### Phase 5：XFL201 URDF / footprint / TF

目标：

- 完成 XFL201 车体模型和安全外廓。

验收：

- RViz 中 `base_link`、`base_scan`、门架、货叉位置合理。
- local/global footprint 与尺寸图一致。
- safety gate 使用同一套 footprint。
- 激光点云/scan 不被车体错误遮挡。

### Phase 6：Nav2 基础导航

目标：

- 在 XFL201 上跑通 A-B 点导航。

验收：

- 直线前进。
- 倒车。
- 低速小角度转弯。
- 舵角 `±90°` 小半径转向。
- 到点停车。
- 障碍物触发 safety stop。

### Phase 7：货叉闭环

目标：

- 接入 XFL201 货叉控制。

验收：

- `ForkMoveTo.action` 可控制升降。
- 可控制侧移。
- 可控制前后倾。
- timeout 停止。
- cancel 停止。
- 限位或 fault 停止。

### Phase 8：两段式取托盘流程

目标：

- 复用现有两段式 N 号库位取托盘任务。

验收：

- 第一段只导航到等待位。
- 第二段升到 N+3。
- 雷达检测偏移。
- 下降到 N。
- 侧移补偿。
- 低速插叉。
- 轻抬离架。
- 任意异常不继续危险动作。

### Phase 9：量产化配置和工具

目标：

- 多台 XFL201 能通过配置文件区分，不改代码。

产物：

```text
config/vehicles/xfl201_default.yaml
config/vehicles/xfl201_unit_001.yaml
config/vehicles/xfl201_unit_002.yaml
```

每台车可配置：

- 轮径标定。
- 轮距标定。
- 编码器比例。
- 左右方向符号。
- 最大速度。
- 货叉速度百分比。
- 雷达 TF。
- 安全余量。

### Phase 10：回归和量产验收

目标：

- 建立量产出厂测试流程。

验收项目：

- CAN 通讯。
- 急停。
- 自动/手动模式。
- 直行 5 m。
- 倒车 2 m。
- 舵角 ±90° 小半径转向 90 度。
- A-B 导航。
- 障碍停车。
- 货叉升降。
- 货叉侧移。
- 货叉倾斜。
- 托盘检测。
- 两段式取托盘。

## 11. 当前建议的代码路线

优先顺序：

1. 新增 `zhongli_can_codec.py`，先把协议编解码测试写扎实。
2. 新增 `xfl201_vehicle_interface.py`，先 dry-run。
3. 新增 XFL201 YAML，把 CAN ID、RPM、方向、尺寸全部参数化。
4. 改真车 launch，支持 `vehicle_model:=xfl201`。
5. 新增 XFL201 Nav2 参数和 footprint。
6. 真车低速 CAN 验证。
7. 接入 odom。
8. 再接货叉闭环和取托盘流程。

## 12. 待现场确认项

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| 左右差速是否完全成立 | 已确认 | 厂家确认不是差速车 |
| 舵轮角度字段是否需要控制 | 已确认 | 必须给；直行给 0 |
| 原地旋转能力 | 已确认 | 不能差速原地旋转；舵角 ±90° 绕中心旋转 |
| 最大 RPM | 已确认 | 当前最大 3000 RPM |
| 最小 RPM | 已确认 | 低速 30 RPM，稳定运行 100 RPM |
| 心跳超时 | 已确认 | 检测 200 ms，建议发送周期 50 ms，允许周期不超过 150 ms |
| 停车/急停方式 | 已确认 | 发 RPM=0，没有刹车选项 |
| 左电机 RPM 正方向 | 待确认 | 协议说车体正方向是货叉反方向，需要实测 |
| 右电机 RPM 正方向 | 待确认 | 需要实测 |
| RPM 是电机轴还是轮端 | 待确认 | 影响速度换算 |
| 齿比 | 待确认 | 如果 RPM 是电机轴必须配置 |
| 驱动轮半径 | 待确认 | 影响 odom 和 RPM 换算 |
| 轴距/转向半径 | 待确认 | 影响舵轮模型 odom 和 swept footprint |
| 脉冲每圈数量 | 待确认 | 影响 odom |
| 货叉高度反馈来源 | 待确认 | `0x233` 只看到控制，没有高度反馈 |
| 侧移反馈来源 | 待确认 | 需要传感器或 CAN 反馈 |
| 倾角反馈来源 | 待确认 | 需要传感器或 CAN 反馈 |
| 激光雷达安装位 | 待确认 | 影响 TF 和避障 |
| 托盘检测雷达安装位 | 待确认 | 影响取托盘偏移计算 |

## 13. 风险点

1. 已确认 XFL201 不是差速模型；后续 planner/controller 不能再按差速原地旋转设计。
2. 如果没有准确 wheel radius / gear ratio / pulse ratio，odom 会漂。
3. 如果货叉没有位置反馈，不能做可靠的 `ForkMoveTo.action`。
4. 新车尺寸更长，旧地图窄通道路线可能需要重新验证。
5. 量产车型必须避免在代码中写死某一台车的标定值。

## 14. 结论

XFL201 应作为独立车型平台接入，而不是在 Curtis 车型上打补丁。

第一版建议目标是：

- 独立分支。
- 独立 CAN codec。
- 独立 vehicle interface。
- 独立 YAML 参数。
- 独立 Nav2 footprint/URDF。
- 复用上层 Task Manager、Safety Gate、两段式取托盘流程。

等第一台车跑通后，再把 `vehicle_model` 抽象稳定下来，为后续多台 XFL201 量产交付做配置化支持。
