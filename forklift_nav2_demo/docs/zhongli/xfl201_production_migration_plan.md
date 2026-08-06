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
| 承载长度 | 434 mm |
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

当前建模约定：

- `base_link` 放在两轮轴中心。
- 车辆前进方向定义为叉臂反方向，即 `base_link +x` 指向车体尾部/配重方向，货叉方向为 `-x`。
- 舵角 `+90°/-90°` 时，先按车辆围绕两轮轴中心旋转处理；该点与 `base_link` 重合，后续实车低速测试复核。
- `Wa = 1743 mm` 暂按外轮廓最小转弯半径使用，不直接等同于控制模型里的 `pivot_turn_radius`；后续作为外置参数标定。

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
forklift_nav2_demo/urdf/xfl201_steered.urdf.xacro
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

- `base_link` 已按当前约定放在两轮轴中心，`+x` 为叉臂反方向，`-x` 为货叉方向。
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
- planner lattice arc radius / steered pivot primitive。
- safety gate footprint。
- rear axle / drive axle offset。
- pivot / in-place rotate 能力。

XFL201 尺寸更长，footprint 必须重新计算，不能沿用当前旧值：

```yaml
footprint: "[[front_x, half_width], [front_x, -half_width], [rear_x, -half_width], [rear_x, half_width]]"
```

其中：

- `half_width` 应至少接近 `1.076 / 2 = 0.538 m`，再加安装误差和安全余量。
- 按当前 `base_link` 约定，初版空车车体 footprint 可先取：
  - `front_x = l2 - x = 2.100 - 0.434 = +1.666 m`。
  - `rear_x = -x = -0.434 m`，到货叉垂直面。
  - `half_width = 0.538 m`，实际配置需要再加安全余量。
- 如果把货叉纳入 footprint，货叉尖端可先按 `rear_x = -(x + fork_length) = -(0.434 + 1.070) = -1.504 m` 估算。
- 叉臂是否纳入 footprint 要按导航场景决定：空车行驶和插叉动作可能需要不同 footprint。

## 6. 底盘建模策略

### 6.0 当前模型复核结论

根据厂家最新回复，XFL201 不是左右差速底盘，而是舵轮角度 + 左右行走电机 RPM 的三支点车型。因此原计划中“按差速车重做车辆模型”的方向不需要继续推进。

现有 Nav2 侧 `ForkliftVehicleModel` 已经具备舵角运动学能力：

- 普通转弯使用 `velocity + steering_angle`。
- 普通转弯 yaw rate 按 `velocity * tan(steering_angle) / wheel_base` 计算。
- 接近 `pivot_steering_angle` 时支持 `allow_pivot_turn` 分支。
- pivot 分支已有 `pivot_turn_radius` 和 `rear_axle_x_offset` 参数，可表达“舵角 ±90° 绕某个中心点旋转”的效果。
- `forklift_safety` 的 command gate 也已经按同类参数做短时 swept footprint 预测。

所以当前判断是：

- **不需要大规模重写 planner/controller/vehicle model C++。**
- **不需要新建差速底盘模型。**
- **需要做 XFL201 专用参数、footprint、URDF、safety gate 参数和里程计标定。**

当前约定 `base_link` 位于两轮轴中心，舵角 `+90°/-90°` 时先按围绕该点旋转处理。因此 XFL201 初版可将 `rear_axle_x_offset` / `lattice_rear_axle_x_offset` 配成 `0.0`。若实车低速测试发现旋转中心相对 `base_link` 有偏移，再通过参数修正；只有参数无法表达时才做小范围模型补丁。

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
- 现有 planner/controller 中的 pivot primitive 必须按“舵角 ±90° 的小半径转向/绕中心旋转”理解，不能再按差速 counter-rotation 理解。

### 6.2 后续优化：XFL201 舵轮语义显式化

量产稳定后，可以把上层控制器的命名和参数显式升级为 XFL201 舵轮底盘语义：

```text
controller output:
  travel velocity v
  steering angle phi

vehicle interface:
  v -> left/right same rpm
  phi -> 0x231 steering angle
```

这样会更符合厂家确认的 XFL201 控制语义。

这不是当前上车前的阻塞项。下周优先按参数化方式确认模型是否够用，只在实车验证发现当前 pivot 几何表达不了 XFL201 行为时，再做小范围代码修改。

可能的改动：

- `ForkliftMpcController` 增加 `drive_model:=curtis_pivot|xfl201_steered`。
- `ForkliftVehicleModel` 参数命名补充 XFL201 舵角模型说明。
- planner primitive 文档和配置中禁止差速原地旋转语义；`±90°` 转向按小半径绕中心旋转处理。
- safety gate swept footprint 参数与 XFL201 Nav2 参数统一。
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

如果 `base_link` 与两轮轴中心/旋转中心重合，pivot 时 `delta_s` 只用于计算 yaw，不应继续作为 `base_link` 的 x/y 平移量积分。
```

需要 YAML 配置：

```yaml
encoder_counts_per_motor_rev: 64
drive_gear_ratio: 26.75
drive_wheel_radius_m: 0.225
drive_wheel_base_m: 1.47
front_track_width_m: 0.936
left_meter_per_pulse: 0.0008257690970300274   # pi*0.45/(26.75*64), supplier-confirmed initial value
right_meter_per_pulse: 0.0008257690970300274  # replace only if 5m/10m field calibration shows drift
pivot_turn_radius_m: 1.743  # 暂按 Wa 外轮廓转弯半径占位，后续实车标定
pivot_center_x_offset_m: 0.0
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

## 12. 下周确认和修改清单

### 12.1 已确认的方向

下周不再按差速车方向推进 XFL201。当前代码分支已经按厂家回复调整为：

- `0x231` 左右电机 RPM 同向同值输出，用作行走速度。
- 转向由 `0x231` 舵轮角度字段控制。
- 直行舵角给 `0`。
- 停车/急停均发 RPM=0，不依赖刹车字段。
- 自动模式必须持续发送 `0x232` 心跳。
- 心跳周期建议保持 `50 ms`，不超过厂家允许的 `150 ms`。
- `0x233` 暂不由底盘 vehicle interface 周期发送，后续由货叉 adapter 独立接管，避免底盘和货叉抢同一帧。

### 12.2 需要确认的模型参数

这些参数决定 Nav2、Safety Gate、URDF、odom 是否一致：

| 参数 | 当前判断 | 下周动作 |
| --- | --- | --- |
| `wheel_base` | 尺寸图读取为 `1.47 m` | 写入 XFL201 Nav2、safety、odom YAML，并现场复核 |
| `pivot_turn_radius` | `Wa=1.743 m` 暂按外轮廓转弯半径占位 | 作为可配置外置参数，后续用实车低速标定替换 |
| `rear_axle_x_offset` | 初版 `0.0 m` | `base_link` 在两轮轴中心，pivot 初版按围绕该点旋转 |
| `base_link` 位置 | 已确认初版 | 两轮轴中心，`+x` 为叉臂反方向，`-x` 为货叉方向 |
| footprint `front_x/rear_x/half_width` | 初版可计算 | 空车约 `front_x=+1.666 m`、`rear_x=-0.434 m`、`half_width=0.538 m + safety_margin` |
| `lattice_rear_axle_x_offset` | 需要和 controller 一致 | 与 `rear_axle_x_offset` 同步 |
| safety gate pivot 参数 | 需要和 Nav2 一致 | 同步 `wheel_base/pivot_turn_radius/rear_axle_x_offset` |

### 12.3 下周建议修改项

1. 新增或整理 `forklift_nav2_demo/config/xfl201_nav2_foxy.yaml`。
2. 将 XFL201 的 `wheel_base`、`max_steering_angle`、`allow_pivot_turn`、`pivot_steering_angle`、`pivot_turn_radius`、`rear_axle_x_offset` 写入专用配置。
3. 将 safety gate 的 XFL201 参数与 Nav2 参数对齐。
4. 新增或重命名 XFL201 URDF 为舵轮车型语义，例如 `xfl201_steered.urdf.xacro`，不要再使用 `diff_drive` 命名。
5. 按 XFL201 尺寸更新 costmap footprint 和 safety footprint；空车和带货叉 footprint 分开考虑。
6. 根据厂家回复或现场测试填写 odom 标定参数：`meter_per_pulse`、轮半径、齿比、左右脉冲方向、舵角反馈方向。
7. 用低速实车测试验证：
   - 直行 1 m 的 odom 距离。
   - 小角度转向 yaw 方向。
   - 舵角 `+90°/-90°` 的旋转方向和等效半径。
   - safety gate 预测轨迹和实车运动是否一致。

### 12.3.1 已提前完成项

2026-07-31 已先完成以下改动：

- 新增 `forklift_nav2_demo/config/xfl201_nav2_foxy.yaml`。
- 新增 `forklift_nav2_demo/urdf/xfl201_steered.urdf.xacro`，用于 XFL201 真车 TF/可视化建模。
- `forklift_real_navigation.launch.py` 支持 `vehicle_model:=xfl201` 时默认选择 XFL201 Nav2 参数和 XFL201 URDF。
- safety gate 在 `vehicle_model:=xfl201` 时使用 `wheel_base=1.47`、`pivot_turn_radius=1.743`、`rear_axle_x_offset=0.0`。
- XFL201 Nav2 配置已写入保守带货叉 footprint：`[[1.666, 0.60], [1.666, -0.60], [-1.504, -0.60], [-1.504, 0.60]]`。
- XFL201 vehicle interface 配置已将 `pivot_turn_radius_m` 改为 `1.743`，并新增 `pivot_center_x_offset_m: 0.0`。
- XFL201 odom 在舵角 `+90°/-90°` 且 `pivot_center_x_offset_m=0.0` 时只积分 yaw，不再把轮端行走距离误算成 `base_link` 平移。
- 已通过 Foxy Docker 构建和 `forklift_vehicle_interface`、`forklift_safety` 测试。

仍需下周现场确认：

- `Wa=1.743 m` 是否能作为 `pivot_turn_radius` 的临时值，或是否需要低速标定出更准确的等效半径。
- XFL201 footprint 是否使用保守带货叉外廓，还是拆成空车/插叉两套 footprint。
- 轮半径、齿比、脉冲到米比例已由厂家确认；左右编码器方向仍需现场低速验证。
- 激光雷达、叉尖光电、叉根托盘到位检测的 TF 和 topic/message。

### 12.3.2 当前剩余工作（2026-08-03）

截至 2026-08-03，代码侧已经完成 XFL201 专用 CAN codec、vehicle interface dry-run、Nav2 参数、URDF、launch 车型选择、safety gate 参数对齐、pivot odom 初版修正。后续剩余工作主要分为“现场标定/验证”和“未实现 adapter”两类。

| 模块 | 当前状态 | 剩余工作 | 阻塞/输入 |
| --- | --- | --- | --- |
| 真 CAN 通讯 | 已有 `xfl201_vehicle_interface` 和 `zhongli_can_codec` | 上车验证 `0x231/0x232` 周期发送、`0x206/0x207/0x209/0x20A/0x6DB` 接收、SocketCAN `125 kbps` 配置 | 需要实车、CAN 接线、自动模式 |
| 心跳/复位流程 | 代码按 20 Hz 发送心跳 | 实测心跳丢失后自动停车、心跳恢复后必须人工复位、任务状态如何恢复 | 厂家复位按键/状态反馈 |
| 行走方向标定 | 初版使用 `body_positive_is_fork_reverse: true`，左右电机同值 RPM | 实测前进/后退方向、左右电机 RPM 正负号、舵角正负号、`steering_angle_deg` 零位 | 安全区域低速试车 |
| RPM 与速度换算 | 厂家确认轮径 `450 mm`、减速比 `26.75`、最大 `3000 RPM` | 用 `315.35 RPM ≈ 1 km/h` 和现场测速复核；确认 RPM 正负方向 | 现场测速/低速试车 |
| 编码器里程计 | 厂家确认 `0x209/0x20A` 是电机端脉冲，`64` 脉冲/电机转，减速比 `26.75` | 已写入 `meter_per_pulse=0.0008257690970300274`；仍需验证左右脉冲符号、上电是否清零、溢出处理，并做 5m/10m 标定 | 低速直行标定 |
| pivot 半径 | `Wa=1.743 m` 暂作为外置占位 | 实测舵角 `+90°/-90°` 转 90 度的等效半径和 yaw 方向，必要时调整 `pivot_turn_radius` | 低速转向标定 |
| pivot 中心 | 初版 `pivot_center_x_offset_m=0.0`、`rear_axle_x_offset=0.0` | 验证车辆是否确实围绕两轮轴中心旋转；如果不是，标定中心相对 `base_link` 的 x 偏移 | 低速转向轨迹 |
| Nav2/safety footprint | 已有保守带货叉 footprint | 确认正式导航使用空车 footprint 还是带货叉 footprint；根据实际外廓、安全余量和通道宽度调参 | 现场通道和避障策略 |
| URDF / TF | 已有 XFL201 初版 URDF | 测量并写入 270° 避障激光、补盲相机、叉尖光电、叉根检测、托盘检测雷达的 `xyz/rpy` | 传感器安装尺寸 |
| 定位接入 | launch 默认 `/odom`、`map -> odom -> base_link` | 明确定位侧最终提供哪些 topic，是否由定位发布 `map -> odom`，以及是否使用 AMCL/SLAM/外部定位 | 定位同事 topic/message 定稿 |
| 实车基础导航 | Nav2 参数已可切 `vehicle_model:=xfl201` | 跑通直行、倒车、小角度转弯、`±90°` 小半径转向、到点停车、障碍停车 | CAN/odom/TF 都正常后执行 |
| XFL201 货叉 adapter | 仍未实现 | 新增 `xfl201_fork_control_adapter`，把 `ForkMoveTo.action` 转成 `0x233` 速度百分比和方向 bit | 货叉反馈来源未确认 |
| 货叉闭环反馈 | 未接入 | 明确高度、侧移、倾角、限位、故障码来源；没有反馈就不能可靠闭环 `ForkMoveTo.action` | 厂家协议或外置传感器 |
| 托盘/雷达接入 | 任务文档已有两段式流程 | 做雷达 message/action adapter，将实际雷达 message 转成托盘偏移结果；接入 task manager 取叉段 | 雷达 topic/message 定稿 |
| 两段式取托盘实车联调 | 设计文档已有 | 导航到等待位、升到 `N+3`、雷达检测、下降到 `N`、偏移补偿、插叉、轻抬的实车流程验证 | 货叉 adapter + 雷达 adapter |
| 量产配置 | 方案中规划了 per-vehicle YAML | 建立 `xfl201_default.yaml` 和单车标定文件，避免把某一台车参数写死在代码里 | 第一台车标定完成后沉淀 |
| 量产验收 | 文档列了验收项 | 编写出厂/回归 checklist 和必要脚本，覆盖 CAN、急停、自动/手动、导航、货叉、托盘检测 | 第一轮实车验证结果 |

优先级建议：

1. 先做真 CAN 通讯和行走方向标定，确认车辆能安全收发、停车、前进/后退。
2. 再做编码器里程计和 pivot 半径/中心标定，保证 `/odom` 与实车运动自洽。
3. 然后做 TF/footprint/safety gate 实车复核，避免模型和避障误差。
4. 在底盘闭环稳定后，再进入 XFL201 货叉 adapter、雷达 adapter 和两段式取托盘实车联调。

### 12.4 是否需要改大模型的判断条件

默认先认为现有 `ForkliftVehicleModel` 足够表达 XFL201：

```text
普通转弯: yaw_rate = v * tan(steering_angle) / wheel_base
90 deg pivot: yaw_rate = v / pivot_turn_radius
旋转中心: rear_axle_x_offset = 0.0，初版等同 base_link
```

只有出现以下情况，才考虑修改 C++ 车辆模型：

- 实车 `+90°/-90°` 旋转中心无法用单一 `rear_axle_x_offset` 表达。
- planner、controller、safety gate 对同一条 pivot 轨迹预测不一致。
- `base_link` 不是当前模型假设的参考点，且通过参数无法修正。
- XFL201 需要同时表达“行走速度参考点”和“旋转中心参考点”两个不同坐标。

如果只是尺寸、半径、中心点、速度限制不同，优先通过 YAML 和 URDF 解决。

## 13. 待现场确认项

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| 左右差速是否完全成立 | 已确认 | 厂家确认不是差速车 |
| 舵轮角度字段是否需要控制 | 已确认 | 必须给；直行给 0 |
| 原地旋转能力 | 已确认 | 不能差速原地旋转；舵角 ±90° 绕中心旋转 |
| 最大 RPM | 已确认 | 当前最大 3000 RPM |
| 最小 RPM | 已确认 | 低速 30 RPM，稳定运行 100 RPM |
| 心跳超时 | 已确认 | 检测 200 ms，建议发送周期 50 ms，允许周期不超过 150 ms |
| 停车/急停方式 | 已确认 | 发 RPM=0，没有刹车选项 |
| `base_link` 位置 | 已确认初版 | 两轮轴中心，`+x` 为叉臂反方向 |
| 轴距 | 已确认初版 | 尺寸图读取为 1470 mm |
| `Wa` 转弯半径 | 已确认占位 | 1743 mm，暂按外轮廓转弯半径，可外置参数修改 |
| 左电机 RPM 正方向 | 待确认 | 协议说车体正方向是货叉反方向，需要实测 |
| 右电机 RPM 正方向 | 待确认 | 需要实测 |
| RPM 是电机轴还是轮端 | 已确认 | 按电机侧 RPM 处理，减速比 `26.75` |
| 齿比 | 已确认 | 电机转 `26.75` 圈，轮子转 `1` 圈 |
| 驱动轮半径 | 已确认初版 | 轮径 `450 mm`，半径 `0.225 m`，现场仍可按滚动半径微调 |
| pivot 等效半径 | 待确认 | `Wa` 可先占位，但真实 `pivot_turn_radius` 仍建议低速标定 |
| 脉冲每圈数量 | 已确认 | 电机转一圈 `64` 个脉冲，`1` 个脉冲约 `0.0008 m` |
| 货叉高度反馈来源 | 待确认 | `0x233` 只看到控制，没有高度反馈 |
| 侧移反馈来源 | 待确认 | 需要传感器或 CAN 反馈 |
| 倾角反馈来源 | 待确认 | 需要传感器或 CAN 反馈 |
| 激光雷达安装位 | 待确认 | 影响 TF 和避障 |
| 托盘检测雷达安装位 | 待确认 | 影响取托盘偏移计算 |

## 14. 风险点

1. 已确认 XFL201 不是差速模型；后续 planner/controller 不能再按差速原地旋转设计。
2. 如果没有准确 wheel radius / gear ratio / pulse ratio，odom 会漂。
3. 如果货叉没有位置反馈，不能做可靠的 `ForkMoveTo.action`。
4. 新车尺寸更长，旧地图窄通道路线可能需要重新验证。
5. 量产车型必须避免在代码中写死某一台车的标定值。

## 15. 最新遗留问题摘要

截至 2026-08-06，厂家已确认轮径、轮距、减速比和电机端脉冲换算；代码侧已经把 `meter_per_pulse=0.0008257690970300274` 写入 XFL201 配置。后续剩余问题集中在实车验证和上层 adapter：

1. 底盘 CAN 实车验证：
   - 验证 `0x231/0x232` 周期发送，`0x206/0x207/0x209/0x20A/0x6DB` 正常接收。
   - 确认 SocketCAN `can0`、`125 kbps`、自动模式和心跳丢失停车/恢复复位流程。

2. 底盘方向和 odom 标定：
   - 低速验证前进/后退方向、左右电机 RPM 正负号、左右脉冲计数正负号、舵角正负方向。
   - 用 5 m / 10 m 直线标定 `/odom`，必要时微调 `left_meter_per_pulse`、`right_meter_per_pulse` 和滚动半径。

3. Pivot 转向标定：
   - 舵角 `+90°/-90°` 低速转向，确认车辆是否绕两轮轴中心旋转。
   - 标定 `pivot_turn_radius_m` 和 `pivot_center_x_offset_m`，必要时同步 Nav2、safety gate 和 vehicle interface 参数。

4. 定位和 TF 接入：
   - 明确定位侧提供 `map -> odom`，还是只提供 `/robot_pose` / `/localization/pose`。
   - 如果定位只给 map 下位姿，需要新增 adapter 转成 `map -> odom`，底盘仍发布 `odom -> base_link`。
   - 测量避障雷达、补盲相机、叉尖光电、叉根检测、托盘检测雷达的 `xyz/rpy`。

5. 货叉与托盘 adapter：
   - 新增 XFL201 货叉 adapter，把 `ForkMoveTo.action` 转成 `0x233` 控制。
   - 明确高度、侧移、倾角、限位和故障反馈来源；没有反馈时不能做可靠闭环。
   - 做雷达 message/action adapter，接入两段式取托盘流程。

6. 量产沉淀：
   - 建立 `xfl201_default.yaml` 和单车标定 YAML，避免把第一台车参数写死在代码里。
   - 补出厂/回归 checklist，覆盖 CAN、急停、手自动、odom、TF、导航、货叉和托盘检测。

## 16. 结论

XFL201 应作为独立车型平台接入，而不是在 Curtis 车型上打补丁。

第一版建议目标是：

- 独立分支。
- 独立 CAN codec。
- 独立 vehicle interface。
- 独立 YAML 参数。
- 独立 Nav2 footprint/URDF。
- 复用上层 Task Manager、Safety Gate、两段式取托盘流程。

等第一台车跑通后，再把 `vehicle_model` 抽象稳定下来，为后续多台 XFL201 量产交付做配置化支持。
