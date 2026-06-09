# P1/P2 Vehicle Interface Next Steps

这份文档给新窗口继续做，用来从已经完成的 P0 基线进入 `forklift_msgs` 和 `forklift_vehicle_interface`。

当前结论：

- P0.1 已完成：`/clock`、`/odom`、TF、AMCL 初始位姿、Nav2 lifecycle 可用。
- P0.2 已完成：`/follow_path` 短路径可走通。
- 下一步不要先移植 ORU MPC。先固定真车/仿真共用的控制接口，否则后面算法输出还会返工。
- 所有 ROS 终端统一使用 `rmw_fastrtps_cpp`，避免手画路径工具和 Nav2 action 之间出现 DDS/RMW 混用卡住。

启动任何 ROS 终端前建议：

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
source /home/pl/robotest/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 1. 目标架构

目标是把 Nav2/ORU 算法和真实底盘协议解耦：

```text
Nav2 / ORU controller
        |
        v
forklift_msgs/msg/ForkliftControlCommand
        |
        v
forklift_vehicle_interface
        |
        +--> 仿真: 转成 /cmd_vel
        |
        +--> 真车: 编码成 Curtis CAN 帧 0x203 / 0x303 / 0x403
```

不要把 CAN 协议写进 planner/controller。

不要让 ORU MPC 直接关心 Curtis 字节布局。

### 1.1 Gazebo 和 bridge 的职责边界

仿真阶段不要让 bridge 发布运动学状态。

Gazebo 已经负责车辆运动学仿真，并由现有插件发布：

```text
/odom
odom -> base_footprint TF
```

因此仿真 bridge 第一版只是 command adapter：

```text
/forklift/control_cmd -> /cmd_vel
```

它不发布：

```text
/odom
odom -> base_link/base_footprint TF
```

真车阶段才由 vehicle interface 根据底盘反馈发布运动学状态：

```text
Curtis CAN feedback / encoder / steering angle
        |
        v
forklift_vehicle_interface
        |
        +--> /odom
        +--> odom -> base_link TF
        +--> /forklift/vehicle_state
        +--> /forklift/fault_state
```

推荐把两个节点名字分清楚：

```text
sim_command_bridge
  仿真用，只做 /forklift/control_cmd -> /cmd_vel

curtis_vehicle_interface
  真车用，做 /forklift/control_cmd -> CAN，并由 CAN feedback 发布 /odom、TF 和车辆状态
```

这样后面 ORU controller 只面向统一的 `ForkliftControlCommand`，不会关心当前是在 Gazebo 还是实车。

## 2. 协议摘要

参考文件：

```text
/home/pl/Downloads/24.12-MK320-AGV通讯协议.pdf
```

从 PDF 抽到的关键信息：

- CANopen / ISO 11898，默认 250 kbit/s。
- PLC/上位机为主站，Curtis 控制器为从站。
- 节点地址为 `0x03`。

上位机发送到 Curtis：

```text
0x203 行走控制
  BYTE0 bit0: 使能/互锁
  BYTE0 bit1: 前进
  BYTE0 bit2: 后退
  BYTE0 bit3: Brake Rate 刹车
  BYTE0 bit4: 小时计清零
  BYTE0 bit5: 刹车继电器 PWM 输出
  BYTE0 bit6: 倒车继电器 PWM 输出
  BYTE1-2: 行驶速度 0-4000 rpm，1 = 1 rpm，小端
  BYTE3: 加速率，2-255 对应 0.2-25.5 s
  BYTE4: 减速率，2-255 对应 0.2-25.5 s
  BYTE6-7: 转向角 -9000 到 9000，对应 -90 到 90 deg，小端有符号

0x303 泵电机和离散输出
  BYTE0-1: 泵电机转速 0-3000 rpm，小端
  BYTE3 bit0: 大灯继电器
  BYTE3 bit1: 喇叭继电器
  BYTE3 bit2: 下降电磁阀

0x403 比例阀
  BYTE0: 下降比例阀，0-200 对应 0-2000 mA
  BYTE1: 起升比例阀，0-200 对应 0-2000 mA
  BYTE2: 收缩比例阀
  BYTE3: 扩张比例阀
  BYTE4: 左移比例阀
  BYTE5: 右移比例阀
  BYTE6: 前倾比例阀
  BYTE7: 后仰比例阀
```

Curtis 反馈到上位机：

```text
0x183 主/左驱动反馈
  BYTE0-1: 主驱动电机转速 -4000 到 4000 rpm
  BYTE2-3: 主驱动电机电流，0-10000 对应 0-1000 A
  BYTE4: 主驱动控制器故障码
  BYTE5: 电量 0-100 %
  BYTE6-7: 驱动控制器温度，-1000 到 3000 对应 -100.0 到 300.0 C

0x283 IO 状态
  BYTE4: 1351 Output 1-8 状态
  BYTE5: 下降电磁阀、刹车继电器、离合、主接触器、倒车继电器等
  BYTE6 bit1: 手自动
  BYTE6 bit2: 软急停
  BYTE6 bit3: 手刹

0x383 转向和右驱反馈
  BYTE0-1: 转向角 -9000 到 9000，对应 -90 到 90 deg
  BYTE2: 转向控制器故障码
  BYTE3-4: 从/右驱动电机转速 -4000 到 4000 rpm
  BYTE5-6: 从/右驱动电机电流，0-10000 对应 0-1000 A
  BYTE7: 从/右驱动控制器故障码

0x483 起升控制器反馈
  BYTE0-1: 起升电机转速
  BYTE2-3: 起升电机电流
  BYTE4-5: 起升控制器温度
  BYTE6: 起升控制器故障码
  BYTE7 bit0: 大灯继电器
  BYTE7 bit1: 喇叭继电器
```

安全注意：

- 手动切自动时，自动发送速度必须先降为 0。
- 急停复位后，必须先收到电控反馈报文，再允许上位机外发控制指令。
- CAN 负载率不要超过 50%。
- 待机状态 `0x203 BYTE0 bit0 = 0`。

## 3. P1: 创建 forklift_msgs

新建 package：

```text
forklift_msgs
```

建议命令：

```bash
cd /home/pl/robotest/src
ros2 pkg create forklift_msgs --build-type ament_cmake
```

如果仓库不是 `src` 布局，就先在 `/home/pl/robotest` 下确认当前 package 位置，再按现有结构创建。

### 3.1 消息文件

第一版先建这些：

```text
forklift_msgs/msg/ForkliftControlCommand.msg
forklift_msgs/msg/ForkliftVehicleState.msg
forklift_msgs/msg/ForkliftFaultState.msg
forklift_msgs/msg/ForkliftIoState.msg
forklift_msgs/srv/SetControlMode.srv
forklift_msgs/srv/SetEmergencyStop.srv
```

`ForkliftControlCommand.msg` 建议：

```text
std_msgs/Header header

bool enable
bool brake
bool forward
bool reverse

float64 velocity_mps
float64 drive_rpm
float64 steering_angle_rad
float64 steering_angle_deg

float64 accel_time_sec
float64 decel_time_sec

float64 pump_rpm
float64 lift_valve_ma
float64 lower_valve_ma
float64 side_shift_left_valve_ma
float64 side_shift_right_valve_ma
float64 tilt_forward_valve_ma
float64 tilt_backward_valve_ma

bool horn
bool light
```

说明：

- `velocity_mps` 给算法/仿真用。
- `drive_rpm` 给真车协议和调试用。
- 第一版允许两个字段同时存在，bridge 可以优先用 `velocity_mps`，CAN codec 可以优先用 `drive_rpm`。
- 后续确认轮径/减速比后，再严格定义 `m/s <-> rpm` 转换。

`ForkliftVehicleState.msg` 建议：

```text
std_msgs/Header header

bool enabled
bool auto_mode
bool emergency_stopped
bool soft_emergency_stop
bool parking_brake
bool interlock

float64 velocity_mps
float64 left_drive_rpm
float64 right_drive_rpm
float64 steering_angle_rad
float64 steering_angle_deg

float64 battery_percent
float64 drive_controller_temperature_c
float64 lift_controller_temperature_c

string mode
```

`ForkliftFaultState.msg` 建议：

```text
std_msgs/Header header

uint8 left_drive_fault_code
uint8 right_drive_fault_code
uint8 steering_fault_code
uint8 lift_fault_code

bool has_fault
string summary
```

`ForkliftIoState.msg` 建议：

```text
std_msgs/Header header

bool lower_valve_output
bool lift_valve_output
bool retract_valve_output
bool extend_valve_output
bool side_shift_left_output
bool side_shift_right_output
bool tilt_forward_output
bool tilt_backward_output

bool brake_relay
bool reverse_relay
bool main_contactor
bool auto_mode_input
bool soft_emergency_stop_input
bool parking_brake_input
bool slowdown_switch_input
```

`SetControlMode.srv` 建议：

```text
string mode
---
bool success
string message
```

`SetEmergencyStop.srv` 建议：

```text
bool emergency_stop
---
bool success
string message
```

### 3.2 P1 验收

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
colcon build --packages-select forklift_msgs --symlink-install
source install/setup.bash

ros2 interface show forklift_msgs/msg/ForkliftControlCommand
ros2 interface show forklift_msgs/msg/ForkliftVehicleState
```

验收标准：

- `colcon build --packages-select forklift_msgs --symlink-install` 通过。
- `ros2 interface show` 能看到字段。
- 字段命名和单位明确，不把 Curtis 字节布局暴露给 planner/controller。

## 4. P2: 创建 forklift_vehicle_interface

新建 package：

```text
forklift_vehicle_interface
```

建议第一版用 Python 快速闭环，后续接 SocketCAN 时再决定是否换 C++：

```bash
cd /home/pl/robotest/src
ros2 pkg create forklift_vehicle_interface \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs nav_msgs tf2_ros forklift_msgs
```

### 4.1 第一版节点

建议文件：

```text
forklift_vehicle_interface/forklift_vehicle_interface/sim_command_bridge.py
forklift_vehicle_interface/forklift_vehicle_interface/curtis_can_codec.py
forklift_vehicle_interface/forklift_vehicle_interface/curtis_vehicle_interface.py
forklift_vehicle_interface/test/test_curtis_can_codec.py
```

第一版先做仿真 bridge：

```text
订阅:
  /forklift/control_cmd     forklift_msgs/msg/ForkliftControlCommand

发布:
  /cmd_vel                  geometry_msgs/msg/Twist
  /forklift/vehicle_state   forklift_msgs/msg/ForkliftVehicleState，可选调试
  /forklift/fault_state     forklift_msgs/msg/ForkliftFaultState，可选调试
```

转换规则第一版保守处理：

```text
if not enable:
  /cmd_vel = 0
elif brake:
  /cmd_vel = 0
elif emergency_stop:
  /cmd_vel = 0
else:
  linear.x = signed velocity_mps
  angular.z = tan(steering_angle_rad) * linear.x / wheel_base
```

方向处理：

```text
forward=true, reverse=false  -> 正速度
forward=false, reverse=true  -> 负速度
其他组合                    -> 0，记录 warning
```

参数建议：

```text
wheel_base: 1.2
max_velocity_mps: 0.4
max_steering_angle_rad: 0.55
command_timeout_sec: 0.5
publish_tf: false
```

注意：

- 当前 Gazebo 已经由 diff_drive 插件发布 `/odom` 和 `odom -> base_footprint`，仿真 bridge 第一版不要重复发布 odom/TF。
- 后续真车版 `forklift_vehicle_interface` 才负责由底盘反馈计算 `/odom` 和 TF。
- 仿真 bridge 的名字建议用 `sim_command_bridge`，避免误解成它负责 vehicle state estimation。

### 4.2 Curtis CAN codec

先不接真实 CAN，先写纯函数 codec 和测试。

建议函数：

```text
encode_0x203(command) -> list[int] length 8
encode_0x303(command) -> list[int] length 8
encode_0x403(command) -> list[int] length 8
decode_0x183(data) -> partial ForkliftVehicleState/FaultState
decode_0x283(data) -> ForkliftIoState
decode_0x383(data) -> steering/right drive feedback
decode_0x483(data) -> lift feedback
```

必须先写的测试：

PDF 示例 1：

```text
转向 45 deg，1500 rpm 前进，加速 3 s，减速 3 s

expected:
0x203 = 03 DC 05 1E 1E 00 94 11
0x303 = 00 00 00 00 00 00 00 00
0x403 = 00 00 00 00 00 00 00 00
```

PDF 示例 2：

```text
货叉起升，泵电机 1500 rpm，起升比例阀 400 mA

expected:
0x203 = 01 00 00 00 00 00 00 00
0x303 = DC 05 00 00 00 00 00 00
0x403 = 00 28 00 00 00 00 00 00
```

角度缩放：

```text
-90 deg -> -9000
0 deg   -> 0
45 deg  -> 4500 -> 0x1194 -> bytes 94 11
90 deg  -> 9000
```

阀电流缩放：

```text
0-2000 mA -> 0-200
400 mA -> 40 -> 0x28
```

### 4.3 P2 验收

构建：

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
colcon build --packages-select forklift_msgs forklift_vehicle_interface --symlink-install
source install/setup.bash
```

运行仿真：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/home/pl/robotest/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/home/pl/robotest/forklift_nav2_demo/config/forklift_nav2_oru_test.yaml \
  use_sim_time:=true \
  rmw_implementation:=rmw_fastrtps_cpp
```

运行 bridge：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run forklift_vehicle_interface sim_command_bridge \
  --ros-args -p use_sim_time:=true
```

直接发控制命令：

```bash
ros2 topic pub --once /forklift/control_cmd forklift_msgs/msg/ForkliftControlCommand "{
  enable: true,
  brake: false,
  forward: true,
  reverse: false,
  velocity_mps: 0.2,
  steering_angle_rad: 0.0,
  accel_time_sec: 3.0,
  decel_time_sec: 3.0
}"
```

检查：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /forklift/vehicle_state --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

验收标准：

- `/forklift/control_cmd` 能驱动车在仿真中低速前进。
- `enable=false` 或 `brake=true` 时输出 `/cmd_vel=0`。
- 命令超时后自动停车。
- codec 单元测试通过 PDF 两个示例。
- 不启动 Nav2 controller，只用 `/forklift/control_cmd` 也能驱动仿真车。

## 5. 完成后再做 ORU

P1/P2 完成后，再进入 ORU 算法移植会更稳：

```text
ORU MPC / Nav2 controller
  输出 ForkliftControlCommand
  不直接关心 CAN 字节

forklift_vehicle_interface
  仿真时转 /cmd_vel
  真车时转 Curtis CAN
```

这时再做：

```text
P3.1 写 forklift_vehicle_model
P4.1 把 ORU State / Control 概念移入 ForkliftMpcController
P4.2 把 Path 转成内部 Trajectory
P4.3 加 preview window
P4.4 接最小 QP/MPC 求解
```

## 6. 建议新窗口第一步

新窗口打开后先做：

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

sed -n '1,220p' forklift_nav2_demo/docs/p1_p2_vehicle_interface_next_steps.md
```

然后从 `P1: 创建 forklift_msgs` 开始，不要先改 controller。
