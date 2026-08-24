# 柯锐 CANopen 拉线编码器实车测试指南

## 1. 测试目的

本文用于测试已经安装在叉车上的柯锐拉线编码器，并收集实现叉臂高度闭环所需的信息。

本次测试需要确认：

- 编码器实际使用的 CAN 波特率和 Node ID。
- 心跳报文和位置报文是否正常。
- TPDO 的 CAN ID、数据长度和位置字段布局。
- 编码器计数与实际叉高的换算关系。
- 位置反馈的方向、周期、稳定性、重复性和有效范围。

首次测试以监听为主，只允许发送指定节点的 NMT 启动命令和只读 SDO 查询。不要在本次测试中修改或保存编码器参数。

## 2. 手册中的已知默认值

| 项目 | 出厂默认值 |
| --- | --- |
| CAN 波特率 | 250 kbit/s |
| Node ID | `0x3F`，十进制 63 |
| 终端电阻 | 开启 |
| 心跳/上电报文 ID | `0x700 + Node ID`，默认 `0x73F` |
| SDO 请求 ID | `0x600 + Node ID`，默认 `0x63F` |
| SDO 响应 ID | `0x580 + Node ID`，默认 `0x5BF` |
| 可能的 TPDO1 ID | 通常为 `0x180 + Node ID`，默认 `0x1BF`，必须实测确认 |

手册中的 M12 五芯接线定义：

| 针脚 | 信号 |
| --- | --- |
| Pin 1 | CAN_GND |
| Pin 2 | +V 电源 |
| Pin 3 | 0V 电源 |
| Pin 4 | CAN_H |
| Pin 5 | CAN_L |

手册没有明确给出 TPDO 数据映射、数据有无符号、物理分辨率和编码器供电电压。这些信息需要通过 EDS 文件、厂家回复或实车标定确认。

## 3. 安全要求

开始测试前：

1. 车辆停在平整、空旷区域。
2. 货叉空载，不带托盘。
3. 人员远离门架、链条、油缸、货叉和拉线区域。
4. 由熟悉车辆的操作人员看守急停。
5. 第一次升降使用车辆人工控制，电脑只监听 CAN 报文。
6. 车辆控制器运行时，不要断开或重新配置 `can0`。
7. 不要发送全节点启动命令 `000#0100`，它会操作总线上的所有 CANopen 设备。
8. 本次测试不要写入 `0x6000`、`0x6003`、`0x1010` 或通信配置对象。

编码器出厂时终端电阻默认开启。整条 CAN 总线断电后，正常的两个 120 欧姆终端电阻并联，CAN_H 与 CAN_L 之间通常约为 60 欧姆。如果车辆总线已经有两个终端电阻，再启用编码器内部终端电阻可能影响通信。电阻测量和接线调整必须断电，并由具备电气操作能力的人员完成。

## 4. 创建测试目录

在车载电脑执行：

```bash
mkdir -p ~/encoder_test
cd ~/encoder_test
date
```

确认已安装 `can-utils`：

```bash
which candump
which cansend
```

实车正在运行时，不要执行 `ip link set can0 down`。

## 5. 检查现有 CAN 接口

```bash
ip -details -statistics link show can0 | tee can0_before.txt
```

检查：

- 接口是否为 `UP`。
- 当前波特率是否与车辆 CAN 总线一致。
- `bus-off`、error、dropped 或 overrun 是否持续增加。

编码器出厂波特率为 250 kbit/s，但装车后可能已被修改。同一条物理总线上的设备必须使用相同波特率。不要为了匹配手册而直接修改正在使用的车辆总线。

## 6. 查找编码器 Node ID

监听 CANopen 心跳范围：

```bash
candump -tz can0,700:780 | tee heartbeat.txt
```

如果接线允许，可以只给编码器重新上电；否则直接观察现有心跳。出厂 Node ID 为 `0x3F` 时，应看到 CAN ID `0x73F`。

常见的一字节 CANopen 状态：

| Data | 含义 |
| --- | --- |
| `00` | Boot-up，上电启动 |
| `04` | Stopped |
| `05` | Operational |
| `7F` | Pre-operational |

如果没有看到心跳，记录 10 秒全部报文：

```bash
timeout 10 candump -tz can0 | tee all_can_10s.txt
```

如果发现了其他心跳 ID，不要继续假设 Node ID 是 `0x3F`。对于实际 Node ID `N`：

```text
心跳 ID     = 0x700 + N
SDO 请求 ID = 0x600 + N
SDO 响应 ID = 0x580 + N
默认 TPDO1  = 0x180 + N
```

## 7. 只启动编码器节点

如果编码器已经是 Operational 状态并持续发送位置报文，跳过本步骤。

Node ID 为 `0x3F` 时，只启动该节点：

```bash
cansend can0 000#013F
```

再次观察心跳：

```bash
timeout 5 candump -tz can0,73F:7FF | tee heartbeat_after_nmt.txt
```

进入 Operational 后，心跳 Data 通常为 `05`。

如果实际 Node ID 不是 `0x3F`，需要替换 NMT 命令的第二个数据字节。例如 Node ID 为 `0x05`：

```bash
cansend can0 000#0105
```

## 8. 查找 TPDO 位置报文

出厂 Node ID 为 `0x3F` 时，先监听可能的 TPDO1：

```bash
candump -tz can0,1BF:7FF | tee tpdo1.txt
```

操作人员使用车辆人工控制，让空载货叉在安全范围内缓慢升降一小段距离，观察报文中是否有字节随叉高连续变化。

如果 `0x1BF` 没有报文，可以使用变化显示查找报文：

```bash
cansniffer -c can0
```

按照下面的顺序重复观察：

```text
静止 -> 上升 -> 静止 -> 下降 -> 静止
```

只有同一个字段能够稳定跟随叉臂运动时，才能初步认为它是位置数据。不能因为某条报文发生变化就直接确定它是编码器位置。

## 9. 只读查询标准位置对象

以下是 SDO 上传请求，只读取参数，不修改参数。命令假设 Node ID 为 `0x3F`。

先在一个终端监听 SDO 响应：

```bash
candump -tz can0,5BF:7FF | tee sdo_responses.txt
```

在另一个终端读取标准当前位置对象 `0x6004:00`：

```bash
cansend can0 63F#4004600000000000
```

四字节快速响应通常类似：

```text
5BF#43046000AABBCCDD
```

其中位置数据按小端排列，计数值为：

```text
0xDDCCBBAA
```

把实际收到的四个数据字节替换到下面命令中即可转换为十进制：

```bash
python3 -c "print(int.from_bytes(bytes.fromhex('AA BB CC DD'), 'little', signed=False))"
```

继续只读查询标准 Scaling 对象：

```bash
cansend can0 63F#4001600000000000
cansend can0 63F#4002600000000000
```

它们分别查询 `0x6001:00` 和 `0x6002:00`。保存完整响应。

如果 SDO 响应以 `80` 开头，表示 SDO Abort。保存完整八字节，因为后四字节是错误码。`0x6004` 不响应或返回 Abort，不一定表示编码器故障，也可能是该型号对象字典或 PDO 映射不同。

## 10. 录制高度标定数据

开始录制完整 CAN 数据：

```bash
cd ~/encoder_test
candump -L can0 | tee kuebler_height.log
```

空载情况下，至少测试五个高度。每到一个高度，保持静止 5 至 10 秒。

建议测试点：

1. 最低安全叉高。
2. 最低位置以上约 100 mm。
3. 最低位置以上约 500 mm。
4. 中间高度。
5. 本次测试允许达到的最高安全高度。

每次都从车辆上的同一个固定基准测量实际叉高。至少选择两个高度，分别在上升过程和下降过程中重复测量，用于检查机械回差和计数重复性。

记录表：

| 测试点 | 运动方向 | 实际叉高 m | 编码器计数 | 静止最小值 | 静止最大值 | 备注 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 1 | 上升/下降 | | | | | |
| 2 | 上升 | | | | | |
| 3 | 上升 | | | | | |
| 4 | 上升 | | | | | |
| 5 | 上升 | | | | | |
| 4 重复 | 下降 | | | | | |
| 2 重复 | 下降 | | | | | |

完成后按 `Ctrl+C` 停止 `candump`。

## 11. 计算初始换算系数

选择两个距离较远且数据有效的测试点：

```text
meter_per_count =
    (height_2_m - height_1_m) / (count_2 - count_1)
```

高度换算公式：

```text
height_m =
    (count - zero_count) * meter_per_count + zero_height_m
```

如果叉臂升高时计数减小，计算出的 `meter_per_count` 会是负数，软件可以直接处理。本次测试不要为了改变符号而修改编码器方向。

使用实测叉高标定时，滑轮、链条等机械倍率已经包含在换算系数中。需要使用多个测试点验证整个行程内是否保持线性。

手册提示，拉线完全回缩时若将预设值设为零，钢丝弹性造成的小负值可能以无符号整数形式回绕到接近 `2^32` 的大数。测试最低位置时要特别观察是否发生回绕，不要在本次测试中写入新的预设值。

## 12. 检查周期和稳定性

根据带时间戳的 TPDO 日志检查：

- 计数方向是否与叉臂高度变化一致。
- 匀速升降时计数是否基本单调。
- 正常行程中是否出现大幅跳变或整数回绕。
- 静止时计数波动能否满足叉高控制精度。
- 从上升和下降两个方向回到同一高度时，计数是否可重复。
- 是否存在长时间断报。

当前叉臂控制建议位置反馈周期为 20 至 50 ms。若发送周期超过 adapter 的 `feedback_timeout_sec`，会被判断为反馈丢失。先测量现有周期，不要立即修改 `0x1800:05`。

## 13. 测试结束检查

再次检查 CAN 统计：

```bash
ip -details -statistics link show can0 | tee can0_after.txt
```

比较 `can0_before.txt` 和 `can0_after.txt`。本次测试不应造成 bus-off，也不应造成错误计数持续增加。

本次测试不要执行参数保存。确认 Node ID、波特率、TPDO 映射、换算系数和终端电阻拓扑后，再单独进行参数配置和断电重启验证。

## 14. 测试后需要提供的资料

完成后保留并提供：

- `can0_before.txt`
- `can0_after.txt`
- `heartbeat.txt`
- `heartbeat_after_nmt.txt`，如果执行过 NMT 启动
- `tpdo1.txt`，如果存在
- `sdo_responses.txt`
- `all_can_10s.txt`，如果 Node ID 不明确
- `kuebler_height.log`
- 填写完成的高度标定表
- 编码器型号铭牌照片
- EDS 文件，如果可以取得

后续 ROS 2 编码器节点需要发布：

```text
/forklift/fork/joint_state
  sensor_msgs/msg/JointState
  name: [fork_height_m]
  position: [height_m]
```

确认实车日志后，再实现独立的柯锐 CANopen 编码器节点：解析位置计数、应用标定公式、检测反馈超时和异常值，并把高度发布给 `fork_control_adapter`。
