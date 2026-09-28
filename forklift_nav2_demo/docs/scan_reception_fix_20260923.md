# Scan 接收与超时诊断修复（2026-09-23）

## 证据与边界

9 月 22 日四个包的 Safety Gate 分别记录了 17、16、20、19 次 `scan timeout`。
这些短暂停车期间，上游 MPC 往往仍在发前进命令。

用录包接收时间更新最新 scan，并按 0.7 s 门限离线检查，每一个上述事件发生时，
录包侧最新 scan 均未过期。这只说明不能把这些停车全部解释成感知断流；
录包接收时间不等于 Safety Gate 回调实际执行时间，也没有录到车端全部生效参数。
原始原因仍需区分旧启动参数覆盖和节点处理滞后，不能仅凭 bag 断言 DDS 或感知异常。

代码中确认存在一条会导致处理滞后的路径：scan 回调与耗时的碰撞检查定时器
使用同一互斥 callback group。即使 executor 有多个线程，检查期间也不能更新 scan。

## 改动

- scan 使用独立互斥 callback group，订阅深度仍为 1，只保留最新待处理数据。
- executor 使用 3 个线程，分别允许停车指令、scan 接收与碰撞检查并发执行。
- scan 消息与其接收时间在短锁内成对更新、成对读取；投影与碰撞计算不持有该锁。
- 投影缓存仅由碰撞定时器维护，以消息对象身份区分帧。scan 回调不再清空正在使用的缓存。
- 碰撞计算后重新检查实际使用帧的年龄，以及当前最新帧的有效性；新帧不能给旧计算结果续期。
- 发运动命令前重新检查高速数据新鲜度，必要时维持原来的 1.0 m/s 降级上限。
- 启动日志打印实际停车门限、高速新鲜度门限、降级速度和碰撞计算预算。
- `scan timeout` 时增加 `receive_age`、`source_age`、`source_stamp`、生效门限和检查阶段日志。

不修改 footprint、托盘豁免、真实障碍物碰撞判断和紧急停止。不改 MPC、规划器或转角。
碰撞计算超过原有 0.15 s 预算仍然停车，没有通过放宽计算预算掩盖卡顿。

## 门限不变

```text
scan_timeout_sec = 0.7
scan_high_speed_freshness_timeout_sec = 0.25
scan_degraded_max_speed_mps = 1.0
collision_compute_budget_sec = 0.15
```

source 时间戳与接收时间都用于判断；重复收到旧时间戳不会延长有效期。
0.25 s 是高速降级门限，不是普通速度下的硬停车门限。无 scan、真正过期、
无效时间戳、TF 不可用或量程不足依然停止。

## 验证

- Foxy Docker 构建 Safety 成功。
- Safety 回归 51 tests，0 errors，0 failures。
- 真实 ROS executor 测试：故意阻塞碰撞回调约 0.9 s，期间持续发送新 scan；
  在计算释放前仍能读取最新帧，释放后的新鲜度检查通过。
- 测试包括旧源时间戳反复接收、未来或零时间戳、真实断流、量程不足、
  计算期间旧帧过期、新帧无效、计算期间跨越高速新鲜度门限，以及并发停车优先级。
- 四包的离线验证仅重放新鲜度检查，不重放整车运动或完整碰撞几何，
  不能宣称实车所有卡顿均已消失。CPU 过载或真正数据缺失仍需要处理。

## 上传与编译

与前一项托盘完成判定修复一起部署，只需更新 Safety 和 Task Manager：

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_safety \
  /home/pl/robotest/forklift_task_manager \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/
```

车端 Docker，停止旧导航与任务节点后：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select forklift_safety forklift_task_manager
source install/setup.bash
```

重启原来的 launch。原有 real navigation launch 已支持 `scan_timeout_sec` 参数，
建议在车端启动命令中显式设置 `scan_timeout_sec:=0.7`，排除旧脚本覆盖。
本次无需更新消息接口或编译 Nav2 plugin。

启动后在相同 ROS 环境确认：

```bash
ros2 param get /safety_command_gate scan_timeout_sec
ros2 param get /safety_command_gate scan_high_speed_freshness_timeout_sec
ros2 param get /safety_command_gate collision_compute_budget_sec
```

日志应包含 `scan_callback=independent latest-only`。
若仍超时，用新日志区分：`source_age` 大表示数据时间戳陈旧，`receive_age` 大表示
本节点很久未处理新 scan。两者均不能单独证明 DDS 故障。真正停车事件不应通过
关闭安全门或忽略超时来消除。
