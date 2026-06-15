# P8.1 Safety Gate Notes

## 1. 目标

P8.1 先做最小安全闸门，不做复杂动态绕行。

目标行为：

- 动态障碍进入保护区时停车或限速。
- 障碍离开后，不需要重启 Nav2，controller 可继续给出速度。
- 急停输入能让车停止。
- 控制命令超时后 vehicle_interface / bridge 自动停车。

P8.1 不是最终独立 safety 架构。后续 P8.2 再考虑独立 `forklift_safety` package、`nav2_collision_monitor`、keepout mask、speed zone 和掉边保护。

## 2. 当前实现

当前 P8.1 分两层：

```text
ForkliftMpcController safety gate
  -> 基于 local costmap + footprint 的近场停车/限速

sim_command_bridge / vehicle_interface gate
  -> emergency stop service
  -> command timeout watchdog
```

controller safety gate 在每次 `computeVelocityCommands()` 中执行：

1. 判断当前 preview 是否是 reverse-intent。
2. forward path 检查车前方保护区。
3. reverse path 检查车后方保护区。
4. 沿当前运动方向按 `safety_sample_spacing` 采样 footprint。
5. 如果 footprint 碰到 lethal/inscribed/unknown blocked cost，记录最近障碍距离。
6. 障碍距离小于 `safety_stop_distance` 时直接输出 zero/brake。
7. 障碍距离在 slowdown zone 内时降低当前方向速度上限。
8. 没有障碍时不限制速度。

速度限制逻辑在 `forklift_safety_gate` helper 中单测，controller 负责把 local costmap 最近障碍距离喂给 helper。

## 3. 参数

ORU test 配置当前打开：

```yaml
safety_gate_enabled: true
safety_emergency_stop_active: false
safety_stop_distance: 0.55
safety_slowdown_distance: 1.25
safety_min_speed: 0.05
safety_sample_spacing: 0.10
```

参数含义：

- `safety_gate_enabled`：打开 controller-side safety gate。
- `safety_emergency_stop_active`：运行时可置 true，controller 立即停车。
- `safety_stop_distance`：最近障碍小于该距离时直接 brake。
- `safety_slowdown_distance`：最近障碍进入该距离后开始限速。
- `safety_min_speed`：slowdown zone 内的最低非零速度上限。
- `safety_sample_spacing`：沿运动方向采样 footprint 的距离间隔。

回退方式：

```yaml
safety_gate_enabled: false
```

或运行时触发 controller 急停：

```bash
ros2 param set /controller_server FollowPath.safety_emergency_stop_active true
```

仿真 bridge 急停：

```bash
ros2 service call /forklift/set_emergency_stop forklift_msgs/srv/SetEmergencyStop "{emergency_stop: true}"
```

## 4. 与 P6 倒车的关系

P6.4a 之后，controller 只在 reverse-intent preview window 中允许负速度。

P8.1 safety gate 复用这个方向判断：

- 普通 forward path：只看车前方保护区。
- reverse-intent path：只看车后方保护区。

这样不会因为打开倒车而在前进路线里检查错误方向，也不会让倒车路径忽略车后障碍。

## 5. 验证记录

Foxy docker 构建：

```text
Summary: 4 packages finished
```

Foxy docker 插件测试：

```text
100% tests passed, 0 tests failed out of 7
Summary: 59 tests, 0 errors, 0 failures, 0 skipped
test_forklift_safety_gate: 5 tests passed
```

`test_forklift_safety_gate` 覆盖：

- safety gate 关闭时不限制速度。
- 障碍进入 stop distance 时停车。
- 障碍进入 slowdown distance 时限速。
- 障碍离开保护区后恢复速度上限。
- 参数 sanitization。

Foxy headless `sparse_90_turn` safety gate 回归：

```text
/follow_path status: 4 SUCCEEDED
control_samples=560 max_velocity_mps=0.450
control_direction_samples forward=560 reverse=0
sim_cmd_samples=1128 max_linear_x=0.450 min_signed_linear_x=0.000 max_signed_linear_x=0.450
```

launch 日志确认：

```text
safety_gate=true safety_stop=0.550 safety_slowdown=1.250
```

bridge watchdog 日志确认命令结束后停车：

```text
Stopping: command timeout.
```

## 6. 后续

P8.1 之后建议先做 A-B acceptance：

```text
NavigateToPose 简单 A-B
障碍进入保护区停车/限速
障碍离开后继续或重新下发目标
```

P8.2 再做独立 safety package：

- 独立订阅 raw control command，输出 gated command。
- 接入急停硬输入或真车 safety relay。
- 接入 keepout / speed zone。
- 掉边保护。
- 与 task_manager 的 pause/resume/replan 状态联动。
