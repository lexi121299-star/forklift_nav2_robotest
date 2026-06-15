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
  -> optional recovery Twist fallback in simulation bridge mode
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

bridge 模式下 Gazebo 订阅 `/forklift/sim_cmd_vel`，正常控制链路是：

```text
ForkliftMpcController
  -> /forklift/control_cmd
  -> sim_command_bridge
  -> /forklift/sim_cmd_vel
```

Nav2 recovery actions，例如 `Spin` / `BackUp`，默认发布 `geometry_msgs/Twist` 到 `/cmd_vel`。如果 bridge 模式只监听 `/forklift/control_cmd`，recovery 会在 BT 里运行，但 Gazebo 车不会实际转动。当前修复是在 `sim_command_bridge` 中加入可选 fallback：

```text
/cmd_vel
  -> sim_command_bridge twist_fallback_topic
  -> /forklift/sim_cmd_vel
```

fallback 只在 `/forklift/control_cmd` 未到达或超过 `command_timeout_sec` 时启用；只要 controller 正常发布 `ForkliftControlCommand`，仍然优先使用统一车辆命令。

### 真车 recovery 注意事项

仿真 bridge fallback 不是最终真车方案。它解决的是 Gazebo bridge 模式下 Nav2 recovery `/cmd_vel` 没有接到 `/forklift/sim_cmd_vel` 的仿真接线问题。

真车上仍然需要 recovery，但不能让底盘裸订阅 Nav2 默认 `/cmd_vel`。真车推荐链路是：

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

P8.2 要解决“recovery 命令怎么安全进入真车”：

- 所有 normal control、manual control、recovery control 都进入同一个安全命令闸门。
- recovery command adapter 只输出叉车约束下允许的 `ForkliftControlCommand` 或等价安全命令。
- 第一版 recovery 白名单建议只包含 wait、clear-costmap 后重试、低速短时 pivot/backoff。
- 每条 recovery 命令都必须受急停、watchdog、限速、footprint collision、车辆状态和传感器健康检查约束。

P8.3 要解决“什么时候执行哪一种 recovery”：

- 动态障碍短时挡路优先 wait。
- 障碍离开后优先继续或 replan。
- 持续阻挡时再进入 clear/replan 或任务暂停。
- 只有安全闸门允许时才执行低速 pivot/backoff。
- recovery 失败不能无限循环，要进入可诊断的暂停/失败状态。

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

仿真 bridge recovery fallback：

```yaml
bridge_twist_fallback_topic: /cmd_vel
bridge_twist_fallback_timeout_sec: 0.5
```

回退方式：

```yaml
bridge_twist_fallback_topic: ''
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

Foxy headless A-B 动态障碍停车/放行快速验证：

```bash
ros2 run forklift_nav2_demo forklift_ab_dynamic_obstacle_acceptance \
  --ros-args -p use_sim_time:=true -p timeout_sec:=150.0
```

脚本行为：

```text
NavigateToPose: (-2.0, -0.5) -> (1.2, -0.5)
spawn obstacle: (-0.6, -0.5)
observe blocked stop samples
delete obstacle
wait for automatic Nav2 recovery/replan to finish the same goal
```

结果：

```text
/navigate_to_pose status: 4 SUCCEEDED
control_samples=399 sim_cmd_samples=1136 forward=94 reverse=0
phase_control_zero blocked=4
phase_sim_max after=0.450
dynamic_obstacle_acceptance=PASS
```

关键日志：

```text
P8.1 safety gate stopping: obstacle at 0.100 m in forward protection zone
Using fallback Twist command.
Navigation succeeded
```

备注：短距离目标 `(0.2, -0.5)` 也能验证 fallback 已生效，但 recovery 后容易越过短目标并触发后续 planner / progress checker 边界条件；正式 quick acceptance 使用较长 A-B 目标 `(1.2, -0.5)`。

## 6. 后续

P8.1 之后建议先做 A-B acceptance：

```text
NavigateToPose 简单 A-B
障碍进入保护区停车/限速
障碍离开后继续或重新下发目标
```

P8.2 再做独立 safety package：

- 独立订阅 raw control command，输出 gated command。
- 接入 recovery command adapter，禁止真车底盘裸吃 Nav2 默认 `/cmd_vel`。
- 接入急停硬输入或真车 safety relay。
- 接入 keepout / speed zone。
- 掉边保护。
- 与 task_manager 的 pause/resume/replan 状态联动。
