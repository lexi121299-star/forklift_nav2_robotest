# P4.3 Preview Window Notes

日期：2026-06-10

## 1. 本轮目标

P4.3 基于 P4.2 的 `MpcTrajectory` 增加 preview window。这个窗口表示当前车辆前方的一段有限轨迹，后续 P4.4 QP/MPC 只需要对这个窗口求解。

新增文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_preview_window.hpp
forklift_nav2_plugins/src/forklift_mpc_preview_window.cpp
forklift_nav2_plugins/test/test_forklift_mpc_preview_window.cpp
```

修改文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
forklift_nav2_plugins/CMakeLists.txt
forklift_nav2_demo/config/forklift_nav2_oru_test.yaml
```

## 2. Preview Window 定义

`MpcPreviewWindow`：

```text
points       从 trajectory 截取出来的有限轨迹点
start_index  窗口起点在完整 trajectory 中的 index
end_index    窗口终点在完整 trajectory 中的 index
length       窗口起点到终点的累计路径长度差，单位 m
valid        是否成功生成窗口
```

`MpcPreviewWindowOptions`：

```text
max_points  从最近轨迹点开始最多截取多少个点
```

## 3. 截取规则

当前函数：

```text
makeMpcPreviewWindow()
makeMpcPreviewWindowFromIndex()
```

规则：

```text
空 trajectory -> invalid window
当前 state -> nearestTrajectoryIndex()
start_index -> 当前最近 trajectory 点
end_index -> min(start_index + max_points - 1, trajectory.size() - 1)
length -> trajectory[end].distance - trajectory[start].distance
```

如果 `max_points=0`，内部会按 1 个点处理，避免空窗口。

## 4. Controller 接入方式

新增 controller 参数：

```yaml
preview_window_points: 10
```

`computeVelocityCommands()` 每周期会从 transformed trajectory 和当前 `MpcState` 生成 `last_preview_window_`。

当前仍不改变控制输出，只增加有限窗口结构和日志可见性。运行时会节流打印：

```text
MPC preview window: start=<index> end=<index> points=<count> length=<meters>
```

这一步的目的不是让控制变聪明，而是给 P4.4 QP/MPC 准备稳定输入。

## 5. 验收命令

构建：

```bash
cd /home/pl/robotest
source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --packages-select forklift_nav2_plugins forklift_nav2_demo \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon test --packages-select forklift_nav2_plugins --event-handlers console_cohesion+
```

单元测试覆盖：

```text
空 trajectory -> invalid window
从最近点开始截取 N 个点
靠近 trajectory 尾部时 end_index 自动限幅
max_points=0 时仍返回 1 个点
start_index 超出末尾时使用最后一个点
```

## 6. 短直线 FollowPath 冒烟

已用 bridge 模式复跑短直线：

```text
map frame path:
(-2.0, -0.5) -> (-1.7, -0.5) -> (-1.3, -0.5)
```

结果：

```text
/follow_path accepted: true
/follow_path status: SUCCEEDED
/forklift/control_cmd samples: 21
/forklift/control_cmd max velocity_mps: 0.3942857142857143
/forklift/control_cmd final velocity_mps: 0.0
/forklift/control_cmd final brake: true
/forklift/sim_cmd_vel samples: 71
/forklift/sim_cmd_vel max linear.x: 0.3942857142857143
/odom final sample: x=-1.474961, y=-0.504639, linear.x near 0
```

controller 日志已显示 preview window：

```text
Configured FollowPath ... preview_points=10 ...
MPC preview window: start=0 end=2 points=3 length=0.700
Reached the goal!
```

日志：

```text
log/p4_3_smoke/nav2_bridge_launch.log
log/p4_3_smoke/follow_path_direct_client.log
```

## 7. 下一步

P4.4：接最小 QP/MPC 求解。

建议先做最小版本：

```text
输入：MpcPreviewWindow + 当前 MpcState + 当前速度
输出：MpcControl(v, w)
约束：速度、转角、转向角速度
暂不做复杂任务状态、多机器人约束
```
