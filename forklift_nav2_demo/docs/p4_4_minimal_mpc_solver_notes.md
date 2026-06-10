# P4.4 Minimal MPC Solver Notes

日期：2026-06-10

## 1. 本轮目标

P4.4 先接一个最小受约束 MPC 求解层，不直接引入完整 `qpOASES`。

新增文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_solver.hpp
forklift_nav2_plugins/src/forklift_mpc_solver.cpp
forklift_nav2_plugins/test/test_forklift_mpc_solver.cpp
```

修改文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
forklift_nav2_plugins/CMakeLists.txt
forklift_nav2_demo/config/forklift_nav2_oru_test.yaml
```

## 2. 当前求解器做什么

`solveMpcCommand()` 输入：

```text
MpcPreviewWindow
当前 MpcState
当前 Twist
ForkliftVehicleModel
MpcSolverParameters
```

输出：

```text
MpcControl(v, w)
ForkliftVehicleCommand(velocity, steering_angle)
score
valid
```

这里：

```text
v = 参考点切向速度
w = 转向角速度 phi_dot
```

## 3. 当前约束

求解器枚举候选 `v` 和 `w`，然后通过 `ForkliftVehicleModel` 限幅：

```text
速度限制 max_velocity / max_reverse_velocity
转向角限制 max_steering_angle
转向角速度限制 max_steering_angle_velocity
终点附近允许 v=0
```

当前还不是严格数学 QP，只是一个确定性的受约束候选优化器。这样可以先把 MPC 数据流跑通，再决定是否把 ORU 的 `qpProblem / qpConstraints / qpOASES` 继续移入。

## 4. 评分项

每个候选控制会用车辆模型在 preview window 上预测，并评分：

```text
path distance error
heading error
steering angle error
terminal distance error
smoothness relative to current velocity
velocity reward
```

得分最低的候选输出为 `MpcControl`。

## 5. Controller 接入方式

新增参数：

```yaml
use_mpc_solver: true
```

`ForkliftMpcController` 每周期：

```text
1. 构造当前 MpcState
2. 构造 MpcPreviewWindow
3. 调用 solveMpcCommand()
4. 把 solver 输出转成现有候选 command
5. 用原有 collision/path scoring 验证
6. 若 solver 候选不可用，则回退到原有采样搜索
```

这样 P4.4 接入了 solver，但仍保留现有碰撞检查和回退路径。

运行时可见日志：

```text
Configured FollowPath ... use_mpc_solver=true ...
MPC preview window: start=<index> end=<index> points=<count> length=<meters>
MPC solver seed accepted: v=<m/s> w=<rad/s> steer=<rad> solver_score=<score>
```

## 6. 验收命令

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
invalid preview window
直线路径输出前进且转向角速度近 0
左转窗口输出正转向角速度
速度、转角、转向角速度限幅
终点容差内允许停车
```

## 7. 短直线 FollowPath 冒烟

已用 bridge 模式复跑短直线：

```text
map frame path:
(-2.0, -0.5) -> (-1.7, -0.5) -> (-1.3, -0.5)
```

结果：

```text
/follow_path accepted: true
/follow_path status: SUCCEEDED
/forklift/control_cmd samples: 23
/forklift/control_cmd max velocity_mps: 0.3942857142857143
/forklift/control_cmd final velocity_mps: 0.0
/forklift/control_cmd final brake: true
/forklift/sim_cmd_vel samples: 74
/forklift/sim_cmd_vel max linear.x: 0.3942857142857143
/odom final sample: x=-1.485110, y=-0.504875, linear.x near 0
```

controller 日志已确认 solver 参与：

```text
Configured FollowPath ... use_mpc_solver=true ...
MPC preview window: start=0 end=2 points=3 length=0.700
MPC solver seed accepted: v=0.450 w=0.000 steer=0.000 solver_score=4.035
Reached the goal!
```

日志：

```text
log/p4_4_smoke/nav2_bridge_launch.log
log/p4_4_smoke/follow_path_direct_client.log
```

## 8. 后续

P4.5 已继续验证 Nav2 Controller API 接入，详见：

```text
forklift_nav2_demo/docs/p4_5_nav2_controller_api_notes.md
```

后续可以在 P5/P6 再增强：

```text
trajectory smoothing / path preprocessing
增加沿 preview window 的 footprint collision check
逐步评估是否移植 ORU qpProblem / qpConstraints / qpOASES
```
