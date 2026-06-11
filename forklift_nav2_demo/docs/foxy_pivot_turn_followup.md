# Foxy 原地 90 度转向改动说明与续做指南

日期：2026-06-11

## 1. 背景

当前 Foxy docker 环境里，90 度直角转弯失败，不是单纯调参问题，而是“仿真运动学假设”和“真实车控制协议能力”不一致：

- 真实协议允许 `steering_angle` 到 `±90 deg`
- 当前 controller / vehicle model / sim bridge 一直按 bicycle / Ackermann 曲率模型处理
- 旧模型里：

```text
omega = v * tan(steering_angle) / wheel_base
```

这意味着：

- `v = 0` 时永远不能原地转
- 90 度直角会被当成“极小转弯半径曲线”，而不是“允许原地 pivot 转向”

因此这次改动的目标是：

```text
当 steering_angle 接近 ±90 deg 且命令速度非零时，
在仿真里把它解释成原地旋转，而不是继续走有限半径圆弧。
```

## 2. 这次已经改了什么

### 2.1 共享车辆模型增加 pivot turn 分支

改动文件：

- `forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_vehicle_model.hpp`
- `forklift_nav2_plugins/src/forklift_vehicle_model.cpp`

新增参数：

```text
allow_pivot_turn
pivot_steering_angle
pivot_steering_tolerance
pivot_turn_radius
```

新增/修改逻辑：

- `isPivotTurnCommand()`
- `linearVelocity()`
- `angularVelocity()`
- `twistFromCommand()`
- `predict()`

新的语义是：

```text
如果允许 pivot turn，
且 |steering_angle| 接近 pivot_steering_angle（默认 90 deg），
且 |velocity| > 0，
则：
  linear.x = 0
  angular.z = velocity / pivot_turn_radius
  角速度符号由 steering_angle 和前后方向共同决定
否则仍按旧 bicycle 模型
```

另外还把最大转角上限从旧的 `1.4 rad` 放宽到 `pi/2`，否则配置里的 `1.570796...` 会被内部截回去。

### 2.2 MPC 状态预测改成复用车辆模型的真实线速度

改动文件：

- `forklift_nav2_plugins/src/forklift_mpc_types.cpp`

原来 `predictMpcState()` 直接用 `command.velocity` 推进 `x/y`。

现在改成：

```text
先问 vehicle_model.linearVelocity(command)
```

这样 pivot 时 MPC 预测只转 `theta`，不会错误地向前漂移。

### 2.3 Controller 参数层增加 pivot 配置透传

改动文件：

- `forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_mpc_controller.hpp`
- `forklift_nav2_plugins/src/forklift_mpc_controller.cpp`

已经做的事：

- 给 `ForkliftMpcController` 增加 pivot 参数成员
- 在 `configure()` 里声明、读取这些参数
- 传给 `ForkliftVehicleModel`
- 日志里打印 pivot 配置

### 2.4 轨迹预处理层允许把高曲率点映射成 90 度转向参考

改动文件：

- `forklift_nav2_plugins/src/forklift_mpc_trajectory.cpp`

已经做的事：

- `steeringFromCurvature()` 增加 pivot 分支
- 当曲率绝对值高于 `1 / pivot_turn_radius` 时，参考转向角直接给到 `±pivot_steering_angle`
- `trajectorySpeedLimit()` 对这种点不再强制压到最小速度，而是允许保持 `max_velocity`

目的不是直接“插入 spin 节点”，而是先让高曲率角点至少能生成 `±90°` 的 steering reference，给 controller 一个进入 pivot 分支的机会。

### 2.5 仿真 bridge 支持把 90 度 steering 命令映射成 Gazebo 原地旋转

改动文件：

- `forklift_vehicle_interface/forklift_vehicle_interface/sim_command_bridge.py`

新增参数：

```text
max_angular_velocity_radps
allow_pivot_turn
pivot_steering_angle_rad
pivot_steering_tolerance_rad
pivot_turn_radius
```

新增逻辑：

- `_is_pivot_turn()`
- 当 steering 接近 90 度时：

```text
twist.linear.x = 0.0
twist.angular.z = direction * speed / pivot_turn_radius
```

- 最后再按 `max_angular_velocity_radps` 限幅

另外修了一个小坑：

- launch 传进来的 bool 参数可能是字符串
- 不能直接 `bool("false")`
- 所以加了 `_bool_param()` 做显式解析

### 2.6 Launch 默认值改成和真车协议一致

改动文件：

- `forklift_vehicle_interface/launch/sim_command_bridge.launch.py`
- `forklift_nav2_demo/launch/forklift_gazebo.launch.py`
- `forklift_nav2_demo/launch/forklift_navigation.launch.py`

已改内容：

- 透传 bridge pivot 参数
- 默认 `max_steering_angle_rad = 1.5707963267948966`
- 默认 `allow_pivot_turn = true`

原因：

如果顶层 launch 还把 bridge 默认值写成 `0.55` 和 `false`，那就会覆盖掉 controller 侧的新能力，导致你 YAML 里开了 90 度，bridge 还是按旧模型执行。

### 2.7 ORU 测试配置切到 90 度 steering 能力

改动文件：

- `forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml`
- `forklift_nav2_demo/config/forklift_nav2_oru_test.yaml`

当前配置改成：

```yaml
max_steering_angle: 1.5707963267948966
max_steering_angle_velocity: 1.6
allow_pivot_turn: true
pivot_steering_angle: 1.5707963267948966
pivot_steering_tolerance: 0.03
pivot_turn_radius: 0.6
```

这里把 `max_steering_angle_velocity` 提高到 `1.6` 的原因是：

- 旧值 `0.7 rad/s`
- 从 0 打到 90 度需要约 `2.24 s`
- 而 controller 预测窗是 `1.8 s`

旧值下 MPC 在预测窗内很难“看见”完整 pivot 动作。

### 2.8 单元测试已补，但还没跑通验证

改动文件：

- `forklift_nav2_plugins/CMakeLists.txt`
- `forklift_nav2_plugins/test/test_forklift_vehicle_model.cpp`（新文件）
- `forklift_nav2_plugins/test/test_forklift_mpc_types.cpp`
- `forklift_nav2_plugins/test/test_forklift_mpc_trajectory.cpp`

新增测试意图：

- 默认 bicycle 模型不变
- pivot 命令时 `linear.x = 0`
- pivot 命令时 `angular.z != 0`
- `predictMpcState()` pivot 时只改 yaw，不改 x/y
- 高曲率 90 度角点能生成 `±pi/2` 的 steering reference

## 3. 这次没有改什么

下面这些这次没有改：

- `ForkliftControlCommand.msg` 字段结构没改
- CAN codec 没改
- Gazebo 的 `libgazebo_ros_diff_drive.so` 没改
- planner 没做显式“插入原地旋转节点”
- costmap / footprint / collision checker 没调

也就是说，这次走的是：

```text
尽量复用现有命令协议和 Gazebo diff_drive，
只把 90 度 steering 的语义从“极小半径弧线”改成“允许原地旋转”
```

## 4. 当前状态

代码已经改到仓库里，但我没有完成最终运行验证。

### 4.1 我已经做过但不要当成验证通过

我尝试过在宿主机直接 `colcon build`，遇到两个问题：

1. `ccache` 默认临时目录在只读路径，需要改到 `/tmp`
2. 宿主机当前 ROS 环境是 Humble，而这条线目标是 Foxy，host build 会撞到 `nav2_core` API 签名不一致

所以：

```text
不要用宿主机 Humble 结果判断这次改动是否成功。
应当在 Foxy docker 里继续。
```

## 5. 新窗口里建议怎么继续

### 5.1 先看这份文档和当前 diff

建议新窗口先做：

```bash
cd /home/pl/robotest
git status
git diff --stat
```

重点关注这些文件：

```text
forklift_nav2_plugins/include/forklift_nav2_plugins/forklift_vehicle_model.hpp
forklift_nav2_plugins/src/forklift_vehicle_model.cpp
forklift_nav2_plugins/src/forklift_mpc_types.cpp
forklift_nav2_plugins/src/forklift_mpc_trajectory.cpp
forklift_nav2_plugins/src/forklift_mpc_controller.cpp
forklift_vehicle_interface/forklift_vehicle_interface/sim_command_bridge.py
forklift_vehicle_interface/launch/sim_command_bridge.launch.py
forklift_nav2_demo/launch/forklift_gazebo.launch.py
forklift_nav2_demo/launch/forklift_navigation.launch.py
forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml
forklift_nav2_demo/config/forklift_nav2_oru_test.yaml
forklift_nav2_plugins/test/test_forklift_vehicle_model.cpp
forklift_nav2_plugins/test/test_forklift_mpc_types.cpp
forklift_nav2_plugins/test/test_forklift_mpc_trajectory.cpp
```

### 5.2 在 Foxy docker 里构建

```bash
cd /home/pl/robotest
./scripts/foxy_docker_build.sh
./scripts/foxy_docker_run.sh bash
```

进入容器后：

```bash
cd /workspace
./scripts/foxy_colcon_build.sh
```

如果容器里也遇到 ccache 临时目录问题，可以手动这样跑：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export CCACHE_DIR=/tmp/ccache
export CCACHE_TEMPDIR=/tmp/ccache-tmp
colcon --log-base log_foxy build \
  --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo \
  --symlink-install \
  --build-base build_foxy \
  --install-base install_foxy \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### 5.3 先跑单元测试

容器里先跑：

```bash
cd /workspace
./scripts/foxy_colcon_test.sh
```

如果只想先看新增测试：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon --log-base log_foxy test \
  --packages-select forklift_nav2_plugins \
  --build-base build_foxy \
  --install-base install_foxy \
  --ctest-args -R "test_forklift_vehicle_model|test_forklift_mpc_types|test_forklift_mpc_trajectory"
```

### 5.4 再跑 Foxy headless 仿真

启动：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/workspace/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/workspace/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml \
  use_sim_time:=true \
  use_rviz:=false \
  gazebo_gui:=false \
  use_sim_command_bridge:=true \
  nav2_start_delay:=5.0 \
  sim_ready_timeout:=30.0
```

发布初始位姿：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
```

### 5.5 先做两个最小话题检查

#### 检查 1：bridge 是否真的把 90 度命令映射成原地转

发布统一控制命令：

```bash
ros2 topic pub --once /forklift/control_cmd forklift_msgs/msg/ForkliftControlCommand "{
  enable: true,
  brake: false,
  forward: true,
  reverse: false,
  velocity_mps: 0.3,
  steering_angle_rad: 1.5707963267948966
}"
```

看 bridge 输出：

```bash
ros2 topic echo /forklift/sim_cmd_vel --once
```

预期：

```text
linear.x = 0.0
angular.z > 0
```

如果还是：

```text
linear.x > 0
```

说明 pivot 分支没触发，要先查 bridge 参数是不是没带进去。

#### 检查 2：负 90 度是否方向正确

```bash
ros2 topic pub --once /forklift/control_cmd forklift_msgs/msg/ForkliftControlCommand "{
  enable: true,
  brake: false,
  forward: true,
  reverse: false,
  velocity_mps: 0.3,
  steering_angle_rad: -1.5707963267948966
}"
ros2 topic echo /forklift/sim_cmd_vel --once
```

预期：

```text
linear.x = 0.0
angular.z < 0
```

如果符号反了，优先检查：

- `forklift_vehicle_model.cpp`
- `sim_command_bridge.py`

里 pivot 角速度符号逻辑是否需要统一反向。

### 5.6 再跑 acceptance

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run forklift_nav2_demo forklift_follow_path_acceptance \
  --ros-args -p use_sim_time:=true -p scenario:=sparse_90_turn -p timeout_sec:=180.0
```

### 5.7 跑的时候重点看这些输出

看 controller 日志：

```text
Configured FollowPath ... allow_pivot_turn=true ...
```

看 path preprocessing：

```text
P5 path preprocessing: ...
```

看 bridge topic：

```bash
ros2 topic echo /forklift/control_cmd
ros2 topic echo /forklift/sim_cmd_vel
```

要重点确认：

1. `/forklift/control_cmd` 是否真的发出 `steering_angle_rad` 接近 `±1.57`
2. `/forklift/sim_cmd_vel` 在这些时刻是否真的变成：

```text
linear.x = 0
angular.z != 0
```

3. `/follow_path` 最终状态是否从 `ABORTED` 变成 `SUCCEEDED`

## 6. 如果新窗口里继续失败，优先看什么

### 6.1 如果 bridge 话题检查都对，但 FollowPath 还是 abort

优先看 controller 侧：

- 是否仍然 `found no collision-free command`
- 是否 local costmap footprint 在原地转动时判碰撞
- 是否 controller 只在极少数点才给到 `±1.57`，导致动作不稳定

这时下一步可能要做的是：

```text
显式插入 turn-in-place / pivot segment
```

而不是只靠高曲率映射。

### 6.2 如果 steering 仍上不去 90 度

重点排查：

- `max_steering_angle` 是否被其他 YAML/launch 覆盖
- `sanitize()` 是否又把值夹回去了
- `control_cmd` 实际发布值是多少

### 6.3 如果 robot 原地转了，但方向不对

统一检查这两处符号逻辑：

- `forklift_nav2_plugins/src/forklift_vehicle_model.cpp`
- `forklift_vehicle_interface/forklift_vehicle_interface/sim_command_bridge.py`

### 6.4 如果原地转时直接判碰撞

优先看：

- local costmap footprint
- inflation
- 原地旋转时前叉是否扫到障碍

这类问题不是协议问题，而是 footprint / feasibility 问题。

## 7. 建议的新窗口起手提示词

你可以在新窗口直接说：

```text
先看 forklift_nav2_demo/docs/foxy_pivot_turn_followup.md，
按里面的步骤在 Foxy docker 里继续验证这次 pivot-turn 改动。
先不要重做分析，先检查当前 diff、构建、单测、bridge 90 度 topic 行为，
再跑 sparse_90_turn acceptance。
```

## 8. 总结

这次改动的核心不是“再调一点 MPC 参数”，而是把仿真里的动作语义改成和真实车协议一致：

```text
steering_angle 接近 ±90 deg 时，
仿真允许原地旋转。
```

当前代码已经改到：

- controller 参数层
- 共享车辆模型
- MPC 预测
- trajectory 参考生成
- sim bridge
- Foxy ORU 配置
- launch 默认值
- 单元测试

但还没有在 Foxy docker 里完成最终闭环验证。这一步请在新窗口按本文档继续。

## 9. 2026-06-11 进度更新

### 9.1 真车信息修正

已确认真车不是 Ackermann 回转模型，而是：

```text
双驱差速；90 度时绕后轴旋转。
```

因此当前方向继续沿 pivot-turn 做，不回退到 Ackermann。

本轮把原先“绕当前参考点原地转”的预测几何，修正为“绕后轴点旋转”：

```text
rear_axle = base_reference + rear_axle_x_offset * heading
90 deg pivot 时保持 rear_axle 不动，
由新的 theta 反算 base_reference 的 x/y。
```

当前 URDF 里左右驱动轮 joint 的 x 坐标都是 `-0.34`，所以配置里使用：

```yaml
rear_axle_x_offset: -0.34
```

### 9.2 本轮代码状态

已完成：

- `forklift_vehicle_model` 增加 `rear_axle_x_offset` 参数。
- `ForkliftVehicleModel::predict()` 的 pivot 分支改为绕后轴旋转。
- `predictMpcState()` 改为复用 `vehicle_model.predict()`，避免 MPC 预测和车辆模型几何分叉。
- `ForkliftMpcController` 读取并透传 `rear_axle_x_offset`。
- Foxy/Humble ORU 测试配置都加入 `rear_axle_x_offset: -0.34`。
- 新增/更新单测覆盖：
  - 默认 bicycle 模型不变。
  - 默认 offset 为 0 时仍表现为绕参考点原地转。
  - offset 为 `-0.34` 时 pivot 后后轴点保持不动。
  - MPC 状态预测使用同一套后轴 pivot 几何。

### 9.3 Base frame 判断

本轮没有移动 `base_link` / `base_footprint` 到后轴中心。

原因：

- 当前 URDF 已经表达了后轴相对 base reference 的偏移。
- 移动 base frame 会连带影响 footprint、laser、Gazebo odom、AMCL、Nav2 costmap 和既有地图/路径解释。
- 只在车辆模型里加入 `rear_axle_x_offset`，可以把控制预测几何修正为真车行为，同时保持现有 TF/传感器/地图语义稳定。

后续只有在真车接口确认 odom pose 的物理参考点就是后轴中心时，才需要重新评估是否移动 base frame；否则优先保持 base frame 不动。

### 9.4 Foxy docker 验证结果

已在 Foxy docker 中完成构建：

```bash
./scripts/foxy_docker_run.sh bash -lc 'export CCACHE_DIR=/tmp/ccache CCACHE_TEMPDIR=/tmp/ccache-tmp; ./scripts/foxy_colcon_build.sh'
```

结果：

```text
4 packages finished
```

已在 Foxy docker 中完成插件测试：

```bash
./scripts/foxy_docker_run.sh bash -lc 'export CCACHE_DIR=/tmp/ccache CCACHE_TEMPDIR=/tmp/ccache-tmp; ./scripts/foxy_colcon_test.sh --packages-select forklift_nav2_plugins --ctest-args -R "test_forklift_vehicle_model|test_forklift_mpc_types|test_forklift_mpc_trajectory"'
```

结果：

```text
100% tests passed, 0 tests failed out of 5
```

启动 Foxy headless 仿真后，controller 配置日志确认：

```text
allow_pivot_turn=true
pivot_steer=1.571
pivot_radius=0.600
rear_axle_x_offset=-0.340
```

随后运行 sparse 90 度 FollowPath acceptance：

```bash
ros2 run forklift_nav2_demo forklift_follow_path_acceptance \
  --ros-args -p use_sim_time:=true -p scenario:=sparse_90_turn -p timeout_sec:=180.0
```

结果：

```text
/follow_path accepted: true
/follow_path status: 4 SUCCEEDED
control_samples=152 max_velocity_mps=0.450
sim_cmd_samples=310 max_linear_x=0.450
odom_final x=-1.584 y=0.398
```

这说明 rear-axle pivot 后，稀疏 90 度路径已经从之前的超时/失败状态，推进到 Foxy headless acceptance 通过。

### 9.5 当前残留问题

仍然看到 terminal 附近 preview window 偶尔收缩到单点：

```text
start=13 end=13 points=1 length=0.000
```

这已经不是 sparse 90 度 acceptance blocker，因为 controller 最终能 `Reached the goal!`。后续如果要继续提高控制质量，可以在 P5/P4 层补 terminal preview window 的最小长度或终端段特殊处理。

### 9.6 对 P6 的影响

从 `oru_migration_execution_plan.md` 的门槛看，P6 的前置条件基本满足：

- P0/P1/P2/P3 已有稳定链路和控制接口。
- P4 controller 能通过 FollowPath / NavigateToPose 闭环。
- P5 已有 path preprocessing、插密、曲率诊断、90 度 sharp turn 处理。
- 本轮又补齐了真车 90 度后轴 pivot 几何，并通过 Foxy sparse_90_turn acceptance。

因此可以开始做 P6，但建议 P6 第一刀不要直接大搬 ORU 全套 planner；先做最小可测的 `x/y/theta_index` lattice scaffold，并保留现有 costmap-aware A* 作为 fallback。
