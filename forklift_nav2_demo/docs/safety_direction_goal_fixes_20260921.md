# Safety Gate、倒车方向与目标替换修复

## 本次范围

按顺序修复 920 记录中三类问题：Safety Gate 命令积压/pivot 扫掠、倒车方向不一致、新目标沿用旧路径。
不修改 footprint、padding、障碍代价阈值、托盘豁免框、最终靠近距离和巡航速度。
没有修改 CAN 协议，也没有提交或推送 Git。

## Safety Gate

- 原始控制订阅和门控发布使用 depth=1，丢弃排队旧指令。
- 独立 callback group + 两线程 executor，让停止回调不必等待同步碰撞计算。
- 停止回调立即发布零牵引命令；停止 generation 防止旧计算结果在停止后重新发运动命令。
- 检查原始消息 header 时间；缺失、明显未来或超时消息停车。
- 修复拷贝 header 的别名问题，门控重新打时间戳不会修改上游消息。
- 运动检查超过计算预算时停车，并记录计算耗时、源命令年龄。
- 普通碰撞暂停保留新鲜的实测舵角；健康故障仍采用原独立硬停止行为。
- 保留货叉 action 的制动状态下液压控制，快速停车回调不会周期性清空正常起升输出。
- scan 超时仍为 0.7 s，同时检查 header 年龄与接收年龄；高速新鲜度门限仍为 0.25 s。实际旧 scan 可能因这项检查被限速/停车，不能靠重新打时间戳掩盖延迟。

### Pivot 预测

不再把直行固定 0.5 m 余量除以 pivot 速度，换成过长的旋转时间。

```text
theta_stop = abs(omega) * reaction_time
           + omega^2 / (2 * angular_deceleration)
           + angular_margin
```

使用指令预测与新鲜 `/odom.twist.twist.angular.z`，取保守旋转范围；残余角速度与指令反向时，两侧都检查。
沿整车最远角点的移动距离采样，scan/costmap 共享旋转姿态序列，保留原始几何 padding。
角速度缺失、非有限或源时间超过定位超时门限时，pivot 不放行。

real navigation launch 新增可选参数：

```bash
pivot_brake_deceleration_radps2:=0.15 \
pivot_stop_margin_rad:=0.05 \
collision_compute_budget_sec:=0.15
```

无需添加参数即可使用默认值；这些参数也通过 Safety Gate 独立 launch 暴露。
反应时间保持 0.9 s。`0.15 rad/s²` 是待标定的初始旋转制动假设，不是从厂家确认或已完成实车认证的数值。
真实减速度如果更小，应下调模型值，扩大制动检查范围。
`collision_compute_budget_sec` 是执行预算，不能为消除停车直接无限放大，应根据日志优化计算。

## 倒车方向

- 当前方向由当前投影路段决定，不再因为远处预瞄点包含倒车就把当前段标成倒车。
- solver 增加 motion_direction 硬约束，限制候选和第一步实际输出的速度符号。
- sampled fallback 倒车时只采样倒车；评分与最终输出再次校验方向。
- 实际速度仍指向上一运动方向时，先发停止，不能继续沿错误方向移动。
- 倒车被禁用时收到倒车路径，明确停止报错，不替换为前进执行。

没有修改轨迹生成器。本次不能宣称 161623 的异常曲率路径已被修成可执行轨迹；该问题需要单独检查规划/预处理，禁止通过方向修复绕过碰撞检查。

## 新目标接管

- NavigateToPose 与直接 FollowPath 共用运动请求/结果跟踪。
- 取消请求返回不代表动作停止；等待旧 action 的终态结果后才发送新 goal。
- 旧请求尚未得到接收确认时也被追踪，迟到的旧 goal handle 会取消并等待终态。
- 连续替换时只有最新 token 对应目标可以发出；旧结果不能完成新任务。
- 等待终态超过 5 s，只使新请求失败，不退出节点，也不会冒险并发发送。旧任务随后真正结束后仍可接收新目标。
- 终态未知时保留执行屏障，不能把通信异常当作旧动作已停止。
- Task Manager 最终成功增加当前位置对原始目标的位置核对，不能在旧 anchor 处误报成功。

本次不更换 BT XML。通过现有 Task Manager 独占入口串行接管，避免 Foxy plan-once BT 的在线 preemption 沿用旧 path。
仍应使用 `/forklift/navigation_goal` 和 `/forklift/pallet_goal`，不要同时启用绕过 Task Manager 的 Nav2 RViz action 客户端。

## 验证

- 最终 Foxy `colcon test-result`：250 tests，0 errors，0 failures，0 skipped；real navigation launch 参数展开检查通过。
- Foxy Docker 构建：`forklift_safety`、`forklift_nav2_plugins`、`forklift_task_manager`、`forklift_nav2_demo`。
- 回归覆盖：过期/未来命令、header 独立拷贝、停止覆盖竞态、真实多线程 executor 下的阻塞检查中停车、液压兼容、pivot 两侧扫掠及障碍拦截。
- MPC 覆盖：solver 和 sampled fallback 的倒车输出、前方存在未来倒车点时的当前前进方向。
- 任务覆盖：取消 ACK 早于终态、接收确认迟到、连续替换、FollowPath/Nav2 互斥、取消超时后继续接单、旧 anchor 不能完成新目标。
- 只读解析 155658/161217/161623/161852 中 45 个 pivot scan 碰撞样本，新参数几何复算不再因旧过长扫掠而拒绝。不是整包闭环回放或实车安全验证。

实车先以低速测试：测量 raw 停止到 gated 停止延迟、gated 停止到实际静止余程，以及 pivot 停止后的余角。
本次没有调整普通终点的制动速度模型；消除命令积压后仍需复核 160928/161049/162033 的最终停车误差。

## 部署

本机同步整个包目录，目录末尾不加 `/`，避免把 include/src 内容摊平到包根目录：

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_safety \
  /home/pl/robotest/forklift_nav2_plugins \
  /home/pl/robotest/forklift_task_manager \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/

rsync -avzc -e "ssh -p 2222" \
  /home/pl/robotest/forklift_nav2_demo/launch/forklift_real_navigation.launch.py \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/forklift_nav2_demo/launch/
```

车端先停止本次导航进程，再在 Docker 中编译，重新 source 并重新启动：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  forklift_safety forklift_nav2_plugins forklift_task_manager forklift_nav2_demo
source install/setup.bash
```

不需要新增或重新生成 message/action。本次不要求覆盖地图和车端 YAML。
