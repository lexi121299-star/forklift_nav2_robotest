# 托盘 Pivot 完成判定修复（2026-09-22）

## 原因与范围

`132614` 和 `133517` 均在 `pallet_pivot_to_target` 收到：

```text
pivot stopped outside yaw tolerance; reverse correction disabled
```

Task Manager 根据 map 朝向生成相对旋转请求，Fine Motion 根据 odom 累积角度
验收。旋转期间 map 到 odom 的航向修正发生变化，导致地图上已经对准，
相对旋转 action 却报失败，任务未进入 `pallet_final_approach`。

本次不改变 pivot 方向或速度、不增加反向校准、不修改 Safety Gate 或 scan
超时、不修改普通导航与 MPC，也不改变 staging、最终靠近距离及速度。

## 新行为

1. 等待 PivotRelative action 返回终态。
2. 正常成功，或仅返回上述明确的 odom 转角验收失败时，执行地图位姿复核。
3. 取消、超时、无进展、急停、反馈异常等其他失败不得转成成功。
4. 复核要求地图朝向误差不超过 0.05 rad；距 staging 不超过原有 0.35 m。
5. TF、odom 和车辆状态必须新鲜且数值有效；车辆处于自动模式且联锁闭合、
   无急停或驻车制动。速度不超过 0.03 m/s，角速度不超过 0.03 rad/s。
6. 合格状态持续至少 0.30 s，期间必须收到更新的地图 TF，不能靠重复旧值通过。
7. 最多复核 3 s。成功才执行既有 MoveRelative 流程；失败保持任务失败。
8. 取消或新任务使旧 action 回调和复核定时器失效，禁止旧结果启动靠近动作。

后续仍由 MoveRelative 执行方向盘回正、以 0.10 m/s 靠近；目标托盘豁免沿用
原有条件，框外障碍物仍由安全链路处理。没有绕过碰撞检查。

## 参数

文件：`forklift_task_manager/config/pallet_approach.yaml`。

```yaml
pallet_pivot_verify_timeout_sec: 3.0
pallet_pivot_yaw_tolerance_rad: 0.05
pallet_pivot_yaw_rate_tolerance_radps: 0.03
pallet_pivot_feedback_timeout_sec: 0.5
pallet_pivot_odom_topic: /odom
```

复核沿用 `pallet_nav_arrival_speed_mps` 和 `pallet_nav_arrival_settle_sec`，
持续停稳时间至少 0.30 s。没有放宽原有位置和转角容差。

## 验证

- Foxy Docker 构建 `forklift_msgs`、`forklift_task_manager` 成功。
- Task Manager 回归：84 tests，0 errors，0 failures。
- 新增测试覆盖：地图对准而 odom 失败、普通 action 失败不可覆盖、偏角、
  位置超限、尚在运动、旧 TF/odom/车辆状态、急停、联锁断开、驻车制动、
  无效数值、取消和持续停稳。
- 状态机集成测试：pivot 复核后下发 `-1.610 m / 0.10 m/s` 最终靠近，
  只有最终 MoveRelative 成功才将任务标记为成功；没有第二次 pivot。
- 离线按真实 TF、odom、车辆状态重放完成复核：
  - `132614`：约 0.348 s 通过，位置误差 0.098 m，朝向误差 0.0156 rad。
  - `133517`：约 0.341 s 通过，位置误差 0.345 m，朝向误差 0.0062 rad。

离线结果仅证明这些历史终态可以进入下一段，不能证明未录制的最后倒车运动
一定完成。特别是 `133517` 的位置误差已靠近 0.35 m 上限，本次没有放宽它。
上传后需验证方向盘回正、最后低速倒车、叉尖目标间距及框外障碍停车。

## 上传与编译

仅部署 Task Manager 包。以下目录参数末尾不要添加 `/`，避免目录被展开到 src 根目录。

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_task_manager \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/
```

车端 Docker 内，停止旧任务和节点后：

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select forklift_task_manager
source install/setup.bash
```

沿用原启动命令，重启 Task Manager。若由 demo 一起启动 Task Manager，则重启
该 launch。没有修改消息接口，不需要重新编译 plugin、Safety 或 Vehicle Interface。

通过时日志包含：

```text
pallet pivot map verification passed: position_error=... heading_error=...
```

任务状态应继续到 `pallet_final_approach`，而不是停在 pivot 的 FAILED。
