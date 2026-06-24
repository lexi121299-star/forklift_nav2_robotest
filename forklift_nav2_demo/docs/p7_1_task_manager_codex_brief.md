# P7.1 forklift_task_manager — Codex 实施任务书

实现 `forklift_task_manager` (v1 最小路点序列器 + 状态机)。

## 唯一真相来源（先读，按它做）
- 细则：`forklift_nav2_demo/docs/oru_migration_execution_plan.md` → 第 9 节 “P7 设计” 下的 **“P7.1 落地计划（v1 实施细则）”** 子节。
- 架构边界：同节上文 “核心边界原则 / 三不 / 能力归属”。
- 参考既有风格：`forklift_safety/`（ament_python 结构、setup.py、launch、test 写法）、`forklift_msgs/`（rosidl 接口怎么加）。

## 必须遵守的硬规矩（违反即返工）
1. 不发布 `/cmd_vel`，不发布 `ForkliftControlCommand`。运动只经 Nav2 `NavigateToPose`。
2. 不写 CAN / 底盘 / 充电协议（属 vehicle_interface）。
3. 不做即时急停、不重启节点。急停只“观察 `/forklift/safety_gate/status` → 任务转 PAUSED”，且**不**调用 `/forklift_safety/set_emergency_stop`。
4. 解除急停后需显式 `resume`，不自动续跑。
5. Foxy / Python 3.8 兼容（镜像 forklift_safety 的工具链）。

## 交付物
1. `forklift_task_manager` package（节点 + route_model + config/stations.yaml + config/routes.yaml + launch + pytest）。
2. `forklift_msgs` 新增 `action/ExecuteRoute.action`、`srv/GoToStation.srv`、`msg/TaskStatus.msg`，并改 `CMakeLists.txt`（含 action_msgs 依赖）。
3. pytest 覆盖：route/stations 加载校验、状态机迁移、段序列+loop、pause/resume/cancel、safety 急停→PAUSED、重试到 FAILED。mock action client，不依赖真 Nav2。

## 验证（提交前自检）
- `bash scripts/foxy_colcon_build.sh`（或仓库现有 Foxy 构建脚本）能构建 forklift_msgs + forklift_task_manager。
- `bash scripts/foxy_colcon_test.sh` 新增 pytest 全绿。
- `ros2 node info` 确认 task_manager **没有** `/cmd_vel` / `ForkliftControlCommand` publisher。
- 不要实跑真车，不要改 forklift_safety / vehicle_interface 的运动逻辑。

## 完成后
- 在 migration plan 勾掉 `[ ] P7.1`，并在 “P7.1 落地计划” 子节末尾追加“执行记录（日期 + 实测结果 + 遗留项）”。
- 不自行做仿真端到端验收（留给人工 / 计划方最终检查）；只保证构建 + 单测通过。

不清楚的设计点先在本文件追加 “待确认” 列表，不要擅自扩大范围（不做 GUI、不做充电、不做调度、不做多机）。
