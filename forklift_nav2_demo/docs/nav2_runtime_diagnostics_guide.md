# Nav2 / ROS2 运行时排查指南
本文档用于现场或仿真运行时快速判断：哪个节点没起来、哪个 topic 没数据、Nav2 为什么不生成路线。

适用环境：

- ROS2 Foxy Docker
- `forklift_nav2_demo`
- Nav2 + Gazebo + RViz 仿真
- 后续真车联调时也可参考

## 1. 进入正在运行的 Docker

先在宿主机查看当前容器：

```bash
docker ps
```

找到 `forklift-nav2:foxy` 对应的容器名，例如：

```text
romantic_mccarthy
```

进入容器：

```bash
docker exec -it <容器名> bash
```

进入后先加载 ROS 环境：

```bash
source /opt/ros/foxy/setup.bash
source /workspace/install/setup.bash
```

如果只想在宿主机终端执行一条命令，也可以这样：

```bash
docker exec -it <容器名> bash -lc 'source /opt/ros/foxy/setup.bash && source /workspace/install/setup.bash && ros2 node list'
```

## 2. 看关键节点有没有挂

查看所有节点：

```bash
ros2 node list
```

重点检查这些节点：

```text
/map_server
/amcl
/planner_server
/controller_server
/bt_navigator
/recoveries_server
/waypoint_follower
```

判断方式：

| 现象 | 说明 |
| --- | --- |
| `/planner_server` 不存在 | 不能生成全局路线 |
| `/controller_server` 不存在 | 有路线也不能跟踪执行 |
| `/amcl` 不存在 | 没有定位，通常不能正常导航 |
| `/map_server` 不存在 | 没有地图 |
| `/bt_navigator` 不存在 | RViz 发 Nav2 Goal 后任务不会正常编排 |

## 3. 看 Nav2 lifecycle 是否 active

Nav2 节点存在不代表能工作，还要看 lifecycle 状态。

```bash
for n in map_server amcl controller_server planner_server bt_navigator recoveries_server waypoint_follower; do
  echo ===$n===
  ros2 lifecycle get /$n || true
done
```

正常状态应该是：

```text
active [3]
```

如果某个节点是 `unconfigured`、`inactive`，或者 `Node not found`，说明它还没有进入可工作状态。

## 4. 看 action 是否存在

```bash
ros2 action list
```

重点检查：

```text
/navigate_to_pose
/compute_path_to_pose
/follow_path
```

判断方式：

| Action | 用途 | 不存在时的影响 |
| --- | --- | --- |
| `/navigate_to_pose` | RViz 的 Nav2 Goal 总入口 | RViz 发目标无效 |
| `/compute_path_to_pose` | 全局规划 | 不能生成路线 |
| `/follow_path` | 路径跟踪 | 有路线也不会走 |

## 5. 看 topic 是否有发布者

```bash
ros2 topic info /map
ros2 topic info /amcl_pose
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /plan
ros2 topic info /goal_pose
```

重点看 `Publisher count`：

| Topic | 正常情况 | 说明 |
| --- | --- | --- |
| `/map` | Publisher count >= 1 | 地图正在发布 |
| `/scan` | Publisher count >= 1 | 激光雷达正在发布 |
| `/odom` | Publisher count >= 1 | 里程计正在发布 |
| `/amcl_pose` | Publisher count >= 1 | AMCL 定位结果正在发布 |
| `/plan` | Publisher count >= 1 | planner 有路径发布能力 |
| `/goal_pose` | 点 RViz Goal 时会出现消息 | RViz 目标点输入 |

注意：`/plan` 有 publisher 不代表已经成功生成路线，还要看有没有实际消息和 planner 日志。

## 6. 看 topic 是否真的有数据

Foxy 的 `ros2 topic echo` 没有 `--once`，建议用 `timeout`：

```bash
timeout 5 ros2 topic echo /amcl_pose
timeout 5 ros2 topic echo /scan
timeout 5 ros2 topic echo /odom
timeout 5 ros2 topic echo /plan
```

判断方式：

| 现象 | 说明 |
| --- | --- |
| 5 秒内有输出 | topic 正常有数据 |
| 5 秒内没有输出 | topic 没有数据，或当前没有触发 |
| `/plan` 没输出 | 当前没有成功生成路径 |
| `/goal_pose` 没输出 | 可能还没在 RViz 点 Nav2 Goal |

## 7. 看 TF 是否正常

导航至少需要这些 TF 链路：

```text
map -> odom -> base_footprint/base_link
```

快速查看：

```bash
timeout 3 ros2 topic echo /tf | grep -E "frame_id: map|child_frame_id: odom|frame_id: odom|child_frame_id: base_footprint|child_frame_id: base_link"
```

正常应能看到类似：

```text
frame_id: map
child_frame_id: odom

frame_id: odom
child_frame_id: base_footprint
```

如果没有 `map -> odom`：

- 仿真中可能还没有在 RViz 点 `2D Pose Estimate`。
- 真车中可能是定位节点没有输出。
- AMCL 可能没有收到地图、雷达或初始位。

## 8. 查看 planner 日志

先找最新日志：

```bash
ls -lt /workspace/.foxy_home/.ros/log/planner_server_*.log | head
```

查看最新 planner 日志末尾：

```bash
tail -100 /workspace/.foxy_home/.ros/log/planner_server_最新文件名.log
```

也可以直接搜关键字：

```bash
grep -E "ERROR|WARN|failed|Failed|Lattice|process has died|exit code|costmap|transform" /workspace/.foxy_home/.ros/log/planner_server_*.log
```

常见日志解释：

| 日志关键字 | 含义 | 常见处理 |
| --- | --- | --- |
| `process has died` | 节点退出 | 看 exit code 和前后日志 |
| `exit code -9` | 进程被系统杀掉 | 通常是地图/costmap 太大，换低分辨率地图 |
| `StaticLayer: Resizing costmap to 5318 X 2178 at 0.050000 m/pix` | 原始大地图进入 costmap | 对仿真规划可能太重 |
| `Timed out waiting for transform from base_link to map` | TF 不通 | 先给初始位，检查 AMCL/定位 |
| `lattice search found no kinodynamic path` | lattice 没找到运动学可行路径 | 检查目标姿态、规划参数、是否需要临时打开 A* fallback |
| `rejected_footprint` 很多 | 车体 footprint 撞障碍 | 检查地图、膨胀层、车体尺寸 |
| `rejected_costmap` 很多 | 路径穿过障碍或未知区域 | 检查目标点、地图和 costmap |

## 9. 查看整体 launch 是否有进程退出

找最新 launch 日志目录：

```bash
find /workspace/.foxy_home/.ros/log -maxdepth 2 -type d -printf "%T@ %p\n" | sort -n | tail -10
```

查看最新 launch.log：

```bash
tail -120 /workspace/.foxy_home/.ros/log/<最新目录>/launch.log
```

如果看到：

```text
[ERROR] [planner_server-xx]: process has died
```

说明 planner 已经挂了，这时继续在 RViz 点目标也不会生成路线，需要先修原因并重启。

## 10. 当前 map5 仿真推荐排查顺序

当前 `map5` 原始地图较大：

```text
5318 x 2178
resolution: 0.05
```

如果只是先验证能不能规划路线，建议先用降采样地图：

```text
/workspace/.tmp/map5_sim/map5_0p50.yaml
```

并配套使用 0.50m 仿真参数：

```text
/workspace/.tmp/map5_sim/forklift_nav2_map5_fake_0p50.yaml
```

推荐启动：

```bash
cd /home/pl/robotest

./scripts/foxy_docker_run.sh bash -lc '
source /workspace/install/setup.bash &&
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  world:=/workspace/forklift_nav2_demo/worlds/map5_factory_approx.world \
  map:=/workspace/.tmp/map5_sim/map5_0p50.yaml \
  nav2_params_file:=/workspace/.tmp/map5_sim/forklift_nav2_map5_fake_0p50.yaml \
  x_pose:=16.544765 \
  y_pose:=11.342589 \
  yaw:=0.109115 \
  use_rviz:=true \
  gazebo_gui:=true \
  use_sim_command_bridge:=true
'
```

启动后操作顺序：

1. RViz 里点 `2D Pose Estimate`，点到车辆实际出生位置附近。
2. 等待 TF 出现 `map -> odom`。
3. 先点一个离车 2 到 5 米的 `Nav2 Goal`。
4. 如果近距离能规划，再逐步点远一点。
5. 如果仍失败，查 `planner_server` 日志。

## 11. 一键健康检查命令

进入容器后可以直接运行：

```bash
echo "=== nodes ==="
ros2 node list | sort

echo "=== lifecycle ==="
for n in map_server amcl controller_server planner_server bt_navigator recoveries_server waypoint_follower; do
  echo ===$n===
  ros2 lifecycle get /$n || true
done

echo "=== actions ==="
ros2 action list

echo "=== topic info ==="
for t in /map /amcl_pose /scan /odom /plan /goal_pose /tf; do
  echo ===$t===
  ros2 topic info $t || true
done

echo "=== latest planner logs ==="
ls -lt /workspace/.foxy_home/.ros/log/planner_server_*.log 2>/dev/null | head
grep -E "ERROR|WARN|failed|Failed|Lattice|process has died|exit code|costmap|transform" /workspace/.foxy_home/.ros/log/planner_server_*.log 2>/dev/null | tail -80
```

## 12. 快速判断表

| 现象 | 优先检查 | 可能原因 |
| --- | --- | --- |
| RViz 点目标没反应 | `/navigate_to_pose`、`/goal_pose` | Nav2 action 不存在，RViz 没发出去 |
| 不显示路线 | `/planner_server`、`/compute_path_to_pose`、planner 日志 | planner 挂了或规划失败 |
| planner 不存在 | launch.log | 地图太大、参数错误、插件加载失败 |
| planner active 但 abort | planner 日志 | 目标姿态不合适、lattice 无解、footprint 碰撞 |
| 没定位 | `/amcl_pose`、TF | 没点初始位，AMCL 没收到 scan/map |
| 有路线但车不动 | `/follow_path`、`/cmd_vel`、safety gate | controller 或安全门阻止运动 |
| Gazebo 车位置和 RViz 不一致 | `x_pose/y_pose/yaw`、`2D Pose Estimate` | Gazebo 出生点和 AMCL 初始位不一致 |
