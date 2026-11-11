# 库位取货配置

`pallet_slots.yaml` 保存取货任务的库位高度和检测参数；
`stations.yaml` 保存车辆导航到库位前等待位的地图坐标。

两份配置通过 `approach_station` 关联。Task Manager 启动时会校验每个
库位引用的等待位都存在，引用错误会直接拒绝启动。

```yaml
slots:
  slot_A01_L02:
    approach_station: slot_A01_L02_approach
    level_index: 2
    pick_height_m: 1.10

pickup_defaults:
  level_pitch_m: 0.55
  scan_level_offset: 3
  fork_insert_depth_m: 0.80
  pallet_clearance_m: 0.08
  max_offset_x_m: 0.20
  max_offset_y_m: 0.08
  min_detection_confidence: 0.75
  fork_timeout_sec: 8.0
  detection_timeout_sec: 3.0
  fine_motion_timeout_sec: 6.0
  fine_motion_speed_mps: 0.08
  tilt_rad: 0.0
```

对应等待位在 `stations.yaml` 中配置：

```yaml
stations:
  slot_A01_L02_approach:
    x: 12.34
    y: -5.67
    yaw: 1.5708
    frame_id: map
```

高度计算固定为：

```text
H_pick = pick_height_m
H_scan = H_pick + scan_level_offset * level_pitch_m
```

现场投用前需要逐项确认：等待位坐标和朝向、货叉最低零位、每层实际取货
高度、层距、最大起升高度、雷达 `dx/dy` 坐标方向以及货叉到位容差。仓位
数据变化只修改 YAML，不修改 Pickup 状态机代码。
