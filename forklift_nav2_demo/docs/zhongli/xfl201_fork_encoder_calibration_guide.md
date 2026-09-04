# XFL201 Fork Encoder Calibration Guide

## Scope and Current Status

This guide covers XFL201 fork height and side-shift calibration using Kuebler
CANopen draw-wire encoders.

Implemented now:

- XFL frame `0x233` encodes lift, lower, side-shift, and tilt direction bits.
- Shared `ForkliftControlCommand` valve-current requests are normalized into
  the protocol's `0..255` speed bytes.
- `fork_control_enabled` defaults to `false`; real hydraulic motion is not
  enabled by this change.

Not implemented yet:

- Kuebler PDO subscriber and raw encoder state publisher.
- Fork mechanical-limit decoding.
- Supervised calibration jog node.
- Conversion from raw counts to `/forklift/fork/joint_state`.

## Confirmed XFL CAN Frames

The XFL protocol uses 125 kbit/s standard frames. Send `0x233` every 50 ms
while a fork command is active.

| Field | Meaning |
| --- | --- |
| `0x233 BYTE0` | Fork / auxiliary speed, `0..255` equals `0..100%`. |
| `0x233 BYTE1` | Lower speed, `0..255` equals `0..100%`. |
| `BYTE2 bit0` | Lift. |
| `BYTE2 bit1` | Lower. |
| `BYTE2 bit2` | Side shift left. |
| `BYTE2 bit3` | Side shift right. |
| `BYTE2 bit4` | Tilt forward. |
| `BYTE2 bit5` | Tilt backward. |
| `BYTE2 bit6` | Clamp open. |
| `BYTE2 bit7` | Clamp close. |

The protocol does not define separate side-shift or tilt speed bytes. Current
code maps their requested current to `BYTE0`, but `fork_control_enabled` must
stay false until this is verified with the VCM supplier.

## Kuebler CANopen Setup

The manual states factory defaults of 250 kbit/s and node ID `0x3F`. An
encoder sharing XFL's CAN bus must use 125 kbit/s and an unused node ID.

| Operation | CANopen request |
| --- | --- |
| Start all nodes | NMT CAN ID `0x000`, data `01 00`. |
| Encoder boot-up | CAN ID `0x700 + node_id`. |
| SDO request / response | `0x600 + node_id` / `0x580 + node_id`. |
| Set 125 kbit/s | Write `0x2100:00 = 0x04`. |
| Set node ID | Write `0x2101:00`. |
| Set PDO1 event timer | Write `0x1800:05` in milliseconds; use 50 ms initially. |
| Auto-start operation | Set bit14 in `0x6000:00`, then save. |
| Set draw-wire extension increasing | Configure bit0 of `0x6000:00` as documented. |
| Set position preset | Write `0x6003:00`. |
| Save application parameters | Write ASCII `save` to `0x1010:01`. |

Do not hard-code a TPDO CAN ID or position object until the supplier provides
the PDO mapping. Read or confirm `0x1800` and `0x1A00`; standard CANopen
defaults are common but are not a substitute for the delivered configuration.

## Required Supplier Confirmation

Before enabling fork control, obtain:

1. Height and side-shift encoder node IDs, bit rate, TPDO COB-IDs, PDO mapping,
   byte order, signedness, and update period.
2. The position object mapped into TPDO and whether the device reports raw
   counts, scaled units, or a 32-bit unsigned position.
3. CAN frame and bit definitions for height upper/lower and side-shift left/right
   mechanical limits.
4. Whether `0x233 BYTE0` controls side-shift and tilt speed, and the minimum
   stable safe speed for each hydraulic function.
5. Actual height stroke and side-shift mechanical stroke measured at the fork.

## Calibration Procedure After the Adapters Exist

The calibration tool must use supervised short jogs, never a blind automatic
move to a mechanical stop.

1. Park in a clear area, lower the forks, unload the truck, and assign one
   operator to the emergency stop.
2. Confirm auto mode, zero travel velocity, healthy CAN, valid encoder samples,
   and valid mechanical-limit inputs.
3. Select one axis. The tool issues only low-speed jogs of at most 0.2 s; every
   release sends a zero `0x233` command.
4. At the lower / left limit, record the raw count and the active limit bit.
5. At the upper / right limit, record the raw count and the active limit bit.
6. Enter the measured physical travel range. The tool computes:

   `meters_per_count = (position_max_m - position_min_m) / (count_max - count_min)`

7. Store the result in a dedicated calibration YAML. Reboot and verify that
   `/forklift/fork/joint_state` reaches the expected values before enabling
   `ForkMoveTo`.

Example output:

```yaml
fork_encoder_calibration:
  height:
    raw_min_count: 1000
    raw_max_count: 892341
    min_position_m: 0.0
    max_position_m: 3.3
    meters_per_count: 0.000003704
    positive_direction: 1
```

Use a software offset first. Do not permanently write `0x6003` presets during
the initial calibration; the manual warns that setting the fully retracted
endpoint to zero can overflow after draw-wire rebound. If a persisted preset is
later required, use a small positive value such as 1000 and save only after a
repeatable validation.

## Enabling the CAN Path

Only after the above checks pass, set these XFL interface parameters:

```yaml
send_fork_stop_frame: true
fork_control_enabled: true
fork_command_full_scale_ma: 800.0
```

Start with dry-run logging and compare the emitted `0x233` frames against the
VCM monitor. Then test one direction at a time at the lowest supplier-approved
speed, with the mechanical limit feedback verified before using automatic
closed-loop fork actions.

## Temporary Manual Jog Procedure

This is a temporary commissioning method for confirming that an XFL `0x233`
command moves the expected hydraulic axis. It is not a replacement for the
future calibration jog node and it must not be used for automatic movement to a
limit.

### Preconditions

1. The truck is unloaded, the surrounding area is clear, and an operator is
   stationed at the emergency stop.
2. Travel is stationary, the truck is in automatic mode, and no Nav2 task or
   fork action is running.
3. The interface has been rebuilt and restarted with the following explicit
   parameters. They default to disabled and must be set only for this test.

   ```yaml
   fork_control_enabled: true
   fork_command_full_scale_ma: 800.0
   ```

4. Open a second terminal and verify the transmitted frame before attempting
   any motion:

   ```bash
   candump -tz can0,233:7FF
   ```

   A lift command should show CAN ID `233`, a non-zero `BYTE0`, and `BYTE2`
   bit0 set. A lower command should show a non-zero `BYTE1` and `BYTE2` bit1
   set.

### Lift and Lower Test

`80 mA` is normalized by the current implementation against `800 mA`, so it
requests a conservative `10%` protocol speed. The `timeout` process ends after
half a second; command timeout handling then sends a zero fork frame. Keep the
duration at `0.5 s` for the first test.

Lift:

```bash
timeout 0.5 ros2 topic pub -r 20 \
  /forklift/control_cmd_raw \
  forklift_msgs/msg/ForkliftControlCommand \
  "{enable: true, brake: true, forward: false, reverse: false, velocity_mps: 0.0, drive_rpm: 0.0, steering_angle_rad: 0.0, steering_angle_deg: 0.0, accel_time_sec: 0.0, decel_time_sec: 0.0, pump_rpm: 0.0, lift_valve_ma: 80.0, lower_valve_ma: 0.0, side_shift_left_valve_ma: 0.0, side_shift_right_valve_ma: 0.0, tilt_forward_valve_ma: 0.0, tilt_backward_valve_ma: 0.0, horn: false, light: false}"
```

Lower:

```bash
timeout 0.5 ros2 topic pub -r 20 \
  /forklift/control_cmd_raw \
  forklift_msgs/msg/ForkliftControlCommand \
  "{enable: true, brake: true, forward: false, reverse: false, velocity_mps: 0.0, drive_rpm: 0.0, steering_angle_rad: 0.0, steering_angle_deg: 0.0, accel_time_sec: 0.0, decel_time_sec: 0.0, pump_rpm: 0.0, lift_valve_ma: 0.0, lower_valve_ma: 80.0, side_shift_left_valve_ma: 0.0, side_shift_right_valve_ma: 0.0, tilt_forward_valve_ma: 0.0, tilt_backward_valve_ma: 0.0, horn: false, light: false}"
```

Do not test side-shift or tilt with this procedure until the VCM supplier
confirms that `0x233 BYTE0` is their speed field for those functions. The
current mapping is intentionally marked as an unverified protocol assumption.

### Stop and Record the Result

Release the command by letting `timeout` expire. If motion does not stop
immediately, use the physical emergency stop; do not publish another direction
to compensate. Record for every accepted jog:

- axis and intended direction;
- wall-clock time and `candump` output;
- observed mechanical movement;
- raw encoder frame, raw count, and any active limit input.

After the test, set `fork_control_enabled: false` and restart the vehicle
interface. Do not enable `ForkMoveTo` until encoder feedback, scaling, and
limit interlocks have been implemented and verified.
