import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from rclpy.time import Time

import forklift_vehicle_interface.fine_motion_adapter as fine_motion

from forklift_vehicle_interface.fine_motion_adapter import (
    motion_command,
    nonnegative_finite,
    pivot_braking_angle_rad,
    pivot_command,
    pivot_yaw_is_settled,
    pivot_step_hold_enabled,
    steering_center_command,
    steering_center_is_settled,
    steering_center_settle_started_at,
    shortest_angle,
)


def test_reverse_motion_command_uses_reverse_direction():
    command = motion_command(-0.1, 1.0, 1.0)

    assert command.enable is True
    assert command.brake is False
    assert command.forward is False
    assert command.reverse is True
    assert command.velocity_mps == -0.1
    assert command.steering_angle_rad == 0.0


def test_zero_motion_command_is_a_stop():
    command = motion_command(0.0, 1.0, 1.0)

    assert command.enable is False
    assert command.brake is True
    assert command.forward is False
    assert command.reverse is False


def test_steering_center_command_has_no_traction_but_stays_enabled():
    command = steering_center_command(1.0, 1.0)

    assert command.enable is True
    assert command.brake is False
    assert command.forward is False
    assert command.reverse is False
    assert command.velocity_mps == 0.0
    assert command.steering_angle_rad == 0.0


def test_steering_center_requires_continuous_in_tolerance_feedback():
    started_at = steering_center_settle_started_at(
        steering_angle_rad=0.04,
        max_steering_angle_rad=0.10,
        now_sec=10.0,
        previous_started_at_sec=None,
    )
    assert started_at == 10.0
    assert not steering_center_is_settled(started_at, 10.39, 0.4)

    started_at = steering_center_settle_started_at(
        steering_angle_rad=0.11,
        max_steering_angle_rad=0.10,
        now_sec=10.40,
        previous_started_at_sec=started_at,
    )
    assert started_at is None

    started_at = steering_center_settle_started_at(
        steering_angle_rad=0.03,
        max_steering_angle_rad=0.10,
        now_sec=10.45,
        previous_started_at_sec=started_at,
    )
    assert started_at == 10.45
    assert steering_center_is_settled(started_at, 10.85, 0.4)


def test_shortest_angle_wraps():
    assert abs(shortest_angle(6.283185307179586)) < 1e-9


def test_zero_pivot_step_hold_is_valid():
    assert nonnegative_finite(0.0, 'pivot_step_hold_sec') == 0.0
    assert pivot_step_hold_enabled(0.0) is False
    assert pivot_step_hold_enabled(0.2) is True
    with pytest.raises(ValueError, match='non-negative'):
        nonnegative_finite(-0.1, 'pivot_step_hold_sec')


def test_pivot_requires_yaw_and_yaw_rate_to_settle():
    assert pivot_yaw_is_settled(0.02, 0.02, 0.025, 0.03)
    assert not pivot_yaw_is_settled(0.03, 0.0, 0.025, 0.03)
    assert not pivot_yaw_is_settled(0.0, 0.04, 0.025, 0.03)


def test_positive_pivot_command_uses_forward_and_left_steering():
    command = pivot_command(0.08, 1.0, 1.5707963267948966, 1.0, 1.0)

    assert command.enable is True
    assert command.forward is True
    assert command.reverse is False
    assert command.velocity_mps == 0.08
    assert command.steering_angle_rad > 1.5


def test_negative_pivot_command_uses_right_steering():
    command = pivot_command(0.06, -1.0, 1.5707963267948966, 1.0, 1.0)

    assert command.forward is True
    assert command.steering_angle_rad < -1.5


@pytest.mark.parametrize('sign', [-1., 1.])
def test_recorded_rate_brakes_before_three_degree_tolerance(sign):
    # 151039 coasted .157 rad after stop at .219 rad/s. Brake from measured
    # rotation, not from the nominal .06 m/s command clipped by the RPM floor.
    angle = pivot_braking_angle_rad(sign * .219, sign, .2, .2, .01)
    assert angle == pytest.approx(.1737025)
    assert angle > .157
    assert angle > .05


def test_pivot_brake_model_uses_only_rate_in_requested_direction():
    assert pivot_braking_angle_rad(-.2, 1., .2, .2, .01) == .01


@pytest.mark.parametrize('rate, reaction, decel, margin', [
    (float('nan'), .2, .2, .01), (.2, -.2, .2, .01),
    (.2, .2, 0., .01), (.2, .2, .2, -.01),
])
def test_pivot_brake_model_rejects_invalid_feedback_or_parameters(rate, reaction, decel, margin):
    with pytest.raises(ValueError):
        pivot_braking_angle_rad(rate, 1., reaction, decel, margin)


@pytest.fixture
def pivot_rig(monkeypatch):
    now = [100.]
    rig = SimpleNamespace(
        now=now, yaw=0., rate=0., command=None, brake_started=None,
        actual_deceleration=.2, reaction=.2, cancelled=False, stale=False,
        freeze_steering=False, commands=[], feedback=[], logs=[],
        early_center=[], brake_onsets=[], final_state='',
    )
    adapter = fine_motion.FineMotionAdapter.__new__(fine_motion.FineMotionAdapter)
    params = dict(
        control_rate_hz=20., pivot_max_speed_mps=.12, pivot_final_speed_mps=.06,
        pivot_timeout_sec=60., pivot_steering_angle_rad=math.pi/2,
        pivot_yaw_tolerance_rad=.05, pivot_yaw_rate_tolerance_radps=.03,
        pivot_yaw_settle_sec=.3, pivot_brake_reaction_time_sec=.2,
        pivot_brake_deceleration_radps2=.2, pivot_brake_margin_rad=.01,
        pivot_brake_max_resumes=1, pivot_stop_velocity_mps=.02,
        pivot_progress_epsilon_rad=.01, pivot_step_angle_rad=math.radians(20),
        pivot_step_hold_sec=0., pivot_final_slowdown_angle_rad=math.radians(20),
        progress_timeout_sec=3., odom_timeout_sec=.5, vehicle_state_timeout_sec=.5,
        max_start_steering_angle_rad=.1, steering_center_timeout_sec=3.,
        steering_center_settle_sec=.4, accel_time_sec=1., decel_time_sec=1.,
    )
    for key, value in params.items():
        setattr(adapter, '_' + key, value)
    adapter._lock = threading.Lock()
    adapter._active_goal = True
    adapter._active_pivot_sign = None
    state = SimpleNamespace(
        steering_angle_rad=math.pi/2, velocity_mps=0., auto_mode=True,
        emergency_stopped=False, soft_emergency_stop=False)
    rig.state = state
    adapter._snapshot = lambda: (
        (0., 0., fine_motion.shortest_angle(rig.yaw), rig.rate,
         now[0] - (1. if rig.stale else 0.)), (state, now[0]))
    adapter.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=now[0]))
    adapter.get_logger = lambda: SimpleNamespace(info=rig.logs.append)

    def publish(command):
        was_moving = rig.command is not None and rig.command.velocity_mps > 0.
        if command.velocity_mps == 0. and was_moving:
            rig.brake_started = now[0]
            rig.brake_onsets.append((now[0], rig.yaw, rig.rate))
        if (command.velocity_mps == 0. and abs(command.steering_angle_rad) < .1
                and abs(rig.rate) > .03):
            rig.early_center.append(command)
        rig.command = command
        rig.commands.append(command)

    adapter._command_pub = SimpleNamespace(publish=publish)

    def sleep(dt):
        command = rig.command
        if command is not None and command.velocity_mps > 0.:
            # Emulate the observed minimum-RPM floor: both nominal speeds
            # produce roughly .2 rad/s, not a factor-of-two angular slowdown.
            target_rate = math.copysign(.2, command.steering_angle_rad)
            change = min(abs(target_rate-rig.rate), .35*dt)
            rig.rate += math.copysign(change, target_rate-rig.rate)
            rig.brake_started = None
        elif rig.brake_started is not None and now[0]-rig.brake_started >= rig.reaction:
            rig.rate = math.copysign(
                max(0., abs(rig.rate)-rig.actual_deceleration*dt), rig.rate)
        rig.yaw += rig.rate*dt
        if command is not None and not rig.freeze_steering:
            state.steering_angle_rad = command.steering_angle_rad
        now[0] += dt

    monkeypatch.setattr(fine_motion.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(fine_motion.time, 'sleep', sleep)
    monkeypatch.setattr(fine_motion.rclpy, 'ok', lambda: True)
    rig.adapter = adapter

    def run(angle=math.pi/2, initial_yaw=0., max_speed=.12):
        rig.yaw = initial_yaw
        goal = SimpleNamespace(
            request=SimpleNamespace(angle_rad=angle, max_speed_mps=max_speed),
            is_cancel_requested=False,
            succeed=lambda: setattr(rig, 'final_state', 'succeeded'),
            abort=lambda: setattr(rig, 'final_state', 'aborted'),
            canceled=lambda: setattr(rig, 'final_state', 'canceled'),
            publish_feedback=rig.feedback.append)
        rig.goal = goal
        return adapter._execute_pivot(goal)

    rig.run = run
    return rig


@pytest.mark.parametrize('angle', [math.pi/2, -math.pi/2, math.pi, -math.pi])
def test_pivot_brakes_holds_steering_and_centers_only_after_settling(pivot_rig, angle):
    rig = pivot_rig
    result = rig.run(angle)
    assert result.success, result.message
    assert abs(fine_motion.shortest_angle(angle-rig.yaw)) <= .05
    assert rig.final_state == 'succeeded'
    assert not rig.early_center
    assert any(f.phase == 'YAW_BRAKE' for f in rig.feedback)
    assert any(f.phase == 'STEERING_CENTER' for f in rig.feedback)
    assert all(c.steering_angle_rad*angle > 0. for c in rig.commands if c.velocity_mps > 0.)
    assert rig.adapter._active_goal is False
    assert rig.adapter._active_pivot_sign is None


def test_pivot_handles_heading_wrap_without_reversing(pivot_rig):
    result = pivot_rig.run(math.pi/2, initial_yaw=3.)
    assert result.success, result.message
    assert abs(fine_motion.shortest_angle(3.+math.pi/2-pivot_rig.yaw)) <= .05


def test_pivot_final_phase_never_exceeds_goal_speed(pivot_rig):
    result = pivot_rig.run(max_speed=.04)
    assert result.success, result.message
    assert max(command.velocity_mps for command in pivot_rig.commands) <= .04


def test_conservative_early_brake_allows_only_one_same_direction_continuation(pivot_rig):
    rig = pivot_rig
    rig.actual_deceleration = .5
    result = rig.run()
    assert result.success, result.message
    assert sum('early-brake continuation' in message for message in rig.logs) == 1
    assert all(c.steering_angle_rad > 0. for c in rig.commands if c.velocity_mps > 0.)


def test_unmodelled_overshoot_fails_without_reverse_calibration(pivot_rig):
    rig = pivot_rig
    rig.actual_deceleration = .07
    result = rig.run()
    assert not result.success
    assert result.message == 'pivot stopped outside yaw tolerance; reverse correction disabled'
    assert not rig.early_center
    assert all(c.steering_angle_rad > 0. for c in rig.commands if c.velocity_mps > 0.)


def test_pivot_steering_center_timeout_is_not_success(pivot_rig):
    rig = pivot_rig
    rig.freeze_steering = True
    result = rig.run()
    assert not result.success
    assert result.message == 'steering did not center after pivot'


def test_pivot_rejects_stale_feedback_and_releases_action_owner(pivot_rig):
    rig = pivot_rig
    rig.stale = True
    result = rig.run()
    assert not result.success
    assert result.message == 'odometry timeout'
    assert rig.adapter._active_goal is False


def test_pivot_cannot_center_while_translation_is_still_reported(pivot_rig):
    rig = pivot_rig
    rig.state.velocity_mps = .04
    rig.adapter._pivot_timeout_sec = 12.
    result = rig.run()
    assert not result.success
    assert result.message == 'pivot motion timeout'
    assert not any(f.phase == 'STEERING_CENTER' for f in rig.feedback)


def test_cancel_during_pivot_sends_zero_rpm_without_steering_return(pivot_rig):
    rig = pivot_rig
    publish = rig.adapter._command_pub.publish

    def cancel_after_first_motion(command):
        publish(command)
        if command.velocity_mps > 0.:
            rig.goal.is_cancel_requested = True

    rig.adapter._command_pub.publish = cancel_after_first_motion
    result = rig.run()
    assert not result.success
    assert rig.final_state == 'canceled'
    assert rig.commands[-1].velocity_mps == 0.
    assert rig.commands[-1].steering_angle_rad == math.pi/2
    assert not rig.early_center


def test_pivot_cancel_command_preserves_current_turning_direction(pivot_rig):
    rig = pivot_rig
    rig.adapter._active_pivot_sign = -1.
    rig.rate = -.2
    rig.adapter._cancel_callback(None)
    command = rig.command
    assert command.velocity_mps == 0.
    assert command.brake
    assert command.steering_angle_rad == -math.pi/2
    assert not rig.early_center


def test_early_brake_resume_budget_is_bounded(pivot_rig):
    rig = pivot_rig
    rig.actual_deceleration = 3.
    result = rig.run()
    assert sum('early-brake continuation' in message for message in rig.logs) <= 1
    if not result.success:
        assert result.message == 'pivot stopped outside yaw tolerance; reverse correction disabled'
    assert all(c.steering_angle_rad > 0. for c in rig.commands if c.velocity_mps > 0.)


def test_nonfinite_pivot_feedback_cannot_finish_successfully(pivot_rig):
    rig = pivot_rig
    rig.rate = float('nan')
    result = rig.run()
    assert not result.success
    assert result.message == 'non-finite pivot motion feedback'


def test_pivot_brake_yaml_loads_in_real_ros_node():
    config = Path(__file__).resolve().parents[1] / 'config/fine_motion_adapter.yaml'
    fine_motion.rclpy.init(args=['--ros-args', '--params-file', str(config)])
    node = None
    try:
        node = fine_motion.FineMotionAdapter()
        assert node._pivot_brake_reaction_time_sec == .2
        assert node._pivot_brake_deceleration_radps2 == .2
        assert node._pivot_brake_margin_rad == .01
        assert node._pivot_brake_max_resumes == 1
        assert node._pivot_stop_velocity_mps == .02
        assert node._pivot_step_hold_sec == 0.
    finally:
        if node is not None:
            node._action_server.destroy()
            node._pivot_action_server.destroy()
            node.destroy_node()
        fine_motion.rclpy.shutdown()
