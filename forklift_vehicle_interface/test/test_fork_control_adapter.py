import pytest

from forklift_vehicle_interface.fork_control_adapter import (
    ForkControlConfig,
    ForkPose,
    ForkTarget,
    build_fork_control_step,
    validate_target,
)


def test_raise_height_uses_pump_and_lift_valve():
    step = build_fork_control_step(
        ForkTarget(height_m=1.0, side_shift_m=0.0, tilt_rad=0.0),
        ForkPose(height_m=0.5, side_shift_m=0.0, tilt_rad=0.0),
        ForkControlConfig(),
    )

    assert step.phase == 'RAISE_HEIGHT'
    assert step.complete is False
    assert step.command.pump_rpm > 0.0
    assert step.command.lift_valve_ma > 0.0
    assert step.command.lower_valve_ma == 0.0


def test_lower_height_uses_lower_valve():
    step = build_fork_control_step(
        ForkTarget(height_m=0.5, side_shift_m=0.0, tilt_rad=0.0),
        ForkPose(height_m=1.0, side_shift_m=0.0, tilt_rad=0.0),
        ForkControlConfig(),
    )

    assert step.phase == 'LOWER_HEIGHT'
    assert step.command.pump_rpm == 0.0
    assert step.command.lift_valve_ma == 0.0
    assert step.command.lower_valve_ma > 0.0


def test_side_shift_runs_after_height_is_reached():
    step = build_fork_control_step(
        ForkTarget(height_m=1.0, side_shift_m=0.04, tilt_rad=0.0),
        ForkPose(height_m=1.0, side_shift_m=0.0, tilt_rad=0.0),
        ForkControlConfig(),
    )

    assert step.phase == 'SIDE_SHIFT_RIGHT'
    assert step.command.side_shift_right_valve_ma > 0.0
    assert step.command.side_shift_left_valve_ma == 0.0


def test_negative_side_shift_can_use_left_valve():
    step = build_fork_control_step(
        ForkTarget(height_m=1.0, side_shift_m=-0.04, tilt_rad=0.0),
        ForkPose(height_m=1.0, side_shift_m=0.0, tilt_rad=0.0),
        ForkControlConfig(),
    )

    assert step.phase == 'SIDE_SHIFT_LEFT'
    assert step.command.side_shift_left_valve_ma > 0.0
    assert step.command.side_shift_right_valve_ma == 0.0


def test_tilt_runs_after_height_and_side_shift_are_reached():
    step = build_fork_control_step(
        ForkTarget(height_m=1.0, side_shift_m=0.0, tilt_rad=0.05),
        ForkPose(height_m=1.0, side_shift_m=0.0, tilt_rad=0.0),
        ForkControlConfig(),
    )

    assert step.phase == 'TILT_BACKWARD'
    assert step.command.tilt_backward_valve_ma > 0.0
    assert step.command.tilt_forward_valve_ma == 0.0


def test_complete_when_all_axes_are_inside_tolerance():
    step = build_fork_control_step(
        ForkTarget(height_m=1.0, side_shift_m=0.0, tilt_rad=0.0),
        ForkPose(height_m=1.005, side_shift_m=0.002, tilt_rad=0.003),
        ForkControlConfig(),
    )

    assert step.phase == 'HOLD'
    assert step.complete is True
    assert step.command.pump_rpm == 0.0
    assert step.command.lift_valve_ma == 0.0
    assert step.command.lower_valve_ma == 0.0


@pytest.mark.parametrize(
    'target',
    [
        ForkTarget(height_m=3.5, side_shift_m=0.0, tilt_rad=0.0),
        ForkTarget(height_m=1.0, side_shift_m=0.2, tilt_rad=0.0),
        ForkTarget(height_m=1.0, side_shift_m=0.0, tilt_rad=0.4),
    ],
)
def test_validate_target_rejects_configured_limits(target):
    ok, message = validate_target(target, ForkControlConfig())

    assert ok is False
    assert 'exceeds configured limits' in message
