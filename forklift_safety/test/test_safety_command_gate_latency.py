import math
import threading
from types import SimpleNamespace

import pytest
from forklift_msgs.msg import ForkliftControlCommand
from rclpy.time import Time

from forklift_safety.safety_command_gate import (
    SafetyCommandGate, clamp_control_command, pivot_braking_poses,
    scan_sweep_collision, source_age_sec,
)


FOOTPRINT = [(1.709, .610), (1.709, -.610), (-1.590, -.610), (-1.590, .610)]


def command():
    msg = ForkliftControlCommand()
    msg.header.stamp = Time(seconds=10).to_msg()
    msg.enable = True
    msg.forward = True
    msg.velocity_mps = .05
    msg.steering_angle_rad = math.pi / 2
    return msg


def test_clamping_does_not_restamp_source_command():
    original = command()
    gated = clamp_control_command(original, 1., .2, math.pi / 2, 2500., 1., 1.)
    gated.header.stamp = Time(seconds=11).to_msg()
    assert original.header.stamp.sec == 10


def test_source_age_rejects_missing_future_and_delayed_stamps():
    assert source_age_sec(Time().to_msg(), 10.) == math.inf
    assert source_age_sec(Time(seconds=11).to_msg(), 10.) == math.inf
    assert source_age_sec(Time(seconds=9).to_msg(), 10.) == 1.


def test_pivot_uses_braking_angle_and_outer_corner_sampling():
    poses = pivot_braking_poses(FOOTPRINT, .05/.6, 0., .9, .15, .05, 0., .05)
    assert poses[-1][2] == pytest.approx(.05/.6*.9 + (.05/.6)**2/.3 + .05)
    radius = max(math.hypot(x, y) for x, y in FOOTPRINT)
    assert all(abs(b[2]-a[2])*radius <= .05 + 1e-9 for a, b in zip(poses, poses[1:]))
    args = dict(scan_points=[(.719, 1.694), (.652, 1.699)], footprint=FOOTPRINT,
                command=command(), wheel_base=1.4, pivot_turn_radius=.6,
                rear_axle_x_offset=0., stopping_distance_m=.546,
                sample_spacing_m=.05, pivot_steering_angle_rad=math.pi/2,
                collision_padding_m=.05)
    assert scan_sweep_collision(**args)[0]
    assert not scan_sweep_collision(**args, prediction_poses=poses)[0]
    args['scan_points'] = [(1.60, .70)]
    assert scan_sweep_collision(**args, prediction_poses=poses)[0]


def test_residual_rotation_is_checked_even_when_command_reverses():
    poses = pivot_braking_poses(FOOTPRINT, .08, -.25, .9, .15, .05, .2, .05)
    assert min(p[2] for p in poses) < -.4
    assert max(p[2] for p in poses) > .1
    assert all(x + .2*math.cos(yaw) == pytest.approx(.2) for x, y, yaw in poses)
    with pytest.raises(ValueError):
        pivot_braking_poses(FOOTPRINT, .08, math.nan, .9, .15, .05, 0., .05)


def fake_gate():
    published = []
    clock = SimpleNamespace(now=lambda: Time(seconds=10))
    gate = SimpleNamespace(
        _command_lock=threading.RLock(), _stop_generation=0,
        _command_timeout_sec=.5, _max_steering_angle_rad=math.pi/2,
        _command_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: clock, _stop_reason_from_health=lambda: '',
        _collision_compute_budget=.15, _checked_command_stamp=Time(seconds=10).to_msg(),
        _publish_status=lambda _: None, _log_reason=lambda _: None,
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None),
    )
    return gate, published


def test_stop_callback_wins_over_inflight_collision_result():
    gate, published = fake_gate()

    def slow_result():
        stop = command()
        stop.brake = True
        stop.velocity_mps = 0.
        SafetyCommandGate._on_raw_command(gate, stop)
        return command(), 'raw command'

    gate._latest_safe_command = slow_result
    SafetyCommandGate._on_timer(gate)
    assert len(published) == 1
    assert published[0].brake and published[0].drive_rpm == 0.
    assert published[0].velocity_mps == 0.
    assert not published[0].forward and not published[0].reverse


def test_stale_motion_input_stops_immediately():
    gate, published = fake_gate()
    msg = command()
    msg.header.stamp = Time(seconds=9).to_msg()
    SafetyCommandGate._on_raw_command(gate, msg)
    assert published[-1].brake and not published[-1].enable


def test_over_budget_collision_result_is_not_published_as_motion(monkeypatch):
    gate, published = fake_gate()
    ticks = iter([10., 10.2])
    monkeypatch.setattr('forklift_safety.safety_command_gate.time.monotonic', lambda: next(ticks))
    gate._latest_safe_command = lambda: (command(), 'raw command')
    SafetyCommandGate._on_timer(gate)
    assert published[-1].brake and not published[-1].enable


def test_health_stop_is_not_reenabled_by_alignment_command():
    gate, published = fake_gate()
    gate._stop_reason_from_health = lambda: 'vehicle emergency stop'
    msg = command()
    msg.brake = True
    SafetyCommandGate._on_raw_command(gate, msg)
    assert not published[-1].enable
    assert published[-1].steering_angle_rad == 0.


def test_braked_fork_action_keeps_hydraulics_but_never_traction():
    gate, published = fake_gate()
    msg = command()
    msg.brake = True
    msg.pump_rpm = 1500.
    msg.lift_valve_ma = 200.
    SafetyCommandGate._on_raw_command(gate, msg)
    assert published[-1].pump_rpm == 1500.
    assert published[-1].lift_valve_ma == 200.
    assert published[-1].velocity_mps == 0.
    assert not published[-1].forward


def test_real_executor_stop_interrupts_blocked_check():
    import os
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    prefix = '/test_gate_{}'.format(os.getpid())
    rclpy.init(args=['--ros-args', '-r', '/forklift/control_cmd_raw:=' + prefix + '/raw',
                    '-r', '/forklift/control_cmd:=' + prefix + '/safe'])
    gate = SafetyCommandGate()
    peer = Node('gate_latency_test_peer')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(gate)
    executor.add_node(peer)
    entered = threading.Event()
    release = threading.Event()
    stopped = threading.Event()
    received = []

    def receive(msg):
        received.append(msg)
        if msg.brake:
            stopped.set()

    peer.create_subscription(ForkliftControlCommand, prefix + '/safe', receive, 10)
    pub = peer.create_publisher(ForkliftControlCommand, prefix + '/raw', 1)
    gate._stop_reason_from_health = lambda: ''

    def blocked_check():
        if entered.is_set():
            msg = command()
            msg.brake = True
            msg.velocity_mps = 0.
            return msg, 'raw stop'
        entered.set()
        release.wait(3.)
        return command(), 'raw command'

    gate._latest_safe_command = blocked_check
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert entered.wait(2.)
        import time
        deadline = time.monotonic() + 2.
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        msg = command()
        msg.header.stamp = peer.get_clock().now().to_msg()
        msg.brake = True
        msg.velocity_mps = 0.
        pub.publish(msg)
        assert stopped.wait(.5), 'Stop was blocked behind collision computation'
        assert not release.is_set()
        release.set()
        time.sleep(.1)
        assert received and all(m.brake and m.velocity_mps == 0. for m in received)
    finally:
        release.set()
        executor.shutdown()
        thread.join(2.)
        peer.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
