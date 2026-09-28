"""Fresh scan delivery must not queue behind swept-footprint computation."""

import threading
from types import MethodType, SimpleNamespace

import pytest
from forklift_msgs.msg import ForkliftControlCommand
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from forklift_safety.safety_command_gate import SafetyCommandGate


def scan(stamp=9.8):
    msg = LaserScan()
    msg.header.stamp = Time(seconds=stamp).to_msg()
    msg.header.frame_id = 'base_scan'
    msg.range_max = 8.0
    return msg


def gate_fixture():
    now = [10.0]
    published, warnings = [], []
    gate = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=now[0])),
        get_logger=lambda: SimpleNamespace(warning=lambda text, **kw: warnings.append(text)),
        _scan_lock=threading.RLock(), _command_lock=threading.RLock(),
        _scan_protection_enabled=True, _scan_timeout_sec=0.7,
        _scan_required_range_m=8.0, _scan_high_speed_freshness_timeout_sec=0.25,
        _scan_degraded_max_speed_mps=1.0, _scan_fresh_recovery_duration_sec=1.0,
        _last_scan=None, _last_scan_time=Time(seconds=10),
        _scan_speed_degraded=False, _scan_fresh_recovery_start=None,
        _cached_scan=None, _cached_scan_points=[], _checked_scan=None,
        _command_timeout_sec=0.5, _collision_compute_budget=0.15,
        _checked_command_stamp=Time(seconds=10).to_msg(), _stop_generation=0,
        _command_pub=SimpleNamespace(publish=published.append),
        _publish_status=lambda _: None, _log_reason=lambda _: None,
    )
    for name in ('_on_scan', '_scan_snapshot', '_scan_snapshot_stop_reason',
                 '_scan_speed_is_degraded'):
        setattr(gate, name, MethodType(getattr(SafetyCommandGate, name), gate))
    return gate, now, published, warnings


def test_new_scan_preserves_inflight_projection_cache_and_atomic_snapshot():
    gate, now, _, _ = gate_fixture()
    first = scan()
    gate._on_scan(first)
    gate._cached_scan, gate._cached_scan_points = first, [(1.0, 2.0)]
    before = gate._scan_snapshot()
    now[0] = 10.1
    second = scan(9.9)
    gate._on_scan(second)
    after = gate._scan_snapshot()
    assert before == (first, Time(seconds=10))
    assert after == (second, Time(seconds=10.1))
    assert gate._cached_scan is first
    assert gate._cached_scan_points == [(1.0, 2.0)]


def test_delayed_but_usable_scan_degrades_speed_without_stopping():
    gate, _, _, _ = gate_fixture()
    gate._on_scan(scan(9.65))
    assert gate._scan_speed_is_degraded()
    assert not gate._scan_snapshot_stop_reason(gate._scan_snapshot(), Time(seconds=10), 'test')


@pytest.mark.parametrize('stamp', [9.0, 0.0, 11.0])
def test_recent_reception_does_not_freshen_stale_or_invalid_source(stamp):
    gate, _, _, warnings = gate_fixture()
    gate._on_scan(scan(stamp))
    assert gate._scan_snapshot_stop_reason(
        gate._scan_snapshot(), Time(seconds=10), 'test') == 'scan timeout'
    assert 'receive_age=0.000s' in warnings[-1]
    assert 'timeout=0.700s' in warnings[-1]


def test_missing_scan_and_delivery_gap_still_stop():
    gate, _, _, _ = gate_fixture()
    assert gate._scan_snapshot_stop_reason(
        gate._scan_snapshot(), Time(seconds=10), 'test') == 'scan missing'
    gate._on_scan(scan())
    assert gate._scan_snapshot_stop_reason(
        gate._scan_snapshot(), Time(seconds=10.8), 'test') == 'scan timeout'


def test_short_range_remains_rejected():
    gate, _, _, _ = gate_fixture()
    msg = scan()
    msg.range_max = 3.0
    gate._on_scan(msg)
    assert 'below required' in gate._scan_snapshot_stop_reason(
        gate._scan_snapshot(), Time(seconds=10), 'test')


def test_old_collision_input_cannot_be_freshened_by_new_arrival(monkeypatch):
    gate, now, published, _ = gate_fixture()
    gate._on_scan(scan(9.31))
    checked = gate._scan_snapshot()

    def calculate():
        gate._checked_scan = checked
        now[0] = 10.1
        gate._on_scan(scan(10.0))
        msg = ForkliftControlCommand()
        msg.enable = msg.forward = True
        msg.velocity_mps = 0.4
        return msg, 'raw command'

    gate._latest_safe_command = calculate
    ticks = iter([10.0, 10.1])
    monkeypatch.setattr('forklift_safety.safety_command_gate.time.monotonic', lambda: next(ticks))
    SafetyCommandGate._on_timer(gate)
    assert published[-1].brake and published[-1].velocity_mps == 0.0


def test_high_speed_freshness_rechecked_after_computation(monkeypatch):
    gate, now, published, _ = gate_fixture()
    gate._on_scan(scan(9.8))

    def calculate():
        gate._checked_scan = gate._scan_snapshot()
        now[0] = 10.1  # Source age crossed .25 during the check, not .7.
        msg = ForkliftControlCommand()
        msg.enable = msg.forward = True
        msg.velocity_mps = 1.4
        return msg, 'raw command'

    gate._latest_safe_command = calculate
    ticks = iter([10.0, 10.1])
    monkeypatch.setattr('forklift_safety.safety_command_gate.time.monotonic', lambda: next(ticks))
    SafetyCommandGate._on_timer(gate)
    assert not published[-1].brake and published[-1].velocity_mps == 1.0


@pytest.mark.parametrize('new_stamp,new_range', [(8.0, 8.0), (10.0, 3.0)])
def test_invalid_new_scan_before_publication_cannot_be_ignored(
    monkeypatch, new_stamp, new_range,
):
    gate, now, published, _ = gate_fixture()
    gate._on_scan(scan(9.8))

    def calculate():
        gate._checked_scan = gate._scan_snapshot()
        now[0] = 10.1
        latest = scan(new_stamp)
        latest.range_max = new_range
        gate._on_scan(latest)
        msg = ForkliftControlCommand()
        msg.enable = msg.forward = True
        msg.velocity_mps = 0.4
        return msg, 'raw command'

    gate._latest_safe_command = calculate
    ticks = iter([10.0, 10.1])
    monkeypatch.setattr('forklift_safety.safety_command_gate.time.monotonic', lambda: next(ticks))
    SafetyCommandGate._on_timer(gate)
    assert published[-1].brake and published[-1].velocity_mps == 0.0


def test_real_executor_receives_latest_scan_while_collision_callback_is_blocked():
    import os
    import time
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    topic = '/test_scan_reception_{}'.format(os.getpid())
    rclpy.init(args=['--ros-args', '-p', 'scan_topic:=' + topic])
    gate = SafetyCommandGate()
    peer = Node('scan_reception_peer')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(gate)
    executor.add_node(peer)
    entered, release, result_ready = threading.Event(), threading.Event(), threading.Event()
    reasons = []

    def slow_check():
        if not entered.is_set():
            entered.set()
            release.wait(3.0)
            reasons.append(gate._scan_snapshot_stop_reason(
                gate._scan_snapshot(), gate.get_clock().now(), 'test_after_block'))
            result_ready.set()
        msg = ForkliftControlCommand()
        msg.brake = True
        return msg, 'raw stop'

    gate._latest_safe_command = slow_check
    pub = peer.create_publisher(LaserScan, topic, 1)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert entered.wait(2.0)
        deadline = time.monotonic() + 1.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert pub.get_subscription_count() > 0
        # A blocked check lasts longer than scan_timeout while fresh scans flow.
        for i in range(10):
            msg = scan(peer.get_clock().now().nanoseconds/1e9 - 0.18)
            msg.ranges = [float(i)]
            pub.publish(msg)
            time.sleep(0.09)
        received, _ = gate._scan_snapshot()
        assert received is not None and received.ranges[0] >= 8.0
        assert not release.is_set(), 'Scan must be received during, not after, the check'
        release.set()
        assert result_ready.wait(1.0)
        assert reasons == ['']
    finally:
        release.set()
        executor.shutdown()
        thread.join(2.0)
        peer.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
