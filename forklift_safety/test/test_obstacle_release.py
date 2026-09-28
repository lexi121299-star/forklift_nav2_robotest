import threading
from types import MethodType, SimpleNamespace

import pytest
from forklift_msgs.msg import ForkliftControlCommand
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time

from forklift_safety.safety_command_gate import ObstacleReleaseState, SafetyCommandGate


def test_stop_keeps_braking_envelope_when_speed_command_falls():
    state = ObstacleReleaseState()
    state.prepare(1, 0.)
    state.blocked(.8)
    state.prepare(1, .03)
    assert state.speed_floor == .8
    assert not state.clear(10., 1, 1)
    assert not state.clear(10.2, 2, 2)
    assert state.clear(10.6, 3, 3)
    assert state.active
    state.commit_release()
    assert not state.active and state.speed_floor == 0.


@pytest.mark.parametrize('scan,costmap', [(1, 2), (2, 1), (1, 1)])
def test_one_clear_frame_cannot_release(scan, costmap):
    state = ObstacleReleaseState()
    state.blocked(.4)
    assert not state.clear(10., 1, 1)
    assert not state.clear(11., scan, costmap)


def test_obstacle_or_health_failure_resets_clear_duration():
    state = ObstacleReleaseState()
    state.blocked(.4)
    assert not state.clear(10., 1, None)
    state.blocked(.1)
    assert state.speed_floor == .4
    assert not state.clear(10.4, 2, None)
    state.interrupt()
    assert not state.clear(11., 3, None)
    assert state.clear(11.6, 4, None)


def test_reverse_or_new_steering_uses_new_sweep_but_still_settles():
    state = ObstacleReleaseState()
    state.prepare(1, 0.)
    state.blocked(.8)
    state.prepare(-1, 0.)
    assert state.active and state.speed_floor == 0.
    assert not state.clear(10., 1, 1)
    assert state.clear(10.6, 2, 2)
    state.blocked(.4)
    state.prepare(-1, .3)
    assert state.speed_floor == 0.


def test_clock_rollback_restarts_clear_interval():
    state = ObstacleReleaseState()
    state.blocked(.3)
    assert not state.clear(10., 1, None)
    assert not state.clear(9., 2, None)
    assert not state.clear(9.2, 3, None)
    assert state.clear(9.6, 4, None)


def test_failed_final_freshness_check_does_not_commit_release():
    state = ObstacleReleaseState()
    state.blocked(.7)
    state.clear(10., 1, None)
    assert state.clear(10.6, 2, None)
    state.interrupt()  # A timeout or raw stop wins before publication.
    state.commit_release()
    assert state.active and state.speed_floor == .7
    assert not state.clear(11., 3, None)


def test_stale_checked_costmap_stops_even_if_new_map_arrives(monkeypatch):
    from test_scan_reception import gate_fixture

    gate, now, published, _ = gate_fixture()
    gate._costmap_monitor_enabled = True
    gate._costmap_timeout_sec = 1.5
    gate._costmap_lock = threading.RLock()
    gate._last_costmap = map_message()
    gate._last_costmap_time = Time(seconds=8.55)
    gate._last_costmap_error = ''
    gate._last_costmap_interval_sec = .2
    for name in ('_costmap_snapshot', '_costmap_snapshot_stop_reason', '_on_costmap'):
        setattr(gate, name, MethodType(getattr(SafetyCommandGate, name), gate))

    def check():
        gate._checked_costmap = gate._costmap_snapshot()
        now[0] = 10.1
        gate._on_costmap(map_message())
        cmd = ForkliftControlCommand()
        cmd.enable = cmd.forward = True
        cmd.velocity_mps = .3
        return cmd, 'raw command'

    gate._latest_safe_command = check
    ticks = iter([10., 10.1])
    monkeypatch.setattr('forklift_safety.safety_command_gate.time.monotonic', lambda: next(ticks))
    SafetyCommandGate._on_timer(gate)
    assert published[-1].brake and published[-1].velocity_mps == 0.


def test_gate_probe_does_not_shrink_or_modify_output_command():
    now = [10.]
    obstacle = [True]
    stamps = [1]
    probes = []
    gate = SimpleNamespace(
        _collision_check_enabled=True, _obstacle_release=ObstacleReleaseState(),
        _scan_protection_enabled=False, _checked_costmap=None,
        _last_vehicle_state=SimpleNamespace(velocity_mps=.8),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=now[0])),
    )
    gate._protected_speed = MethodType(SafetyCommandGate._protected_speed, gate)

    def check(cmd):
        probes.append(cmd.velocity_mps)
        gate._checked_costmap = (object(), Time(seconds=stamps[0]), '', 0.)
        return 'scan footprint sweep collision' if obstacle[0] else ''

    gate._collision_check_reason = check
    cmd = ForkliftControlCommand()
    cmd.enable = cmd.forward = True
    cmd.velocity_mps = .7
    assert 'collision' in SafetyCommandGate._collision_stop_reason(gate, cmd)
    gate._last_vehicle_state.velocity_mps = 0.
    cmd.velocity_mps = .1
    assert 'collision' in SafetyCommandGate._collision_stop_reason(gate, cmd)
    assert probes == [.8, .8] and cmd.velocity_mps == .1
    obstacle[0] = False
    assert 'stable clearance' in SafetyCommandGate._collision_stop_reason(gate, cmd)
    now[0] = 10.6
    stamps[0] = 2
    assert not SafetyCommandGate._collision_stop_reason(gate, cmd)
    assert cmd.velocity_mps == .1


def map_message():
    msg = OccupancyGrid()
    msg.info.width = msg.info.height = 1
    msg.info.resolution = .05
    msg.data = [0]
    return msg


def test_costmap_snapshot_is_atomic_and_old_check_cannot_be_freshened():
    now = [10.]
    warnings = []
    gate = SimpleNamespace(
        _costmap_lock=threading.RLock(), _last_costmap=None,
        _last_costmap_time=Time(seconds=10), _last_costmap_error='',
        _last_costmap_interval_sec=0., _costmap_timeout_sec=1.5,
        _costmap_monitor_enabled=True,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=now[0])),
        get_logger=lambda: SimpleNamespace(warning=lambda s, **kw: warnings.append(s)),
    )
    SafetyCommandGate._on_costmap(gate, map_message())
    old = SafetyCommandGate._costmap_snapshot(gate)
    now[0] = 11.6
    SafetyCommandGate._on_costmap(gate, map_message())
    new = SafetyCommandGate._costmap_snapshot(gate)
    assert old[0] is not new[0]
    assert SafetyCommandGate._costmap_snapshot_stop_reason(
        gate, old, Time(seconds=11.6), 'after_check') == 'costmap timeout'
    assert not SafetyCommandGate._costmap_snapshot_stop_reason(
        gate, new, Time(seconds=11.6), 'after_check')
    assert 'receive_age=1.600s' in warnings[-1]


def test_costmap_reception_runs_during_blocked_collision_check():
    import os
    import time
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    topic = '/test_costmap_reception_{}'.format(os.getpid())
    rclpy.init(args=['--ros-args', '-p', 'costmap_topic:=' + topic,
                    '-p', 'costmap_message_type:=occupancy_grid'])
    gate, peer = SafetyCommandGate(), Node('costmap_reception_peer')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(gate)
    executor.add_node(peer)
    entered, release = threading.Event(), threading.Event()

    def blocked():
        entered.set()
        release.wait(3.)
        return ForkliftControlCommand(), 'raw stop'

    gate._latest_safe_command = blocked
    pub = peer.create_publisher(OccupancyGrid, topic, 1)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert entered.wait(2.)
        deadline = time.monotonic() + 1.
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        assert pub.get_subscription_count() > 0
        for i in range(6):
            msg = map_message()
            msg.data = [i]
            pub.publish(msg)
            time.sleep(.1)
        assert not release.is_set()
        assert gate._costmap_snapshot()[0].data[0] >= 4
    finally:
        release.set()
        executor.shutdown()
        thread.join(2.)
        peer.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
