from types import SimpleNamespace

from forklift_task_manager.dispatch_report import build_dispatch_report


def test_dispatch_report_includes_fault_and_low_battery_alarms():
    report = build_dispatch_report(
        'forklift_test',
        SimpleNamespace(
            state='PAUSED',
            active_route='route_a',
            current_segment='station_b',
            segment_index=1,
            segment_count=3,
            reason='vehicle fault',
        ),
        SimpleNamespace(
            has_fault=True,
            summary='left_drive=7',
            left_drive_fault_code=7,
            right_drive_fault_code=0,
            steering_fault_code=0,
            lift_fault_code=0,
        ),
        SimpleNamespace(
            enabled=True,
            auto_mode=True,
            emergency_stopped=False,
            soft_emergency_stop=False,
            parking_brake=False,
            interlock=True,
            velocity_mps=0.0,
            battery_percent=12.5,
            drive_controller_temperature_c=30.0,
            lift_controller_temperature_c=29.0,
            mode='auto',
        ),
        'vehicle fault',
        20.0,
    )

    codes = [alarm['code'] for alarm in report['alarms']]
    assert report['robot_id'] == 'forklift_test'
    assert report['fault']['left_drive_fault_code'] == 7
    assert report['vehicle']['battery_percent'] == 12.5
    assert 'VEHICLE_FAULT' in codes
    assert 'LOW_BATTERY' in codes
    assert 'TASK_PAUSED' in codes


def test_dispatch_report_handles_missing_inputs():
    report = build_dispatch_report(
        'forklift_test',
        None,
        None,
        None,
        '',
        20.0,
    )

    assert report['task']['state'] == 'UNKNOWN'
    assert report['fault']['available'] is False
    assert report['vehicle']['battery_percent'] is None
    assert report['alarms'] == []
