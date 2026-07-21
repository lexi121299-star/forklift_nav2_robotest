"""Pure helpers for dispatch report generation."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


def finite_or_none(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def build_dispatch_report(
    robot_id: str,
    task_status: Any,
    fault_state: Any,
    vehicle_state: Any,
    safety_status: str,
    battery_low_threshold: float,
) -> Dict[str, Any]:
    report: Dict[str, Any] = {
        'robot_id': robot_id,
        'task': task_report(task_status),
        'fault': fault_report(fault_state),
        'vehicle': vehicle_report(vehicle_state),
        'safety_status': safety_status,
    }
    report['alarms'] = alarm_report(report, battery_low_threshold)
    return report


def task_report(msg: Any) -> Dict[str, Any]:
    if msg is None:
        return {
            'available': False,
            'state': 'UNKNOWN',
            'active_route': '',
            'current_segment': '',
            'segment_index': -1,
            'segment_count': 0,
            'reason': 'task status missing',
        }
    return {
        'available': True,
        'state': msg.state,
        'active_route': msg.active_route,
        'current_segment': msg.current_segment,
        'segment_index': int(msg.segment_index),
        'segment_count': int(msg.segment_count),
        'reason': msg.reason,
    }


def fault_report(msg: Any) -> Dict[str, Any]:
    if msg is None:
        return {
            'available': False,
            'has_fault': False,
            'summary': 'fault state missing',
            'left_drive_fault_code': 0,
            'right_drive_fault_code': 0,
            'steering_fault_code': 0,
            'lift_fault_code': 0,
        }
    return {
        'available': True,
        'has_fault': bool(msg.has_fault),
        'summary': msg.summary,
        'left_drive_fault_code': int(msg.left_drive_fault_code),
        'right_drive_fault_code': int(msg.right_drive_fault_code),
        'steering_fault_code': int(msg.steering_fault_code),
        'lift_fault_code': int(msg.lift_fault_code),
    }


def vehicle_report(msg: Any) -> Dict[str, Any]:
    if msg is None:
        return {
            'available': False,
            'enabled': False,
            'auto_mode': False,
            'emergency_stopped': False,
            'soft_emergency_stop': False,
            'parking_brake': False,
            'interlock': False,
            'velocity_mps': None,
            'battery_percent': None,
            'drive_controller_temperature_c': None,
            'lift_controller_temperature_c': None,
            'mode': 'UNKNOWN',
        }
    return {
        'available': True,
        'enabled': bool(msg.enabled),
        'auto_mode': bool(msg.auto_mode),
        'emergency_stopped': bool(msg.emergency_stopped),
        'soft_emergency_stop': bool(msg.soft_emergency_stop),
        'parking_brake': bool(msg.parking_brake),
        'interlock': bool(msg.interlock),
        'velocity_mps': finite_or_none(msg.velocity_mps),
        'battery_percent': finite_or_none(msg.battery_percent),
        'drive_controller_temperature_c': finite_or_none(
            msg.drive_controller_temperature_c
        ),
        'lift_controller_temperature_c': finite_or_none(
            msg.lift_controller_temperature_c
        ),
        'mode': msg.mode,
    }


def alarm_report(report: Dict[str, Any], battery_low_threshold: float) -> List[Dict[str, str]]:
    alarms: List[Dict[str, str]] = []
    fault = report['fault']
    vehicle = report['vehicle']
    task = report['task']
    safety_status = str(report.get('safety_status', '')).strip()

    if fault['available'] and fault['has_fault']:
        alarms.append({
            'level': 'ERROR',
            'code': 'VEHICLE_FAULT',
            'message': fault['summary'] or 'vehicle fault',
        })
    if vehicle['available']:
        if vehicle['emergency_stopped'] or vehicle['soft_emergency_stop']:
            alarms.append({
                'level': 'ERROR',
                'code': 'EMERGENCY_STOP',
                'message': 'vehicle emergency stop',
            })
        battery_percent = vehicle['battery_percent']
        if battery_percent is not None and battery_percent <= battery_low_threshold:
            alarms.append({
                'level': 'WARN',
                'code': 'LOW_BATTERY',
                'message': 'battery {:.1f}% <= {:.1f}%'.format(
                    battery_percent, battery_low_threshold
                ),
            })
    if safety_status and safety_status not in {'raw command', 'raw stop', 'bypass'}:
        alarms.append({
            'level': 'WARN',
            'code': 'SAFETY_STATUS',
            'message': safety_status,
        })
    if task['available'] and task['state'] in {'PAUSED', 'FAILED'} and task['reason']:
        alarms.append({
            'level': 'WARN',
            'code': 'TASK_{}'.format(task['state']),
            'message': task['reason'],
        })
    return alarms
