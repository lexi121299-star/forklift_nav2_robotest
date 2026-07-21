"""Aggregate forklift runtime state into a dispatch-friendly report."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import List, Optional

import rclpy
from forklift_msgs.msg import ForkliftFaultState, ForkliftVehicleState, TaskStatus
from rclpy.node import Node
from std_msgs.msg import String

from forklift_task_manager.dispatch_report import build_dispatch_report


class DispatchReporter(Node):
    """Publish a single dispatch report from task, fault, vehicle and safety state."""

    def __init__(self) -> None:
        super().__init__('forklift_dispatch_reporter')

        self.declare_parameter('robot_id', 'forklift_001')
        self.declare_parameter('report_topic', '/forklift/dispatch_report')
        self.declare_parameter('task_status_topic', '/forklift/task_status')
        self.declare_parameter('fault_state_topic', '/forklift/fault_state')
        self.declare_parameter('vehicle_state_topic', '/forklift/vehicle_state')
        self.declare_parameter('safety_status_topic', '/forklift/safety_gate/status')
        self.declare_parameter('report_rate_hz', 2.0)
        self.declare_parameter('battery_low_threshold', 20.0)
        self.declare_parameter('dispatch_http_url', '')
        self.declare_parameter('http_timeout_sec', 0.5)

        self._robot_id = str(self.get_parameter('robot_id').value)
        self._report_topic = str(self.get_parameter('report_topic').value)
        self._dispatch_http_url = str(self.get_parameter('dispatch_http_url').value)
        self._http_timeout_sec = self._positive_param('http_timeout_sec', 0.5)
        self._battery_low_threshold = float(
            self.get_parameter('battery_low_threshold').value
        )
        report_rate_hz = self._positive_param('report_rate_hz', 2.0)

        self._task_status: Optional[TaskStatus] = None
        self._fault_state: Optional[ForkliftFaultState] = None
        self._vehicle_state: Optional[ForkliftVehicleState] = None
        self._safety_status = ''
        self._last_http_error = ''

        self._report_pub = self.create_publisher(String, self._report_topic, 10)
        self.create_subscription(
            TaskStatus,
            str(self.get_parameter('task_status_topic').value),
            self._on_task_status,
            10,
        )
        self.create_subscription(
            ForkliftFaultState,
            str(self.get_parameter('fault_state_topic').value),
            self._on_fault_state,
            10,
        )
        self.create_subscription(
            ForkliftVehicleState,
            str(self.get_parameter('vehicle_state_topic').value),
            self._on_vehicle_state,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('safety_status_topic').value),
            self._on_safety_status,
            10,
        )
        self.create_timer(1.0 / report_rate_hz, self._on_timer)
        self.get_logger().info(
            'dispatch reporter ready: publishing {}'.format(self._report_topic)
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warning(
                'Parameter {} must be positive; using fallback {}.'.format(
                    name, fallback
                )
            )
            return fallback
        return value

    def _on_task_status(self, msg: TaskStatus) -> None:
        self._task_status = msg

    def _on_fault_state(self, msg: ForkliftFaultState) -> None:
        self._fault_state = msg

    def _on_vehicle_state(self, msg: ForkliftVehicleState) -> None:
        self._vehicle_state = msg

    def _on_safety_status(self, msg: String) -> None:
        self._safety_status = msg.data

    def _on_timer(self) -> None:
        report = build_dispatch_report(
            self._robot_id,
            self._task_status,
            self._fault_state,
            self._vehicle_state,
            self._safety_status,
            self._battery_low_threshold,
        )
        payload = json.dumps(report, ensure_ascii=False, sort_keys=True)
        msg = String()
        msg.data = payload
        self._report_pub.publish(msg)
        if self._dispatch_http_url:
            self._post_report(payload)

    def _post_report(self, payload: str) -> None:
        request = urllib.request.Request(
            self._dispatch_http_url,
            data=payload.encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self._http_timeout_sec):
                pass
            if self._last_http_error:
                self.get_logger().info('Dispatch HTTP report recovered.')
                self._last_http_error = ''
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            text = str(exc)
            if text != self._last_http_error:
                self._last_http_error = text
                self.get_logger().warning('Dispatch HTTP report failed: {}'.format(exc))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DispatchReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
