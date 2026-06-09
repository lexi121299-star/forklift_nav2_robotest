from __future__ import annotations

from typing import Optional

import rclpy
from forklift_msgs.msg import ForkliftControlCommand
from rclpy.node import Node

from forklift_vehicle_interface.curtis_can_codec import (
    encode_0x203,
    encode_0x303,
    encode_0x403,
    format_frame,
)


class CurtisVehicleInterface(Node):
    """Dry-run Curtis interface skeleton.

    Real SocketCAN I/O is intentionally left out of P2. This node validates the
    command-to-frame boundary and logs the exact frames that a CAN transport
    layer will send later.
    """

    def __init__(self) -> None:
        super().__init__('curtis_vehicle_interface')
        self.declare_parameter('dry_run', True)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self.create_subscription(
            ForkliftControlCommand,
            '/forklift/control_cmd',
            self._on_command,
            10,
        )
        if self._dry_run:
            self.get_logger().info('curtis_vehicle_interface running in dry_run mode.')
        else:
            self.get_logger().warning('SocketCAN transport is not implemented in P2; commands are logged only.')

    def _on_command(self, command: ForkliftControlCommand) -> None:
        frame_203 = encode_0x203(command)
        frame_303 = encode_0x303(command)
        frame_403 = encode_0x403(command)
        self.get_logger().info(
            'Curtis TX dry-run: '
            f'0x203 [{format_frame(frame_203)}], '
            f'0x303 [{format_frame(frame_303)}], '
            f'0x403 [{format_frame(frame_403)}]'
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CurtisVehicleInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
