#!/usr/bin/env python3
"""
Address Service Node for Warehouse Navigation System.

ROS2 service node that exposes AddressResolver functionality via the
/manipulator/get_address_coordinates service. Resolves warehouse addresses
to full poses (position + orientation) via TF lookup.

Story 3.2 - Epic 3: Address Navigation System
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from manipulator_utils.address_resolver import AddressResolver
from manipulator_control.srv import GetAddressCoordinates


class AddressServiceNode(Node):
    """ROS2 service node exposing AddressResolver via GetAddressCoordinates service."""

    def __init__(self):
        super().__init__('address_service_node')

        # Initialize AddressResolver with this node for TF access
        self._address_resolver = AddressResolver(self)

        # Create service server
        self._service = self.create_service(
            GetAddressCoordinates,
            '/manipulator/get_address_coordinates',
            self._handle_get_address_coordinates
        )

        self.get_logger().info('GetAddressCoordinates service ready at /manipulator/get_address_coordinates')

    def _handle_get_address_coordinates(self, request, response):
        """Handle service request by delegating to AddressResolver."""
        self.get_logger().debug(
            f'GetAddressCoordinates request: side={request.side}, '
            f'cabinet={request.cabinet_num}, row={request.row}, col={request.column}'
        )

        # Call AddressResolver.get_address_pose() for full pose with orientation
        x, y, z, qx, qy, qz, qw, success, error_msg = self._address_resolver.get_address_pose(
            request.side, request.cabinet_num, request.row, request.column
        )

        response.success = success
        if success:
            response.pose = Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
            )
            response.error_message = ''
            self.get_logger().debug(
                f'Resolved pose: pos=({x:.3f}, {y:.3f}, {z:.3f}), '
                f'orient=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
            )
        else:
            response.pose = Pose()
            response.error_message = error_msg
            self.get_logger().debug(f'Resolution failed: {error_msg}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddressServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
