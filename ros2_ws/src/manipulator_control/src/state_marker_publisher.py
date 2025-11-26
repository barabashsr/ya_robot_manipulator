#!/usr/bin/env python3
"""
State Marker Publisher Node

Publishes RViz markers for manipulator state visualization:
- Magnet engaged: Red sphere at gripper magnet link
- Target address: Green cube at target address TF frame
- Extracted addresses: Red cubes at extracted address TF frames

Subscribes to:
  /manipulator/electromagnet/engaged (std_msgs/Bool)
  /manipulator/target_address (std_msgs/String)
  /manipulator/extracted_addresses (std_msgs/String)

Publishes to:
  /visualization_marker_array (visualization_msgs/MarkerArray) @ 10 Hz
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class StateMarkerPublisher(Node):
    """
    RViz marker publisher for manipulator state visualization.

    Subscribes to state topics and publishes MarkerArray at configured rate.
    Marker namespace: "manipulator_state" for RViz filtering.
    """

    def __init__(self):
        super().__init__('state_marker_publisher')

        # Load configurations
        self.load_marker_config()
        self.load_storage_params()

        # State variables
        self.magnet_engaged = False
        self.target_address = ""
        self.extracted_addresses = []

        # Marker ID counters
        self.MAGNET_MARKER_ID = 0
        self.TARGET_MARKER_ID = 1
        self.EXTRACTED_MARKER_START_ID = 100  # IDs 100+ for extracted addresses

        # Subscribers
        self.magnet_sub = self.create_subscription(
            Bool,
            '/manipulator/electromagnet/engaged',
            self.magnet_callback,
            10
        )

        self.target_sub = self.create_subscription(
            String,
            '/manipulator/target_address',
            self.target_callback,
            10
        )

        self.extracted_sub = self.create_subscription(
            String,
            '/manipulator/extracted_addresses',
            self.extracted_callback,
            10
        )

        # Publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )

        # Timer for marker publication
        publish_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(publish_period, self.publish_markers)

        self.get_logger().info('State Marker Publisher started')
        self.get_logger().info(f'Publishing to /visualization_marker_array at {self.publish_rate} Hz')
        self.get_logger().info(f'Namespace: {self.marker_namespace}')

    def load_marker_config(self):
        """Load marker configuration from state_markers.yaml."""
        try:
            pkg_share = get_package_share_directory('manipulator_control')
            config_path = os.path.join(pkg_share, 'config', 'state_markers.yaml')

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            self.publish_rate = config.get('publish_rate', 10.0)
            self.marker_namespace = config.get('marker_namespace', 'manipulator_state')
            self.magnet_config = config.get('magnet_marker', {})
            self.target_config = config.get('target_marker', {})
            self.extracted_config = config.get('extracted_marker', {})

            self.get_logger().info(f'Loaded marker config from: {config_path}')

        except Exception as e:
            self.get_logger().error(f'Failed to load marker config: {e}')
            # Use defaults
            self.publish_rate = 10.0
            self.marker_namespace = 'manipulator_state'
            self.magnet_config = {
                'frame_id': 'left_gripper_magnet',
                'diameter': 0.05,
                'color': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.8}
            }
            self.target_config = {
                'color': {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 0.5}
            }
            self.extracted_config = {
                'color': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.5}
            }

    def load_storage_params(self):
        """Load box dimensions from storage_params.yaml."""
        try:
            pkg_share = get_package_share_directory('manipulator_description')
            config_path = os.path.join(pkg_share, 'config', 'storage_params.yaml')

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            self.box_configs = config.get('box_configurations', {})
            self.dept_configs = config.get('department_configurations', {})

            self.get_logger().info(f'Loaded storage params from: {config_path}')

        except Exception as e:
            self.get_logger().warn(f'Failed to load storage params: {e}')
            self.box_configs = {}
            self.dept_configs = {}

    def magnet_callback(self, msg: Bool):
        """Handle electromagnet engaged state updates."""
        self.magnet_engaged = msg.data

    def target_callback(self, msg: String):
        """Handle target address updates."""
        self.target_address = msg.data.strip()

    def extracted_callback(self, msg: String):
        """
        Handle extracted addresses updates.
        Expects comma-separated address frame IDs or empty string.
        """
        data = msg.data.strip()
        if data:
            self.extracted_addresses = [addr.strip() for addr in data.split(',') if addr.strip()]
        else:
            self.extracted_addresses = []

    def get_box_dimensions(self, address_frame: str):
        """
        Get box dimensions based on address frame.

        Address format: addr_{side}_{cabinet}_{row}_{col}
        Returns (width, height, depth) tuple.
        """
        # Default dimensions if parsing fails
        default_dims = (0.06, 0.09, 0.02)

        try:
            # Parse address: addr_l_1_2_3 -> side=l, cabinet=1, row=2, col=3
            parts = address_frame.split('_')
            if len(parts) < 5 or parts[0] != 'addr':
                return default_dims

            # For now, use default medium box size
            # Future: Look up cabinet type and compute exact dimensions
            width = self.box_configs.get('columns_4', {}).get('width', 0.06)
            height = self.box_configs.get('rows_10', {}).get('height', 0.09)
            depth = self.dept_configs.get('departments_10', {}).get('depth', 0.02)

            return (width, height, depth)

        except Exception:
            return default_dims

    def create_magnet_marker(self) -> Marker:
        """Create magnet engaged marker (red sphere)."""
        marker = Marker()
        marker.header.frame_id = self.magnet_config.get('frame_id', 'left_gripper_magnet')
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_namespace
        marker.id = self.MAGNET_MARKER_ID
        marker.type = Marker.SPHERE

        if self.magnet_engaged:
            marker.action = Marker.ADD
        else:
            marker.action = Marker.DELETE

        # Position at frame origin
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale (diameter)
        diameter = self.magnet_config.get('diameter', 0.05)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter

        # Color
        color = self.magnet_config.get('color', {})
        marker.color.r = float(color.get('r', 1.0))
        marker.color.g = float(color.get('g', 0.0))
        marker.color.b = float(color.get('b', 0.0))
        marker.color.a = float(color.get('a', 0.8))

        return marker

    def create_target_marker(self) -> Marker:
        """Create target address marker (green cube)."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_namespace
        marker.id = self.TARGET_MARKER_ID
        marker.type = Marker.CUBE

        if self.target_address:
            marker.header.frame_id = self.target_address
            marker.action = Marker.ADD

            # Get dimensions from storage params
            width, height, depth = self.get_box_dimensions(self.target_address)
            marker.scale.x = width
            marker.scale.y = depth
            marker.scale.z = height
        else:
            marker.header.frame_id = "base_link"
            marker.action = Marker.DELETE

        # Position at frame origin
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Color
        color = self.target_config.get('color', {})
        marker.color.r = float(color.get('r', 0.0))
        marker.color.g = float(color.get('g', 1.0))
        marker.color.b = float(color.get('b', 0.0))
        marker.color.a = float(color.get('a', 0.5))

        return marker

    def create_extracted_markers(self) -> list:
        """Create extracted address markers (red cubes)."""
        markers = []

        for i, address in enumerate(self.extracted_addresses):
            marker = Marker()
            marker.header.frame_id = address
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = self.marker_namespace
            marker.id = self.EXTRACTED_MARKER_START_ID + i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Get dimensions from storage params
            width, height, depth = self.get_box_dimensions(address)
            marker.scale.x = width
            marker.scale.y = depth
            marker.scale.z = height

            # Position at frame origin
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Color (red)
            color = self.extracted_config.get('color', {})
            marker.color.r = float(color.get('r', 1.0))
            marker.color.g = float(color.get('g', 0.0))
            marker.color.b = float(color.get('b', 0.0))
            marker.color.a = float(color.get('a', 0.5))

            markers.append(marker)

        return markers

    def publish_markers(self):
        """Publish all markers as MarkerArray."""
        marker_array = MarkerArray()

        # Magnet marker
        marker_array.markers.append(self.create_magnet_marker())

        # Target address marker
        marker_array.markers.append(self.create_target_marker())

        # Extracted address markers
        marker_array.markers.extend(self.create_extracted_markers())

        self.marker_pub.publish(marker_array)


def main(args=None):
    """Main entry point for state marker publisher node."""
    rclpy.init(args=args)

    try:
        node = StateMarkerPublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in StateMarkerPublisher: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
