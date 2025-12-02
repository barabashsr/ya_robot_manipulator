#!/usr/bin/env python3
"""Visualize YZ trajectory as markers in RViz.

Publishes trajectory waypoints as a LINE_STRIP marker for visualization.

Usage:
    ros2 run manipulator_control visualize_trajectory.py --trajectory insertion --side left
"""
import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Add src to path
PACKAGE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(PACKAGE_DIR / 'src'))

from utils.yz_trajectory_generator import YZTrajectoryGenerator


class TrajectoryVisualizer(Node):
    """Publish trajectory as visualization markers."""

    def __init__(self, trajectory_name: str, side: str, base_y: float, base_z: float):
        super().__init__('trajectory_visualizer')

        self.publisher = self.create_publisher(
            MarkerArray, '/trajectory_visualization', 10
        )

        # Load trajectory
        config_dir = PACKAGE_DIR / 'config'
        self.generator = YZTrajectoryGenerator(
            waypoints_path=str(config_dir / 'extraction_trajectories.yaml'),
            config_path=str(config_dir / 'trajectory_config.yaml'),
        )

        self.waypoints = self.generator.load_trajectory(
            name=trajectory_name,
            side=side,
            base_y=base_y,
            base_z=base_z,
        )

        self.trajectory_name = trajectory_name
        self.side = side
        self.base_y = base_y
        self.base_z = base_z

        # Publish at 1Hz
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f'Visualizing {trajectory_name} trajectory ({side} side) '
            f'with {len(self.waypoints)} waypoints'
        )
        self.get_logger().info(f'Base position: Y={base_y}, Z={base_z}')

    def publish_markers(self):
        """Publish trajectory markers."""
        markers = MarkerArray()

        # Line strip for trajectory path
        line_marker = Marker()
        line_marker.header.frame_id = 'world'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'trajectory_path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005  # Line width

        # Color based on trajectory type
        if self.trajectory_name == 'insertion':
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        else:
            line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red

        line_marker.pose.orientation.w = 1.0

        # Convert waypoints to 3D points
        # Y axis = gripper depth (into cabinet) -> maps to world Y
        # Z axis = vertical -> maps to world Z
        # X is fixed (robot position)
        for wp in self.waypoints:
            point = Point()
            point.x = 0.5  # Fixed X position for visualization
            point.y = wp.y  # Gripper Y position
            point.z = wp.z  # Selector Z position
            line_marker.points.append(point)

        markers.markers.append(line_marker)

        # Sphere markers for waypoints
        for i, wp in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = 'world'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'trajectory_waypoints'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = 0.01
            sphere.scale.y = 0.01
            sphere.scale.z = 0.01

            # First/last waypoints in different color
            if i == 0:
                sphere.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue = start
            elif i == len(self.waypoints) - 1:
                sphere.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow = end
            else:
                sphere.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)  # White = intermediate

            sphere.pose.position.x = 0.5
            sphere.pose.position.y = wp.y
            sphere.pose.position.z = wp.z
            sphere.pose.orientation.w = 1.0

            markers.markers.append(sphere)

        # Text marker showing info
        text = Marker()
        text.header.frame_id = 'world'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'trajectory_info'
        text.id = 100
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.5
        text.pose.position.y = self.base_y
        text.pose.position.z = self.base_z + 0.1
        text.pose.orientation.w = 1.0
        text.scale.z = 0.03
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f'{self.trajectory_name} ({self.side})\n{len(self.waypoints)} waypoints'

        markers.markers.append(text)

        self.publisher.publish(markers)


def main():
    parser = argparse.ArgumentParser(description='Visualize YZ trajectory')
    parser.add_argument('--trajectory', '-t', default='insertion',
                        choices=['insertion', 'extraction'],
                        help='Trajectory to visualize')
    parser.add_argument('--side', '-s', default='left',
                        choices=['left', 'right'],
                        help='Cabinet side')
    parser.add_argument('--base-y', type=float, default=0.0,
                        help='Base Y position')
    parser.add_argument('--base-z', type=float, default=0.5,
                        help='Base Z position')
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryVisualizer(args.trajectory, args.side, args.base_y, args.base_z)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
