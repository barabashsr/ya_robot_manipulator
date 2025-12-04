#!/usr/bin/env python3
"""Publish YZ trajectory waypoints as RViz markers.

Visualizes insertion and extraction trajectories as:
- Spheres at each waypoint
- Line strips connecting waypoints
- Arrow showing direction

Usage:
    ros2 run manipulator_control trajectory_marker_publisher.py
    # Or directly:
    python3 scripts/trajectory_marker_publisher.py --side left --base-z 0.5

Then in RViz, add MarkerArray display for topic: /trajectory_markers
"""
import argparse
import sys
from pathlib import Path

import rclpy
import rclpy.parameter
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Add src to path
PACKAGE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(PACKAGE_DIR / 'src'))

from utils.yz_trajectory_generator import YZTrajectoryGenerator


class TrajectoryMarkerPublisher(Node):
    """Publishes trajectory waypoints as RViz markers."""

    # End effector frames for each side (from kinematic_chains.yaml)
    END_EFFECTOR_FRAMES = {
        'left': 'left_gripper_magnet',
        'right': 'right_gripper_magnet',
    }

    def __init__(self, side: str = 'left', base_y: float = 0.0, base_z: float = 0.5):
        super().__init__('trajectory_marker_publisher', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.side = side
        self.base_y = base_y
        self.base_z = base_z

        # Get end effector frame for this side
        self.end_effector_frame = self.END_EFFECTOR_FRAMES.get(side, 'gripper')

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        # Load trajectory generator
        config_dir = PACKAGE_DIR / 'config'
        self.generator = YZTrajectoryGenerator(
            waypoints_path=str(config_dir / 'extraction_trajectories.yaml'),
            config_path=str(config_dir / 'trajectory_config.yaml'),
        )

        # Publish markers periodically
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f'Publishing trajectory markers for {side} side '
            f'(frame: {self.end_effector_frame})'
        )

    def create_waypoint_markers(
        self,
        waypoints: list,
        namespace: str,
        color: ColorRGBA,
        id_offset: int = 0,
    ) -> list:
        """Create sphere markers for each waypoint."""
        markers = []

        for i, wp in enumerate(waypoints):
            marker = Marker()
            # Use the end effector frame (left_gripper_magnet or right_gripper_magnet)
            marker.header.frame_id = self.end_effector_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace
            marker.id = id_offset + i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # In end effector frame coordinates:
            # The trajectory describes motion relative to current magnet position
            # - Trajectory Y: depth into cabinet (becomes local Y for left, -Y for right)
            # - Trajectory Z: vertical clearance offset (becomes local Z)
            #
            # Get relative trajectory values (offset from base position)
            raw_y = wp.y - self.base_y  # Relative depth
            raw_z = wp.z - self.base_z  # Relative Z offset

            # In magnet frame: Y points into the cabinet (for left side)
            # For right side, trajectory Y is already negated by load_trajectory
            # But here we're showing relative curve shape, so just use raw values
            marker.pose.position.x = 0.0   # No lateral offset
            marker.pose.position.y = raw_y  # Depth into cabinet (local Y)
            marker.pose.position.z = raw_z  # Vertical clearance offset

            marker.pose.orientation.w = 1.0

            # Size
            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = 0.015

            marker.color = color
            marker.lifetime.sec = 0  # Persistent

            markers.append(marker)

        return markers

    def create_line_marker(
        self,
        waypoints: list,
        namespace: str,
        color: ColorRGBA,
        marker_id: int,
    ) -> Marker:
        """Create line strip connecting waypoints."""
        marker = Marker()
        marker.header.frame_id = self.end_effector_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for wp in waypoints:
            raw_y = wp.y - self.base_y
            raw_z = wp.z - self.base_z
            p = Point()
            p.x = 0.0
            p.y = raw_y  # Depth into cabinet
            p.z = raw_z  # Vertical offset
            marker.points.append(p)

        marker.scale.x = 0.005  # Line width

        marker.color = color
        marker.lifetime.sec = 0

        return marker

    def create_arrow_marker(
        self,
        start_wp,
        end_wp,
        namespace: str,
        color: ColorRGBA,
        marker_id: int,
    ) -> Marker:
        """Create arrow showing trajectory direction."""
        marker = Marker()
        marker.header.frame_id = self.end_effector_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Start point
        start_raw_y = start_wp.y - self.base_y
        start_raw_z = start_wp.z - self.base_z
        start = Point()
        start.x = 0.0
        start.y = start_raw_y
        start.z = start_raw_z
        marker.points.append(start)

        # End point
        end_raw_y = end_wp.y - self.base_y
        end_raw_z = end_wp.z - self.base_z
        end = Point()
        end.x = 0.0
        end.y = end_raw_y
        end.z = end_raw_z
        marker.points.append(end)

        marker.scale.x = 0.008  # Shaft diameter
        marker.scale.y = 0.015  # Head diameter
        marker.scale.z = 0.02   # Head length

        marker.color = color
        marker.lifetime.sec = 0

        return marker

    def create_text_marker(
        self,
        text: str,
        position: Point,
        namespace: str,
        marker_id: int,
    ) -> Marker:
        """Create text label."""
        marker = Marker()
        marker.header.frame_id = self.end_effector_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = position
        marker.pose.position.z += 0.03  # Offset above

        marker.scale.z = 0.02  # Text height

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = text
        marker.lifetime.sec = 0

        return marker

    def publish_markers(self):
        """Publish all trajectory markers."""
        marker_array = MarkerArray()

        # Colors
        insertion_color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)  # Blue
        extraction_color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.8)  # Orange

        # Load trajectories
        insertion_wps = self.generator.load_trajectory(
            'insertion', self.side, self.base_y, self.base_z
        )
        extraction_wps = self.generator.load_trajectory(
            'extraction', self.side, self.base_y, self.base_z
        )

        # Insertion trajectory markers
        marker_array.markers.extend(
            self.create_waypoint_markers(insertion_wps, 'insertion_points', insertion_color, 0)
        )
        marker_array.markers.append(
            self.create_line_marker(insertion_wps, 'insertion_line', insertion_color, 100)
        )
        # Arrow from start to mid-point
        mid_idx = len(insertion_wps) // 2
        marker_array.markers.append(
            self.create_arrow_marker(
                insertion_wps[0], insertion_wps[mid_idx], 'insertion_arrow', insertion_color, 101
            )
        )

        # Extraction trajectory markers (offset Y slightly for visibility)
        extraction_offset = []
        for wp in extraction_wps:
            from utils.yz_trajectory_generator import TransformedWaypoint
            extraction_offset.append(
                TransformedWaypoint(y=wp.y, z=wp.z + 0.005, time_from_start=wp.time_from_start)
            )

        marker_array.markers.extend(
            self.create_waypoint_markers(extraction_offset, 'extraction_points', extraction_color, 200)
        )
        marker_array.markers.append(
            self.create_line_marker(extraction_offset, 'extraction_line', extraction_color, 300)
        )
        # Arrow from start to mid-point
        marker_array.markers.append(
            self.create_arrow_marker(
                extraction_offset[0], extraction_offset[mid_idx], 'extraction_arrow', extraction_color, 301
            )
        )

        # Labels - use relative coordinates in YZ plane
        start_raw_y = insertion_wps[0].y - self.base_y
        start_raw_z = insertion_wps[0].z - self.base_z
        start_pos = Point(x=0.0, y=start_raw_y, z=start_raw_z)
        marker_array.markers.append(
            self.create_text_marker('START', start_pos, 'labels', 400)
        )

        end_raw_y = insertion_wps[-1].y - self.base_y
        end_raw_z = insertion_wps[-1].z - self.base_z
        end_pos = Point(x=0.0, y=end_raw_y, z=end_raw_z)
        marker_array.markers.append(
            self.create_text_marker('DEPTH', end_pos, 'labels', 401)
        )

        # Info text
        info_pos = Point(x=0.0, y=0.2, z=0.05)
        info_text = f'{self.side.upper()} magnet\nBlue=Insert\nOrange=Extract'
        marker_array.markers.append(
            self.create_text_marker(info_text, info_pos, 'labels', 402)
        )

        self.marker_pub.publish(marker_array)


def main():
    parser = argparse.ArgumentParser(description='Publish trajectory markers to RViz')
    parser.add_argument('--side', choices=['left', 'right'], default='left',
                        help='Cabinet side (default: left)')
    parser.add_argument('--base-y', type=float, default=0.0,
                        help='Base Y position (default: 0.0)')
    parser.add_argument('--base-z', type=float, default=0.5,
                        help='Base Z position (default: 0.5)')
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryMarkerPublisher(
        side=args.side,
        base_y=args.base_y,
        base_z=args.base_z,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
