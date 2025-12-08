#!/usr/bin/env python3
"""YZ Trajectory Generator - Load and execute parametric curve trajectories.

This module provides utilities to load pre-generated trajectory waypoints
from YAML files and execute them via JointTrajectoryControllers.

The system uses separate JointTrajectoryControllers per joint:
- selector_frame_gripper_joint_controller (Y-axis: into/out of cabinet)
- main_frame_selector_frame_joint_controller (Z-axis: vertical)

Usage:
    generator = YZTrajectoryGenerator(
        waypoints_path='config/extraction_trajectories.yaml',
        config_path='config/trajectory_config.yaml'
    )

    # Load and transform waypoints
    waypoints = generator.load_trajectory(
        name='insertion',
        side='left',
        base_y=0.0,
        base_z=0.5
    )

    # Execute via action clients (coordinated Y+Z motion)
    success = await generator.execute_trajectory(waypoints, y_client, z_client)
"""
import asyncio
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

logger = logging.getLogger(__name__)


@dataclass
class TransformedWaypoint:
    """A waypoint transformed to world coordinates with timing."""

    y: float  # Joint Y position (meters) - gripper depth
    z: float  # Joint Z position (meters) - vertical
    time_from_start: float  # Seconds from trajectory start


class YZTrajectoryGenerator:
    """Load and execute parametric curve trajectories for box operations.

    This class handles:
    - Loading waypoints from generated YAML files
    - Transforming waypoints based on cabinet side (left/right)
    - Applying base position offsets
    - Calculating trajectory timing
    - Building and executing JointTrajectory messages via separate controllers

    Controller mapping:
    - Y-axis: selector_frame_gripper_joint_controller/follow_joint_trajectory
    - Z-axis: main_frame_selector_frame_joint_controller/follow_joint_trajectory
    """

    # Joint names for YZ motion
    Y_JOINT = 'selector_frame_gripper_joint'  # Y-axis (into/out of cabinet)
    Z_JOINT = 'main_frame_selector_frame_joint'  # Z-axis (vertical)

    # Action server names
    Y_ACTION = '/selector_frame_gripper_joint_controller/follow_joint_trajectory'
    Z_ACTION = '/main_frame_selector_frame_joint_controller/follow_joint_trajectory'

    # Legacy combined names for backward compatibility
    JOINT_NAMES = [Y_JOINT, Z_JOINT]

    def __init__(
        self,
        waypoints_path: str = 'config/extraction_trajectories.yaml',
        config_path: str = 'config/trajectory_config.yaml',
    ):
        """Initialize trajectory generator.

        Args:
            waypoints_path: Path to generated waypoints YAML
            config_path: Path to trajectory config (for waypoint_duration)
        """
        self.waypoints_path = Path(waypoints_path)
        self.config_path = Path(config_path)

        # Load waypoints
        with open(self.waypoints_path) as f:
            self.waypoints_data = yaml.safe_load(f)
        self.trajectories = self.waypoints_data['trajectories']

        # Load config for timing
        with open(self.config_path) as f:
            self.config = yaml.safe_load(f)

        logger.info(f"Loaded trajectories: {list(self.trajectories.keys())}")

    def load_trajectory(
        self,
        name: str,
        side: str,
        base_y: float,
        base_z: float,
        trajectory_name: str = 'extract_left',
    ) -> List[TransformedWaypoint]:
        """Load trajectory and transform to world coordinates.

        Args:
            name: Trajectory name ('insertion' or 'extraction')
            side: Cabinet side ('left' or 'right') - right flips Y sign
            base_y: Current gripper Y position (center position)
            base_z: Current selector Z position
            trajectory_name: Config trajectory name for timing lookup

        Returns:
            List of TransformedWaypoint with world coordinates and timing

        Raises:
            ValueError: If trajectory name not found
        """
        if name not in self.trajectories:
            available = list(self.trajectories.keys())
            raise ValueError(f"Unknown trajectory: {name}. Available: {available}")

        waypoints = self.trajectories[name]

        # Get timing from config (AC6)
        traj_config = self.config['trajectories'].get(trajectory_name, {})
        sampling = traj_config.get('sampling', {})
        mapping = traj_config.get('mapping', {})

        # Calculate waypoint_duration from speed if provided
        if 'speed' in sampling:
            speed = sampling['speed']  # m/s
            # Get trajectory length from y_output range
            y_output = mapping.get('y_output', [0.0, 0.4])
            trajectory_length = abs(y_output[1] - y_output[0])
            # total_time = distance / speed
            total_time = trajectory_length / speed
            # waypoint_duration = total_time / num_waypoints
            num_points = len(waypoints)
            waypoint_duration = total_time / num_points
            logger.debug(
                f"Speed={speed}m/s, length={trajectory_length}m, "
                f"total_time={total_time}s, waypoint_duration={waypoint_duration}s"
            )
        else:
            waypoint_duration = sampling.get('waypoint_duration', 0.5)

        # Transform based on side (AC4: right side = negative Y)
        sign = 1.0 if side == 'left' else -1.0

        # The raw YAML has insertion Z values inverted (negative instead of positive)
        # Both trajectories should follow the same physical path shape (curve UP)
        # So we negate Z for insertion to match extraction's shape
        z_sign = -1.0 if name == 'insertion' else 1.0

        transformed = []
        for i, wp in enumerate(waypoints):
            # AC5: Apply base position offsets
            # AC6: Timing starts from waypoint_duration (not 0) so robot has time
            # to reach first waypoint from current position
            transformed.append(
                TransformedWaypoint(
                    y=base_y + sign * wp['y'],
                    z=base_z + z_sign * wp['z'],
                    time_from_start=(i + 1) * waypoint_duration,
                )
            )

        logger.debug(
            f"Loaded {name} trajectory: {len(transformed)} waypoints, "
            f"side={side}, base=({base_y}, {base_z})"
        )

        return transformed

    def build_joint_trajectory(
        self, waypoints: List[TransformedWaypoint]
    ) -> JointTrajectory:
        """Build combined JointTrajectory message from waypoints (legacy).

        Args:
            waypoints: List of TransformedWaypoint

        Returns:
            JointTrajectory message ready to send

        Note:
            Used for testing. In production, use execute_trajectory() which
            coordinates separate Y and Z controllers.
        """
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        for i, wp in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = [wp.y, wp.z]

            # Only specify velocity at first and last points
            if i == 0 or i == len(waypoints) - 1:
                point.velocities = [0.0, 0.0]

            secs = int(wp.time_from_start)
            nsecs = int((wp.time_from_start - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)

            traj.points.append(point)

        return traj

    def build_single_joint_trajectory(
        self, waypoints: List[TransformedWaypoint], joint_name: str, axis: str
    ) -> JointTrajectory:
        """Build trajectory for a single joint.

        Args:
            waypoints: List of TransformedWaypoint
            joint_name: Name of the joint
            axis: 'y' or 'z' to select which coordinate to use

        Returns:
            JointTrajectory message for single joint

        Note:
            Velocities are left empty to let the JointTrajectoryController
            use spline interpolation for smooth motion between waypoints.
            Only the first and last waypoints have velocity=0 to ensure
            smooth start and stop.
        """
        traj = JointTrajectory()
        traj.joint_names = [joint_name]

        for i, wp in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = [wp.y if axis == 'y' else wp.z]

            # Only specify velocity at first and last points (start/stop at rest)
            # Leave intermediate velocities empty for smooth spline interpolation
            if i == 0 or i == len(waypoints) - 1:
                point.velocities = [0.0]
            # else: leave velocities empty - controller will interpolate

            secs = int(wp.time_from_start)
            nsecs = int((wp.time_from_start - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)

            traj.points.append(point)

        return traj

    async def _execute_single_trajectory(
        self,
        trajectory: JointTrajectory,
        action_client: ActionClient,
    ) -> bool:
        """Execute a single joint trajectory.

        Args:
            trajectory: JointTrajectory message
            action_client: ActionClient for FollowJointTrajectory

        Returns:
            True if completed successfully
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        goal_handle = await action_client.send_goal_async(goal)

        if not goal_handle.accepted:
            logger.error(f"Trajectory rejected for {trajectory.joint_names}")
            return False

        result = await goal_handle.get_result_async()
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    async def execute_trajectory(
        self,
        waypoints: List[TransformedWaypoint],
        y_action_client: ActionClient,
        z_action_client: ActionClient,
        timeout_sec: float = 30.0,
        marker_publisher=None,
        node: Optional[Node] = None,
        address_frame: Optional[str] = None,
        side: str = 'left',
    ) -> bool:
        """Execute YZ trajectory via coordinated joint controllers.

        Sends trajectories to both Y and Z controllers in parallel for
        smooth coordinated motion. Optionally publishes trajectory markers
        during execution for RViz visualization.

        Args:
            waypoints: Transformed waypoints from load_trajectory()
            y_action_client: ActionClient for Y-axis (selector_frame_gripper_joint)
            z_action_client: ActionClient for Z-axis (main_frame_selector_frame_joint)
            timeout_sec: Timeout for trajectory execution
            marker_publisher: Optional ROS2 publisher for MarkerArray (/trajectory_markers)
            node: Optional ROS2 node for timer creation (required if marker_publisher set)
            address_frame: Optional TF frame for markers (e.g., 'addr_l_1_5_2')
            side: Cabinet side ('left' or 'right') for marker orientation

        Returns:
            True if both trajectories completed successfully
        """
        # Build separate trajectories for Y and Z joints
        y_traj = self.build_single_joint_trajectory(waypoints, self.Y_JOINT, 'y')
        z_traj = self.build_single_joint_trajectory(waypoints, self.Z_JOINT, 'z')

        logger.info(f"Executing YZ trajectory with {len(waypoints)} waypoints")

        # Set up marker publishing timer if enabled
        marker_timer = None
        if marker_publisher is not None and node is not None:
            def publish_markers():
                self.publish_trajectory_markers(
                    waypoints=waypoints,
                    marker_publisher=marker_publisher,
                    node=node,
                    side=side,
                )

            # Publish immediately, then continue at 2Hz
            publish_markers()
            marker_timer = node.create_timer(0.5, publish_markers)
            logger.info(f"Publishing trajectory markers in {self.END_EFFECTOR_FRAMES[side]} frame")

        # Execute both trajectories in parallel
        y_task = asyncio.create_task(
            self._execute_single_trajectory(y_traj, y_action_client)
        )
        z_task = asyncio.create_task(
            self._execute_single_trajectory(z_traj, z_action_client)
        )

        # Wait for both with timeout
        try:
            results = await asyncio.wait_for(
                asyncio.gather(y_task, z_task, return_exceptions=True),
                timeout=timeout_sec,
            )
        except asyncio.TimeoutError:
            logger.error(f"Trajectory execution timed out after {timeout_sec}s")
            y_task.cancel()
            z_task.cancel()
            if marker_timer is not None:
                marker_timer.cancel()
                self.clear_trajectory_markers(marker_publisher, node)
            return False

        # Stop marker publishing and clear markers
        if marker_timer is not None:
            marker_timer.cancel()
            self.clear_trajectory_markers(marker_publisher, node)

        # Check results
        y_success = results[0] if not isinstance(results[0], Exception) else False
        z_success = results[1] if not isinstance(results[1], Exception) else False

        if y_success and z_success:
            logger.info("YZ trajectory completed successfully")
            return True
        else:
            logger.error(f"Trajectory failed: Y={y_success}, Z={z_success}")
            return False

    # End effector frame names for each side
    END_EFFECTOR_FRAMES = {
        'left': 'left_gripper_magnet',
        'right': 'right_gripper_magnet',
    }

    def publish_trajectory_markers(
        self,
        waypoints: List[TransformedWaypoint],
        marker_publisher,
        node: Node,
        side: str = 'left',
        address_frame: str = None,
        color: Optional[ColorRGBA] = None,
    ) -> None:
        """Publish trajectory waypoints as RViz markers in a fixed address frame.

        The trajectory is published in the address frame (e.g., addr_l_1_5_2),
        which is fixed in world space. Markers stay stationary while the gripper
        moves along the path, clearly showing where the end effector will travel.

        Address frame coordinate system:
        - Origin: at the box location (inside cabinet, at depth)
        - Y axis: points OUT of the cabinet (positive = toward robot)
        - Z axis: vertical (same as world Z)

        End effector trajectory in address frame:
        - Extraction: starts at Y=0 (at box), ends at Y=cabinet_depth (outside)
        - Insertion: starts at Y=cabinet_depth (outside), ends at Y=0 (at box)

        The waypoints contain JOINT positions. To convert to end effector positions
        in address frame, we need to account for:
        1. The magnet offset (end effector is ahead of joint by magnet_offset)
        2. The cabinet depth (address origin is at depth inside cabinet)

        For LEFT cabinet (joint Y positive into cabinet):
        - End effector Y = joint Y + magnet_offset
        - Address frame Y = cabinet_depth - end_effector Y
        - (Y=0 at address origin means at box, Y=cabinet_depth means outside)

        For RIGHT cabinet (joint Y negative into cabinet):
        - End effector Y = -(joint Y) + magnet_offset (absolute value)
        - Same address frame mapping applies

        Args:
            waypoints: List of TransformedWaypoint to visualize
            marker_publisher: ROS2 publisher for MarkerArray
            node: ROS2 node for getting clock time
            side: Cabinet side ('left' or 'right') - determines Y direction
            address_frame: Target address frame (e.g., 'addr_l_1_5_2'). If None,
                          uses 'world' frame with waypoint absolute positions.
            color: Optional color for markers (default: cyan)
        """
        if color is None:
            color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)  # Cyan

        marker_array = MarkerArray()
        # Use timestamp 0 to tell RViz to use the latest available transform
        from builtin_interfaces.msg import Time
        stamp = Time(sec=0, nanosec=0)

        if not waypoints:
            return

        # Use address frame if provided, otherwise use odom
        frame_id = address_frame if address_frame else 'odom'

        # Get the first waypoint's Z as base for relative Z positions
        first_z = waypoints[0].z

        # Determine sign for side (left = +1, right = -1)
        sign = 1.0 if side == 'left' else -1.0

        # Load config values for magnet offset and cabinet depth
        # These are needed to convert joint positions to address frame positions
        traj_config = self.config.get('trajectories', {}).get('extract_left', {})
        mapping = traj_config.get('mapping', {})
        y_output = mapping.get('y_output', [0.0, 0.4])
        cabinet_depth = max(y_output)  # Maximum Y from trajectory = cabinet depth

        # Magnet offset from kinematic chains config
        # Default to 0.040m if not available in config
        magnet_offset = 0.040

        # Convert joint Y positions to address frame Y positions
        #
        # The address frame has NO rotation - its axes are aligned with world axes.
        # Address origin is at the box location (cabinet_depth into the cabinet).
        #
        # For LEFT cabinet:
        # - Robot center is at world Y=0
        # - Box/address origin is at world Y=0.4 (cabinet_depth)
        # - Gripper extends in +Y direction to reach box
        #
        # In address frame local coordinates:
        # - Y=0 means at the address origin (at the box, cabinet_depth from robot)
        # - Y=-0.4 means at robot center (outside cabinet)
        # - Trajectory going INTO cabinet: Y goes from -0.4 → 0
        # - Trajectory coming OUT of cabinet: Y goes from 0 → -0.4
        #
        # Conversion: address_frame_Y = end_effector_world_Y - cabinet_depth
        # Since ee_world_Y = joint_Y + magnet_offset (for left):
        # address_frame_Y = (joint_Y + magnet_offset) - cabinet_depth
        #
        # For extraction (left): joint goes from 0.36 → -0.04
        #   - First: ee_Y = 0.36 + 0.04 = 0.4, addr_Y = 0.4 - 0.4 = 0 (at box)
        #   - Last:  ee_Y = -0.04 + 0.04 = 0, addr_Y = 0 - 0.4 = -0.4 (outside)
        #   Shows: start at box (Y=0), end outside (Y=-0.4) - correct!
        #
        # For insertion (left): joint goes from -0.04 → 0.36
        #   - First: ee_Y = -0.04 + 0.04 = 0, addr_Y = 0 - 0.4 = -0.4 (outside)
        #   - Last:  ee_Y = 0.36 + 0.04 = 0.4, addr_Y = 0.4 - 0.4 = 0 (at box)
        #   Shows: start outside (Y=-0.4), end at box (Y=0) - correct!

        def joint_to_address_y(joint_y: float) -> float:
            """Convert joint Y position to address frame Y position."""
            # End effector position in world Y
            # For left: magnet is ahead (+Y), so ee_world_Y = joint_Y + offset
            # For right: joint_Y is negative, so we need to use sign
            ee_world_y = sign * joint_y + magnet_offset
            # Address frame Y = ee_world_Y - cabinet_depth
            # (address origin is at cabinet_depth into cabinet)
            return ee_world_y - cabinet_depth

        # Sphere markers for each waypoint (AC2, AC3)
        for i, wp in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'trajectory_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position in address frame
            addr_y = joint_to_address_y(wp.y)
            rel_z = wp.z - first_z

            marker.pose.position.x = 0.0   # No lateral movement
            marker.pose.position.y = addr_y  # Y in address frame
            marker.pose.position.z = rel_z  # Vertical offset
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.012
            marker.scale.y = 0.012
            marker.scale.z = 0.012
            marker.color = color
            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        # Line strip connecting waypoints
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = stamp
        line_marker.ns = 'trajectory_path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        for wp in waypoints:
            addr_y = joint_to_address_y(wp.y)
            rel_z = wp.z - first_z
            p = Point()
            p.x = 0.0
            p.y = addr_y
            p.z = rel_z
            line_marker.points.append(p)

        line_marker.scale.x = 0.004
        line_marker.color = color
        line_marker.lifetime.sec = 0

        marker_array.markers.append(line_marker)

        # Arrow showing direction (start to end) (AC2)
        arrow_marker = Marker()
        arrow_marker.header.frame_id = frame_id
        arrow_marker.header.stamp = stamp
        arrow_marker.ns = 'trajectory_direction'
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        # Arrow from start position to end position
        start_y = joint_to_address_y(waypoints[0].y)
        end_y = joint_to_address_y(waypoints[-1].y)
        end_z = waypoints[-1].z - first_z

        start = Point(x=0.0, y=start_y, z=0.0)
        end = Point(x=0.0, y=end_y, z=end_z)
        arrow_marker.points = [start, end]

        arrow_marker.scale.x = 0.006
        arrow_marker.scale.y = 0.012
        arrow_marker.scale.z = 0.015
        arrow_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # Orange
        arrow_marker.lifetime.sec = 0

        marker_array.markers.append(arrow_marker)

        # Start marker (green sphere) (AC2)
        start_marker = Marker()
        start_marker.header.frame_id = frame_id
        start_marker.header.stamp = stamp
        start_marker.ns = 'trajectory_endpoints'
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = 0.0
        start_marker.pose.position.y = start_y
        start_marker.pose.position.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.02
        start_marker.scale.y = 0.02
        start_marker.scale.z = 0.02
        start_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)  # Green
        start_marker.lifetime.sec = 0
        marker_array.markers.append(start_marker)

        # End marker (red sphere) (AC2)
        end_marker = Marker()
        end_marker.header.frame_id = frame_id
        end_marker.header.stamp = stamp
        end_marker.ns = 'trajectory_endpoints'
        end_marker.id = 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.position.x = 0.0
        end_marker.pose.position.y = end_y
        end_marker.pose.position.z = end_z
        end_marker.pose.orientation.w = 1.0
        end_marker.scale.x = 0.02
        end_marker.scale.y = 0.02
        end_marker.scale.z = 0.02
        end_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # Red
        end_marker.lifetime.sec = 0
        marker_array.markers.append(end_marker)

        marker_publisher.publish(marker_array)
        logger.debug(f"Published {len(waypoints)} trajectory markers in {frame_id} frame")

    def clear_trajectory_markers(
        self,
        marker_publisher,
        node: Node,
    ) -> None:
        """Clear all trajectory markers from RViz.

        Args:
            marker_publisher: ROS2 publisher for MarkerArray
            node: ROS2 node for getting clock time
        """
        marker_array = MarkerArray()
        stamp = node.get_clock().now().to_msg()

        # Delete all markers in our namespaces
        for ns in ['trajectory_waypoints', 'trajectory_path', 'trajectory_direction', 'trajectory_endpoints']:
            delete_marker = Marker()
            delete_marker.header.stamp = stamp
            delete_marker.ns = ns
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        marker_publisher.publish(marker_array)
        logger.debug("Cleared trajectory markers")
