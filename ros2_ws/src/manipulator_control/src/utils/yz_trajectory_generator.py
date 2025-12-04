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

        transformed = []
        for i, wp in enumerate(waypoints):
            # AC5: Apply base position offsets
            # AC6: Timing starts from waypoint_duration (not 0) so robot has time
            # to reach first waypoint from current position
            transformed.append(
                TransformedWaypoint(
                    y=base_y + sign * wp['y'],
                    z=base_z + wp['z'],
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
    ) -> bool:
        """Execute YZ trajectory via coordinated joint controllers.

        Sends trajectories to both Y and Z controllers in parallel for
        smooth coordinated motion.

        Args:
            waypoints: Transformed waypoints from load_trajectory()
            y_action_client: ActionClient for Y-axis (selector_frame_gripper_joint)
            z_action_client: ActionClient for Z-axis (main_frame_selector_frame_joint)
            timeout_sec: Timeout for trajectory execution

        Returns:
            True if both trajectories completed successfully
        """
        # Build separate trajectories for Y and Z joints
        y_traj = self.build_single_joint_trajectory(waypoints, self.Y_JOINT, 'y')
        z_traj = self.build_single_joint_trajectory(waypoints, self.Z_JOINT, 'z')

        logger.info(f"Executing YZ trajectory with {len(waypoints)} waypoints")

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
            return False

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
        address_frame: str,
        side: str = 'left',
        color: Optional[ColorRGBA] = None,
    ) -> None:
        """Publish trajectory waypoints as RViz markers relative to target address.

        The trajectory is published in the address frame, showing the fixed path
        the end effector will follow. The trajectory:
        - Starts at the address position (where gripper should align)
        - Extends INTO the cabinet (positive Y for left side, negative for right)
        - Shows any vertical clearance adjustments

        Coordinate mapping in address frame:
        - X axis: not used (along cabinet depth would be X in address frame)
        - Y axis: lateral (into cabinet = towards center for left, away for right)
        - Z axis: vertical offset

        Note: The waypoints contain absolute joint positions. We convert them
        to positions relative to the address frame for visualization.

        Args:
            waypoints: List of TransformedWaypoint to visualize
            marker_publisher: ROS2 publisher for MarkerArray
            node: ROS2 node for getting clock time
            address_frame: TF frame of target address (e.g., 'addr_l_1_5_2')
            side: Cabinet side ('left' or 'right')
            color: Optional color for markers (default: cyan)
        """
        if color is None:
            color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)  # Cyan

        marker_array = MarkerArray()
        stamp = node.get_clock().now().to_msg()

        if not waypoints:
            return

        # Use address frame - trajectory is fixed relative to the target address
        frame_id = address_frame

        # First waypoint is the "home" position at the address
        # Subsequent waypoints show the path INTO the cabinet
        first_y = waypoints[0].y
        first_z = waypoints[0].z

        # Direction multiplier: left side goes +Y into cabinet, right goes -Y
        # But in address frame, we always show trajectory going "forward" (+X or +Y)
        # The sign is already applied in the waypoints, so relative positions work

        # Direction: left side trajectory goes +Y (into cabinet towards center)
        # Right side goes -Y. In address frame, we show trajectory going into cabinet.
        # For left: gripper Y increases -> show as -Y in address frame (towards center)
        # For right: gripper Y decreases -> show as +Y in address frame (towards center)
        y_direction = -1.0 if side == 'left' else 1.0

        # Sphere markers for each waypoint
        for i, wp in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'trajectory_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position relative to address origin
            # Address frame has same orientation as world:
            # - X: forward (along robot rail)
            # - Y: lateral (left cabinet = +Y side, right cabinet = -Y side)
            # - Z: vertical
            rel_y = wp.y - first_y  # How far gripper moves
            rel_z = wp.z - first_z  # Vertical offset (clearance)

            # Map to address frame
            marker.pose.position.x = 0.0  # No X movement
            marker.pose.position.y = y_direction * rel_y  # Into cabinet
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
            rel_y = wp.y - first_y
            rel_z = wp.z - first_z
            p = Point()
            p.x = 0.0
            p.y = y_direction * rel_y
            p.z = rel_z
            line_marker.points.append(p)

        line_marker.scale.x = 0.004
        line_marker.color = color
        line_marker.lifetime.sec = 0

        marker_array.markers.append(line_marker)

        # Arrow showing direction (start to end)
        arrow_marker = Marker()
        arrow_marker.header.frame_id = frame_id
        arrow_marker.header.stamp = stamp
        arrow_marker.ns = 'trajectory_direction'
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        last_rel_y = waypoints[-1].y - first_y
        last_rel_z = waypoints[-1].z - first_z

        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(x=0.0, y=y_direction * last_rel_y, z=last_rel_z)
        arrow_marker.points = [start, end]

        arrow_marker.scale.x = 0.006
        arrow_marker.scale.y = 0.012
        arrow_marker.scale.z = 0.015
        arrow_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # Orange
        arrow_marker.lifetime.sec = 0

        marker_array.markers.append(arrow_marker)

        # Start marker (green sphere at address)
        start_marker = Marker()
        start_marker.header.frame_id = frame_id
        start_marker.header.stamp = stamp
        start_marker.ns = 'trajectory_endpoints'
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = 0.0
        start_marker.pose.position.y = 0.0
        start_marker.pose.position.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.02
        start_marker.scale.y = 0.02
        start_marker.scale.z = 0.02
        start_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)  # Green
        start_marker.lifetime.sec = 0
        marker_array.markers.append(start_marker)

        # End marker (red sphere at deepest point)
        end_marker = Marker()
        end_marker.header.frame_id = frame_id
        end_marker.header.stamp = stamp
        end_marker.ns = 'trajectory_endpoints'
        end_marker.id = 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.pose.position.x = 0.0
        end_marker.pose.position.y = y_direction * last_rel_y
        end_marker.pose.position.z = last_rel_z
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
        for ns in ['trajectory_waypoints', 'trajectory_path', 'trajectory_direction']:
            delete_marker = Marker()
            delete_marker.header.stamp = stamp
            delete_marker.ns = ns
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        marker_publisher.publish(marker_array)
        logger.debug("Cleared trajectory markers")
