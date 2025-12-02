#!/usr/bin/env python3
"""Test YZ trajectory execution at a real address.

1. Navigate to an address using NavigateToAddress
2. Visualize the trajectory markers at that position
3. Execute the trajectory

Usage:
    python3 test_trajectory_execution.py --side left --cabinet 1 --row 3 --column 2
"""
import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

# Add src to path
PACKAGE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(PACKAGE_DIR / 'src'))

from utils.yz_trajectory_generator import YZTrajectoryGenerator

# Import custom messages
from manipulator_control.action import NavigateToAddress
from manipulator_control.srv import GetAddressCoordinates


class TrajectoryExecutionTest(Node):
    """Test trajectory execution at an address."""

    # Approach configuration
    APPROACH_SPEED = 0.05  # m/s - slow approach to not damage box
    CONTACT_PENETRATION = 0.01  # m - how far into the box the magnet goes (10mm)

    def __init__(self, side: str, cabinet: int, row: int, column: int):
        super().__init__('trajectory_execution_test')

        self.side = side
        self.cabinet = cabinet
        self.row = row
        self.column = column

        # Select end effector frame based on side
        # Left side uses left magnet, right side uses right magnet
        self.end_effector_frame = (
            'left_gripper_magnet' if side == 'left' else 'right_gripper_magnet'
        )

        # TF2 buffer for looking up magnet positions
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/trajectory_visualization', 10
        )

        # Service client for address lookup
        self.address_client = self.create_client(
            GetAddressCoordinates, '/manipulator/get_address_coordinates'
        )

        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToAddress, '/navigate_to_address'
        )
        self.y_traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/selector_frame_gripper_joint_controller/follow_joint_trajectory'
        )
        self.z_traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/main_frame_selector_frame_joint_controller/follow_joint_trajectory'
        )

        # Joint state subscriber
        self.current_joints = {}
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Load trajectory generator
        config_dir = PACKAGE_DIR / 'config'
        self.generator = YZTrajectoryGenerator(
            waypoints_path=str(config_dir / 'extraction_trajectories.yaml'),
            config_path=str(config_dir / 'trajectory_config.yaml'),
        )

        self.get_logger().info(
            f'Testing trajectory at {side}-{cabinet}-{row}-{column}'
        )

    def joint_state_callback(self, msg):
        """Store current joint positions."""
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def get_magnet_world_position(self) -> tuple:
        """Get current magnet world position via TF lookup.

        Returns:
            Tuple of (x, y, z) in world frame, or None if lookup fails
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', self.end_effector_frame, rclpy.time.Time()
            )
            pos = transform.transform.translation
            return (pos.x, pos.y, pos.z)
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return None

    def get_address_coordinates(self):
        """Get world coordinates for the address."""
        if not self.address_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Address service not available')
            return None

        request = GetAddressCoordinates.Request()
        request.side = self.side
        request.cabinet_num = self.cabinet
        request.row = self.row
        request.column = self.column

        future = self.address_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Address service call failed')
            return None

        result = future.result()
        if not result.success:
            self.get_logger().error(f'Address lookup failed: {result.error_message}')
            return None

        return result.pose.position.x, result.pose.position.y, result.pose.position.z

    def navigate_to_address(self):
        """Navigate to the address."""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            return False

        goal = NavigateToAddress.Goal()
        goal.side = self.side
        goal.cabinet_num = self.cabinet
        goal.row = self.row
        goal.column = self.column
        goal.approach_distance = 0.0

        self.get_logger().info('Sending navigation goal...')
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        self.get_logger().info('Navigation goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()

        # Spin until complete with longer timeout for navigation
        start_time = time.time()
        while not result_future.done() and (time.time() - start_time) < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            self.get_logger().error('Navigation timed out')
            return False

        result = result_future.result()
        if result is None:
            self.get_logger().error('Navigation result is None')
            return False

        if result.result.success:
            self.get_logger().info('Navigation complete!')
            return True
        else:
            self.get_logger().error(f'Navigation failed: {result.result.message}')
            return False

    def approach_box(self, target_joint_y: float):
        """Move the gripper to approach/contact the box.

        This moves the gripper joint to the target position at a slow speed
        to safely approach and contact the box.

        Args:
            target_joint_y: Target gripper joint position (absolute)

        Returns:
            True if successful
        """
        if not self.y_traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Y trajectory action server not available')
            return False

        # Get current position
        current_y = self.current_joints.get('selector_frame_gripper_joint', 0.0)
        distance = abs(target_joint_y - current_y)

        # Calculate time based on approach speed
        approach_time = distance / self.APPROACH_SPEED

        self.get_logger().info(
            f'Approaching box: Y {current_y:.4f} -> {target_joint_y:.4f} '
            f'(distance={distance:.3f}m, time={approach_time:.2f}s)'
        )

        # Build simple 2-point trajectory
        traj = JointTrajectory()
        traj.joint_names = [self.generator.Y_JOINT]

        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = [current_y]
        start_point.velocities = [0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
        traj.points.append(start_point)

        # End point (target position)
        end_point = JointTrajectoryPoint()
        end_point.positions = [target_joint_y]
        end_point.velocities = [0.0]
        secs = int(approach_time)
        nsecs = int((approach_time - secs) * 1e9)
        end_point.time_from_start = Duration(sec=secs, nanosec=nsecs)
        traj.points.append(end_point)

        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.y_traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Approach goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        start = time.time()
        while time.time() - start < approach_time + 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                break

        if result_future.done() and result_future.result().result.error_code == 0:
            self.get_logger().info('Approach complete - magnet at box')
            return True
        else:
            self.get_logger().error('Approach failed')
            return False

    def publish_trajectory_markers(self, world_x: float, world_y: float, world_z: float,
                                    trajectory_name: str):
        """Publish trajectory markers at the address position.

        The trajectory is visualized in the world frame, showing the path the
        end effector (magnet) takes during extraction/insertion.

        Uses TF to get the current magnet position and calculate the trajectory
        path in world coordinates. This eliminates hardcoded magnet offset calculations.

        Args:
            world_x: Address X coordinate in world frame
            world_y: Address Y coordinate in world frame (positive for left, negative for right)
            world_z: Address Z coordinate in world frame
            trajectory_name: 'insertion' or 'extraction'
        """
        # Load raw trajectory waypoints (joint offsets from 0 to 0.4)
        raw_waypoints = self.generator.trajectories[trajectory_name]

        markers = MarkerArray()

        # Direction sign: left side cabinets are at +Y, right side at -Y
        y_sign = 1.0 if self.side == 'left' else -1.0

        # Get current magnet and joint positions via TF
        magnet_pos = self.get_magnet_world_position()
        if magnet_pos is None:
            self.get_logger().warning('Cannot get magnet position, using approximate visualization')
            # Fallback: assume trajectory ends at address
            max_joint = 0.4
            magnet_base_y = world_y - y_sign * max_joint
        else:
            # Current gripper joint position
            current_gripper_y = self.current_joints.get('selector_frame_gripper_joint', 0.0)
            # Calculate where magnet would be at joint=0
            # magnet_world_y = base_y + y_sign * joint_y
            # So: base_y = magnet_world_y - y_sign * joint_y
            magnet_base_y = magnet_pos[1] - y_sign * current_gripper_y
            self.get_logger().debug(
                f'Magnet at {magnet_pos}, joint_y={current_gripper_y:.4f}, base_y={magnet_base_y:.4f}'
            )

        # Line strip for trajectory path
        line = Marker()
        line.header.frame_id = 'world'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'trajectory_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.008  # Line width
        line.pose.orientation.w = 1.0

        # Color: green for insertion, red for extraction
        if trajectory_name == 'insertion':
            line.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            line.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # Convert waypoints to world coordinates
        # world_y_magnet = magnet_base_y + y_sign * joint_y
        for wp in raw_waypoints:
            point = Point()
            point.x = world_x
            point.y = magnet_base_y + y_sign * wp['y']
            point.z = world_z + wp['z']
            line.points.append(point)

        markers.markers.append(line)

        # Start/end spheres
        start_wp = raw_waypoints[0]
        end_wp = raw_waypoints[-1]
        for i, (wp, color, name) in enumerate([
            (start_wp, ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), 'start'),
            (end_wp, ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0), 'end'),
        ]):
            sphere = Marker()
            sphere.header.frame_id = 'world'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = f'trajectory_{name}'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02
            sphere.color = color
            sphere.pose.position.x = world_x
            sphere.pose.position.y = magnet_base_y + y_sign * wp['y']
            sphere.pose.position.z = world_z + wp['z']
            sphere.pose.orientation.w = 1.0
            markers.markers.append(sphere)

        # Address marker (cyan sphere at address origin)
        addr_marker = Marker()
        addr_marker.header.frame_id = 'world'
        addr_marker.header.stamp = self.get_clock().now().to_msg()
        addr_marker.ns = 'address_origin'
        addr_marker.id = 3
        addr_marker.type = Marker.SPHERE
        addr_marker.action = Marker.ADD
        addr_marker.scale.x = addr_marker.scale.y = addr_marker.scale.z = 0.025
        addr_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan
        addr_marker.pose.position.x = world_x
        addr_marker.pose.position.y = world_y
        addr_marker.pose.position.z = world_z
        addr_marker.pose.orientation.w = 1.0
        markers.markers.append(addr_marker)

        # Info text
        text = Marker()
        text.header.frame_id = 'world'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'trajectory_info'
        text.id = 10
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = world_x
        text.pose.position.y = world_y
        text.pose.position.z = world_z + 0.1
        text.pose.orientation.w = 1.0
        text.scale.z = 0.03
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f'{trajectory_name.upper()}\n{self.side}-{self.cabinet}-{self.row}-{self.column}'
        markers.markers.append(text)

        self.marker_pub.publish(markers)
        self.get_logger().info(
            f'Published {trajectory_name} trajectory markers at address '
            f'({world_x:.3f}, {world_y:.3f}, {world_z:.3f})'
        )

    def execute_trajectory(self, trajectory_name: str, base_y: float, base_z: float):
        """Execute the trajectory.

        The trajectory waypoints are ABSOLUTE joint positions (0 to 0.4m for Y).
        The base_y/base_z offsets shift the entire trajectory to start from
        the current position.

        Navigation has already positioned the selector_frame so that when the
        gripper joint reaches its maximum (0.4m), the correct magnet will be
        at the address position.
        """
        # Wait for action servers
        if not self.y_traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Y trajectory action server not available')
            return False
        if not self.z_traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Z trajectory action server not available')
            return False

        # Load trajectory - no magnet offset needed here
        # The waypoints define absolute joint positions, and navigation has
        # already positioned the robot so the magnet reaches the target
        waypoints = self.generator.load_trajectory(
            name=trajectory_name,
            side=self.side,
            base_y=base_y,
            base_z=base_z,
        )

        self.get_logger().info(
            f'Trajectory base: Y={base_y:.4f}, Z={base_z:.4f}'
        )

        # Build separate trajectories for Y and Z
        y_traj = self.generator.build_single_joint_trajectory(
            waypoints, self.generator.Y_JOINT, 'y'
        )
        z_traj = self.generator.build_single_joint_trajectory(
            waypoints, self.generator.Z_JOINT, 'z'
        )

        self.get_logger().info(
            f'Executing {trajectory_name} trajectory with {len(waypoints)} waypoints...'
        )

        # Send both goals
        y_goal = FollowJointTrajectory.Goal()
        y_goal.trajectory = y_traj
        z_goal = FollowJointTrajectory.Goal()
        z_goal.trajectory = z_traj

        y_future = self.y_traj_client.send_goal_async(y_goal)
        z_future = self.z_traj_client.send_goal_async(z_goal)

        rclpy.spin_until_future_complete(self, y_future, timeout_sec=5.0)
        rclpy.spin_until_future_complete(self, z_future, timeout_sec=5.0)

        y_handle = y_future.result()
        z_handle = z_future.result()

        if not y_handle.accepted or not z_handle.accepted:
            self.get_logger().error('Trajectory goals rejected')
            return False

        self.get_logger().info('Trajectory goals accepted, executing...')

        # Wait for results
        y_result_future = y_handle.get_result_async()
        z_result_future = z_handle.get_result_async()

        # Spin until both complete (with timeout based on trajectory duration)
        total_time = waypoints[-1].time_from_start + 5.0
        start = time.time()
        while time.time() - start < total_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if y_result_future.done() and z_result_future.done():
                break

        y_success = (y_result_future.done() and
                     y_result_future.result().result.error_code == 0)
        z_success = (z_result_future.done() and
                     z_result_future.result().result.error_code == 0)

        if y_success and z_success:
            self.get_logger().info('Trajectory execution complete!')
            return True
        else:
            self.get_logger().error(
                f'Trajectory failed: Y={y_success}, Z={z_success}'
            )
            return False

    def run(self, skip_nav: bool = False, trajectory: str = 'extraction'):
        """Run the test.

        Full extraction workflow:
        1. Navigate to address (X-Z positioning)
        2. Approach box (move gripper Y so magnet reaches contact position)
        3. Execute extraction trajectory (pull box out with curved path)

        Uses TF to calculate positions - no hardcoded magnet offsets needed.
        The magnet frame position is looked up directly via TF.
        """
        # Get address coordinates
        coords = self.get_address_coordinates()
        if coords is None:
            return False

        world_x, world_y, world_z = coords
        self.get_logger().info(
            f'Address coordinates: X={world_x:.3f}, Y={world_y:.3f}, Z={world_z:.3f}'
        )
        self.get_logger().info(f'Using end effector: {self.end_effector_frame}')

        # Navigate to address (unless skipped)
        if not skip_nav:
            if not self.navigate_to_address():
                return False
            time.sleep(1.0)  # Let it settle

        # Wait for TF to be available and get fresh joint states
        rclpy.spin_once(self, timeout_sec=0.5)
        time.sleep(0.5)  # Let TF buffer fill

        # Get current joint positions
        base_y = self.current_joints.get('selector_frame_gripper_joint', 0.0)
        base_z = self.current_joints.get('main_frame_selector_frame_joint', world_z)

        self.get_logger().info(
            f'Current joint positions: Y={base_y:.4f}, Z={base_z:.4f}'
        )

        # Publish trajectory markers at the ADDRESS coordinates
        self.publish_trajectory_markers(world_x, world_y, world_z, trajectory)
        time.sleep(2.0)  # Give time to see markers in RViz

        # Calculate contact position using TF
        # 1. Get current magnet position from TF
        # 2. Calculate how much to move the gripper joint to reach the address
        magnet_pos = self.get_magnet_world_position()
        if magnet_pos is None:
            self.get_logger().error('Cannot get magnet position from TF')
            return False

        current_gripper_y = self.current_joints.get('selector_frame_gripper_joint', 0.0)
        y_sign = 1.0 if self.side == 'left' else -1.0

        # Current magnet Y position and target (address Y + penetration into box)
        current_magnet_y = magnet_pos[1]
        target_magnet_y = world_y + y_sign * self.CONTACT_PENETRATION

        # Calculate delta: how much the magnet needs to move
        delta_y = target_magnet_y - current_magnet_y

        # Since gripper joint moves in Y direction with sign:
        # magnet_world_y = base + y_sign * gripper_joint_y
        # So: delta_magnet_y = y_sign * delta_gripper_joint_y
        # Therefore: delta_gripper_joint_y = delta_y / y_sign = delta_y * y_sign
        delta_joint_y = delta_y * y_sign

        # Target gripper joint position
        contact_joint_y = current_gripper_y + delta_joint_y

        self.get_logger().info(
            f'Phase 1: Approaching box'
        )
        self.get_logger().info(
            f'  Current magnet Y: {current_magnet_y:.4f}'
        )
        self.get_logger().info(
            f'  Target magnet Y: {target_magnet_y:.4f} (address + {self.CONTACT_PENETRATION}m penetration)'
        )
        self.get_logger().info(
            f'  Joint Y: {current_gripper_y:.4f} -> {contact_joint_y:.4f} (delta={delta_joint_y:.4f})'
        )

        # Approach the box (slow movement to contact)
        if not self.approach_box(contact_joint_y):
            return False

        time.sleep(0.5)  # Brief pause at contact

        # Update positions after approach
        rclpy.spin_once(self, timeout_sec=0.1)  # Get fresh joint states
        current_y = self.current_joints.get('selector_frame_gripper_joint', contact_joint_y)
        current_z = self.current_joints.get('main_frame_selector_frame_joint', base_z)

        self.get_logger().info(
            f'Phase 2: Executing {trajectory} trajectory from Y={current_y:.4f}, Z={current_z:.4f}'
        )

        # Execute extraction trajectory
        # The trajectory waypoints are ABSOLUTE positions (0 to 0.4 for insertion, 0.4 to 0 for extraction)
        # We pass base_y=0 and base_z=current_z so the trajectory executes relative to current Z
        # but uses absolute Y positions (the extraction waypoints go from 0.4 down to 0)
        return self.execute_trajectory(trajectory, base_y=0.0, base_z=current_z)


def main():
    parser = argparse.ArgumentParser(description='Test trajectory execution')
    parser.add_argument('--side', '-s', default='left', choices=['left', 'right'])
    parser.add_argument('--cabinet', '-c', type=int, default=1)
    parser.add_argument('--row', '-r', type=int, default=3)
    parser.add_argument('--column', '-col', type=int, default=2)
    parser.add_argument('--skip-nav', action='store_true',
                        help='Skip navigation, use current position')
    parser.add_argument('--trajectory', '-t', default='extraction',
                        choices=['insertion', 'extraction'])
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryExecutionTest(args.side, args.cabinet, args.row, args.column)

    try:
        success = node.run(skip_nav=args.skip_nav, trajectory=args.trajectory)
        if success:
            node.get_logger().info('TEST PASSED')
        else:
            node.get_logger().error('TEST FAILED')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
