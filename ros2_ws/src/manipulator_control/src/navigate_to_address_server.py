#!/usr/bin/env python3
"""
NavigateToAddress Action Server

High-level action for navigating the manipulator to any warehouse address.
Combines GetAddressCoordinates service (Story 3.2) with MoveJointGroup action
(Story 3.3) to provide a complete navigation solution.

Story 3.4 - Epic 3: Address Navigation System

AC1:  Action interface with goal (side, cabinet, row, column, approach_distance),
      result (success, final_position, positioning_error, message),
      feedback (current_position, distance_to_target, progress_percent)
AC2:  Calls GetAddressCoordinates service to resolve address to world coordinates
AC3:  Uses MoveJointGroup with joint_group="navigation" for coordinated XZ motion
AC4:  Navigation group in kinematic_chains.yaml defines end_effector_frame and coordinate_mapping
AC5:  Joint target positions computed using coordinate_mapping offsets
AC6:  Position verification via TF lookup, success requires error < 2cm (NFR-002)
AC7:  Approach distance adjusts Y coordinate based on side
AC8:  Navigation timeout of 30 seconds (configurable)
AC9:  Invalid addresses immediately rejected with error message from service
AC10: Feedback published at 5 Hz with current end effector position from TF
AC11: INFO/WARN/ERROR logging for navigation lifecycle events
"""

import os
import math
import time
import yaml
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory

from manipulator_control.action import NavigateToAddress, MoveJointGroup
from manipulator_control.srv import GetAddressCoordinates


class NavigateToAddressServer(Node):
    """
    High-level action server for warehouse address navigation.

    Orchestrates:
    1. Address resolution via GetAddressCoordinates service
    2. World-to-joint coordinate mapping using kinematic_chains.yaml config
    3. Coordinated XZ motion via MoveJointGroup action
    4. Position verification via TF lookup

    Uses TF to verify final end effector position matches target within tolerance.
    """

    # Default timeouts (overridden by action_servers.yaml)
    DEFAULT_SERVICE_TIMEOUT = 5.0  # seconds
    DEFAULT_ACTION_TIMEOUT = 30.0  # seconds
    DEFAULT_POSITION_TOLERANCE = 0.02  # 2cm per NFR-002
    DEFAULT_FEEDBACK_RATE = 5.0  # Hz

    def __init__(self):
        super().__init__('navigate_to_address_server')

        # Load configuration
        self._load_config()
        self._load_kinematic_config()

        # TF buffer for position verification (AC6)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Callback group for concurrent operations
        self._callback_group = ReentrantCallbackGroup()

        # Action server (AC1)
        self._action_server = ActionServer(
            self,
            NavigateToAddress,
            'navigate_to_address',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group
        )

        # Service client for address resolution (AC2)
        self._address_client = self.create_client(
            GetAddressCoordinates,
            '/manipulator/get_address_coordinates',
            callback_group=self._callback_group
        )

        # Action client for joint group motion (AC3)
        self._move_group_client = ActionClient(
            self,
            MoveJointGroup,
            'move_joint_group',
            callback_group=self._callback_group
        )

        self.get_logger().info(
            f'NavigateToAddress ready. End effector: {self._end_effector_frame}, '
            f'tolerance: {self._position_tolerance}m, timeout: {self._action_timeout}s'
        )

    def _load_config(self):
        """Load action server parameters from action_servers.yaml (AC8)."""
        self._service_timeout = self.DEFAULT_SERVICE_TIMEOUT
        self._action_timeout = self.DEFAULT_ACTION_TIMEOUT
        self._position_tolerance = self.DEFAULT_POSITION_TOLERANCE
        self._feedback_rate = self.DEFAULT_FEEDBACK_RATE

        try:
            pkg_path = get_package_share_directory('manipulator_control')
            config_file = os.path.join(pkg_path, 'config', 'action_servers.yaml')

            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if config and 'navigate_to_address' in config:
                nav_config = config['navigate_to_address']
                self._service_timeout = float(nav_config.get('service_timeout', self._service_timeout))
                self._action_timeout = float(nav_config.get('action_timeout', self._action_timeout))
                self._position_tolerance = float(nav_config.get('position_tolerance', self._position_tolerance))
                self._feedback_rate = float(nav_config.get('feedback_rate', self._feedback_rate))
                self.get_logger().info(f'Loaded config from {config_file}')
            else:
                self.get_logger().warning('No navigate_to_address config found, using defaults')

        except FileNotFoundError:
            self.get_logger().warning('action_servers.yaml not found, using defaults')
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}, using defaults')

    def _load_kinematic_config(self):
        """Load navigation group config from kinematic_chains.yaml (AC4)."""
        self._joints = []
        self._end_effector_frame = 'selector_frame'
        self._end_effector_offset = [0.0, 0.0, 0.0]
        self._coordinate_mapping = {}

        try:
            pkg_path = get_package_share_directory('manipulator_control')
            config_file = os.path.join(pkg_path, 'config', 'kinematic_chains.yaml')

            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if config and 'joint_groups' in config and 'navigation' in config['joint_groups']:
                nav_group = config['joint_groups']['navigation']
                self._joints = nav_group.get('joints', [])
                self._end_effector_frame = nav_group.get('end_effector_frame', 'selector_frame')
                self._end_effector_offset = nav_group.get('end_effector_offset', [0.0, 0.0, 0.0])
                self._coordinate_mapping = nav_group.get('coordinate_mapping', {})
                self.get_logger().info(
                    f'Loaded navigation config: joints={self._joints}, '
                    f'end_effector={self._end_effector_frame}, '
                    f'mapping={list(self._coordinate_mapping.keys())}'
                )
            else:
                self.get_logger().error('No navigation group found in kinematic_chains.yaml')

        except FileNotFoundError:
            self.get_logger().error('kinematic_chains.yaml not found')
        except Exception as e:
            self.get_logger().error(f'Error loading kinematic config: {e}')

    def _world_to_joint_positions(self, target_x: float, target_y: float, target_z: float) -> list:
        """
        Convert world coordinates to joint positions using coordinate_mapping (AC5).

        For each joint in the navigation group, looks up which world axis it controls
        and subtracts the offset.

        Example: z_joint = z_world - 0.301 for selector_frame joint
        """
        world_coords = {'x': target_x, 'y': target_y, 'z': target_z}
        joint_positions = []

        for joint_name in self._joints:
            mapping = self._coordinate_mapping.get(joint_name, {})
            axis = mapping.get('axis', 'x')
            offset = mapping.get('offset', 0.0)

            joint_pos = world_coords[axis] - offset
            joint_positions.append(joint_pos)

            self.get_logger().debug(
                f'Joint {joint_name}: world_{axis}={world_coords[axis]:.3f} - '
                f'offset={offset:.3f} = {joint_pos:.3f}'
            )

        return joint_positions

    def _apply_approach_distance(self, x: float, y: float, z: float,
                                  side: str, approach_distance: float) -> tuple:
        """
        Adjust Y coordinate for approach distance (AC7).

        For left cabinets: Y decreases (approach from +Y side)
        For right cabinets: Y increases (approach from -Y side)
        """
        if approach_distance <= 0:
            return x, y, z

        if side == 'left':
            adjusted_y = y - approach_distance
        else:  # right
            adjusted_y = y + approach_distance

        self.get_logger().debug(
            f'Approach distance {approach_distance}m for {side} side: '
            f'Y {y:.3f} -> {adjusted_y:.3f}'
        )
        return x, adjusted_y, z

    def _get_end_effector_position(self) -> tuple:
        """
        Look up current end effector position via TF (AC6).

        Returns: (x, y, z, success, error_msg)
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                'world',
                self._end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = transform.transform.translation.x + self._end_effector_offset[0]
            y = transform.transform.translation.y + self._end_effector_offset[1]
            z = transform.transform.translation.z + self._end_effector_offset[2]

            return (x, y, z, True, '')
        except TransformException as e:
            return (0.0, 0.0, 0.0, False, f'TF lookup failed: {e}')

    def _calc_distance(self, x1: float, y1: float, z1: float,
                       x2: float, y2: float, z2: float) -> float:
        """Calculate Euclidean distance between two 3D points."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def _calc_xz_distance(self, x1: float, z1: float, x2: float, z2: float) -> float:
        """Calculate Euclidean distance in XZ plane (navigation plane)."""
        return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

    def goal_callback(self, goal_request):
        """
        Accept or reject incoming goal (AC9).

        Validates:
        - Side is "left" or "right"
        - Cabinet number is 1-4
        - Row and column are >= 1
        - Services/actions are available
        """
        side = goal_request.side
        cabinet_num = goal_request.cabinet_num
        row = goal_request.row
        column = goal_request.column

        # Validate side
        if side not in ('left', 'right'):
            self.get_logger().warning(f'Rejecting goal: invalid side "{side}" (must be "left" or "right")')
            return GoalResponse.REJECT

        # Validate cabinet range
        if cabinet_num < 1 or cabinet_num > 4:
            self.get_logger().warning(f'Rejecting goal: cabinet_num {cabinet_num} out of range 1-4')
            return GoalResponse.REJECT

        # Validate row and column
        if row < 1:
            self.get_logger().warning(f'Rejecting goal: row {row} must be >= 1')
            return GoalResponse.REJECT
        if column < 1:
            self.get_logger().warning(f'Rejecting goal: column {column} must be >= 1')
            return GoalResponse.REJECT

        # Check service availability
        if not self._address_client.service_is_ready():
            self.get_logger().warning('Rejecting goal: GetAddressCoordinates service not available')
            return GoalResponse.REJECT

        # Check action server availability
        if not self._move_group_client.server_is_ready():
            self.get_logger().warning('Rejecting goal: MoveJointGroup action server not available')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting goal: navigate to {side}-{cabinet_num}-{row}-{column}'
        )
        return GoalResponse.ACCEPT

    def _goal_callback(self, goal_request):
        """Wrapper for goal_callback."""
        return self.goal_callback(goal_request)

    def _cancel_callback(self, goal_handle):
        """Accept cancel request."""
        self.get_logger().info('Received cancel request - accepting')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """
        Execute navigation to address.

        Steps:
        1. Resolve address to world coordinates (AC2)
        2. Apply approach distance offset (AC7)
        3. Convert world to joint positions (AC5)
        4. Execute MoveJointGroup motion (AC3)
        5. Verify position via TF (AC6)
        6. Return result
        """
        goal = goal_handle.request
        start_time = time.time()

        self.get_logger().info(
            f'Navigating to {goal.side}-{goal.cabinet_num}-{goal.row}-{goal.column} '
            f'(approach: {goal.approach_distance:.3f}m)'
        )

        # Step 1: Resolve address to world coordinates (AC2, AC9)
        resolve_result = self._resolve_address_sync(goal)
        if not resolve_result['success']:
            self.get_logger().error(f'Address resolution failed: {resolve_result["message"]}')
            goal_handle.abort()
            return self._create_result(False, Point(), 0.0, resolve_result['message'])

        target_x = resolve_result['x']
        target_y = resolve_result['y']
        target_z = resolve_result['z']

        # Step 2: Apply approach distance (AC7)
        target_x, target_y, target_z = self._apply_approach_distance(
            target_x, target_y, target_z,
            goal.side, goal.approach_distance
        )

        self.get_logger().info(
            f'World target: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})'
        )

        # Step 3: Convert world coordinates to joint positions (AC5)
        joint_targets = self._world_to_joint_positions(target_x, target_y, target_z)
        self.get_logger().info(
            f'Joint targets: {[f"{p:.3f}" for p in joint_targets]}'
        )

        # Step 4: Execute navigation via MoveJointGroup (AC3, AC8, AC10)
        motion_result = self._execute_navigation_sync(
            goal_handle, joint_targets, target_x, target_y, target_z
        )
        if not motion_result['success']:
            self.get_logger().error(f'Motion execution failed: {motion_result["message"]}')
            goal_handle.abort()
            return self._create_result(False, Point(), 0.0, motion_result['message'])

        # Step 5: Verify final position via TF (AC6)
        actual_x, actual_y, actual_z, tf_success, tf_error = self._get_end_effector_position()
        if not tf_success:
            self.get_logger().error(f'Position verification failed: {tf_error}')
            goal_handle.abort()
            return self._create_result(False, Point(), 0.0, tf_error)

        # Calculate positioning error in XZ plane (navigation plane)
        positioning_error = self._calc_xz_distance(target_x, target_z, actual_x, actual_z)

        self.get_logger().info(
            f'Navigation complete: end_effector at ({actual_x:.3f}, {actual_y:.3f}, {actual_z:.3f}), '
            f'error={positioning_error:.4f}m'
        )

        # Check tolerance (AC6)
        if positioning_error > self._position_tolerance:
            message = f'Position error {positioning_error:.4f}m exceeds tolerance {self._position_tolerance}m'
            self.get_logger().error(message)
            goal_handle.abort()
            return self._create_result(
                False,
                Point(x=actual_x, y=actual_y, z=actual_z),
                positioning_error,
                message
            )

        # Warn if approaching tolerance (AC11)
        if positioning_error > self._position_tolerance * 0.75:
            self.get_logger().warning(f'Position error approaching tolerance: {positioning_error:.4f}m')

        # Success!
        execution_time = time.time() - start_time
        goal_handle.succeed()
        return self._create_result(
            True,
            Point(x=actual_x, y=actual_y, z=actual_z),
            positioning_error,
            f'Navigation complete in {execution_time:.2f}s, error={positioning_error:.4f}m'
        )

    def _resolve_address_sync(self, goal) -> dict:
        """
        Resolve address to world coordinates via GetAddressCoordinates service (AC2).
        Uses async call with threading.Event for synchronization - works with
        MultiThreadedExecutor + ReentrantCallbackGroup.

        The done_callback extracts the result from the future in the executor context
        where it's safe to do so.

        Returns: dict with 'success', 'message', 'x', 'y', 'z'
        """
        request = GetAddressCoordinates.Request()
        request.side = goal.side
        request.cabinet_num = goal.cabinet_num
        request.row = goal.row
        request.column = goal.column

        # Use threading.Event for proper synchronization
        event = threading.Event()
        result_holder = {'response': None, 'exception': None}

        def done_callback(future):
            # Extract result from future in the executor's context
            try:
                # Access the internal result directly to avoid event loop issues
                # rclpy Future stores result in _result attribute when done
                if hasattr(future, '_result'):
                    result_holder['response'] = future._result
                else:
                    # Fallback - try to get result normally
                    result_holder['response'] = future.result()
            except Exception as e:
                result_holder['exception'] = e
            event.set()

        try:
            future = self._address_client.call_async(request)
            future.add_done_callback(done_callback)

            # Wait for completion with timeout
            if not event.wait(timeout=self._service_timeout):
                future.cancel()
                return {
                    'success': False,
                    'message': f'GetAddressCoordinates service timeout ({self._service_timeout}s)',
                    'x': 0.0, 'y': 0.0, 'z': 0.0
                }

            if result_holder['exception']:
                return {
                    'success': False,
                    'message': f'Service call exception: {result_holder["exception"]}',
                    'x': 0.0, 'y': 0.0, 'z': 0.0
                }

            response = result_holder['response']

            if response is None:
                return {
                    'success': False,
                    'message': 'GetAddressCoordinates returned None',
                    'x': 0.0, 'y': 0.0, 'z': 0.0
                }

            if not response.success:
                return {
                    'success': False,
                    'message': response.error_message or 'Address resolution failed',
                    'x': 0.0, 'y': 0.0, 'z': 0.0
                }

            return {
                'success': True,
                'message': '',
                'x': response.pose.position.x,
                'y': response.pose.position.y,
                'z': response.pose.position.z
            }

        except Exception as e:
            return {
                'success': False,
                'message': f'Service call exception: {e}',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            }

    def _execute_navigation_sync(self, goal_handle, joint_targets: list,
                                   target_x: float, target_y: float, target_z: float) -> dict:
        """
        Execute navigation via MoveJointGroup action (AC3, AC8, AC10).
        Uses threading.Event for synchronization - works with MultiThreadedExecutor.

        Returns: dict with 'success', 'message'
        """
        # Create MoveJointGroup goal
        move_goal = MoveJointGroup.Goal()
        move_goal.joint_names = ['navigation']  # Named group
        move_goal.target_positions = joint_targets
        move_goal.max_velocity = 0.0  # Use default from kinematic_chains.yaml

        # Use threading.Event for goal acceptance
        goal_accepted_event = threading.Event()
        goal_holder = {'handle': None, 'exception': None}

        def goal_response_callback(future):
            # Extract result from future in the executor's context
            try:
                if hasattr(future, '_result'):
                    goal_holder['handle'] = future._result
                else:
                    goal_holder['handle'] = future.result()
            except Exception as e:
                goal_holder['exception'] = e
            goal_accepted_event.set()

        # Send goal
        send_goal_future = self._move_group_client.send_goal_async(
            move_goal,
            feedback_callback=lambda fb: self._navigation_feedback_callback(
                fb, goal_handle, target_x, target_y, target_z
            )
        )
        send_goal_future.add_done_callback(goal_response_callback)

        # Wait for goal to be accepted with timeout
        if not goal_accepted_event.wait(timeout=5.0):
            return {'success': False, 'message': 'Timeout waiting for goal acceptance'}

        if goal_holder['exception']:
            return {'success': False, 'message': f'Failed to send goal: {goal_holder["exception"]}'}

        goal_handle_move = goal_holder['handle']
        if goal_handle_move is None or not goal_handle_move.accepted:
            return {'success': False, 'message': 'MoveJointGroup goal rejected'}

        # Use threading.Event for result
        result_event = threading.Event()
        result_holder = {'result': None, 'exception': None}

        def result_callback(future):
            # Extract result from future in the executor's context
            try:
                if hasattr(future, '_result'):
                    result_holder['result'] = future._result
                else:
                    result_holder['result'] = future.result()
            except Exception as e:
                result_holder['exception'] = e
            result_event.set()

        result_future = goal_handle_move.get_result_async()
        result_future.add_done_callback(result_callback)

        # Wait for result with timeout, checking for cancel requests
        start_time = time.time()
        while not result_event.is_set():
            # Check for cancel
            if goal_handle.is_cancel_requested:
                goal_handle_move.cancel_goal_async()
                return {'success': False, 'message': 'Canceled by client'}

            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > self._action_timeout:
                goal_handle_move.cancel_goal_async()
                return {'success': False, 'message': f'Navigation timeout exceeded ({self._action_timeout}s)'}

            # Wait with short timeout to allow cancel checks
            result_event.wait(timeout=0.1)

        if result_holder['exception']:
            return {'success': False, 'message': f'Action result exception: {result_holder["exception"]}'}

        result = result_holder['result']

        if result.result.success:
            return {'success': True, 'message': result.result.message}
        else:
            return {'success': False, 'message': result.result.message}

    def _navigation_feedback_callback(self, feedback_msg, goal_handle,
                                       target_x: float, target_y: float, target_z: float):
        """
        Process MoveJointGroup feedback and publish NavigateToAddress feedback (AC10).
        """
        # Get current end effector position via TF
        x, y, z, success, _ = self._get_end_effector_position()
        if not success:
            return  # Skip feedback if TF lookup fails

        # Calculate distance to target (XZ plane)
        distance = self._calc_xz_distance(target_x, target_z, x, z)

        # Create and publish feedback
        nav_feedback = NavigateToAddress.Feedback()
        nav_feedback.current_position = Point(x=x, y=y, z=z)
        nav_feedback.distance_to_target = distance
        nav_feedback.progress_percent = int(feedback_msg.feedback.progress_percent)

        goal_handle.publish_feedback(nav_feedback)

    def _create_result(self, success: bool, final_position: Point,
                       positioning_error: float, message: str) -> NavigateToAddress.Result:
        """Create NavigateToAddress result message."""
        result = NavigateToAddress.Result()
        result.success = success
        result.final_position = final_position
        result.positioning_error = positioning_error
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToAddressServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NavigateToAddress action server')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
