#!/usr/bin/env python3
"""
MoveJoint Action Server

Commands individual joints to target positions with feedback.
Uses ControllerInterface for joint control and limit validation.
PREEMPT policy: new goal cancels any executing goal.

AC-1: Server at src/move_joint_server.py
AC-2: Validates joint_name and position limits
AC-3: Uses ControllerInterface to command joint
AC-4: Feedback at 10 Hz with current_position and progress_percent
AC-5: Monitors /joint_states for target reached (0.01m tolerance)
AC-6: Returns success or aborts on timeout (30s default)
AC-7: Supports preemption (cancel during motion)
AC-8: Timeout loaded from config/action_servers.yaml
AC-11: New goal PREEMPTS current goal - no queueing
"""

import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from manipulator_control.action import MoveJoint

# Import ControllerInterface from src directory
import sys
sys.path.insert(0, os.path.dirname(__file__))
from controller_interface import ControllerInterface


class MoveJointServer(Node):
    """
    Action server for commanding individual joints to target positions.

    Uses ControllerInterface (Story 2.2) for:
    - Joint limit validation
    - Position feedback from /joint_states
    - Publishing commands to controllers

    Goal Policy: PREEMPT
    - New valid goals always accepted
    - Current executing goal is canceled when new goal arrives
    """

    def __init__(self):
        super().__init__('move_joint_server')

        # Load configuration from action_servers.yaml (AC-8)
        self._load_config()

        # Initialize ControllerInterface (shared utility from Story 2.2) (AC-3)
        self.controller = ControllerInterface(self)

        # Track current goal for preemption (AC-11)
        self._current_goal_handle = None
        self._preempt_requested = False

        # Create action server with ReentrantCallbackGroup for preemption
        self._action_server = ActionServer(
            self,
            MoveJoint,
            'move_joint',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('MoveJoint action server ready (PREEMPT policy)')
        self.get_logger().info(
            f'Config: timeout={self.timeout_sec}s, '
            f'tolerance={self.position_tolerance}m, '
            f'feedback_rate={self.feedback_rate}Hz'
        )

    def _load_config(self):
        """
        Load action server parameters from config/action_servers.yaml (AC-8).

        Parameters loaded:
        - timeout_sec: Max time to reach target (default 30.0)
        - position_tolerance: Success threshold in meters (default 0.01)
        - feedback_rate: Feedback publish rate in Hz (default 10.0)
        """
        # Defaults per story requirements
        self.timeout_sec = 30.0
        self.position_tolerance = 0.01
        self.feedback_rate = 10.0

        try:
            pkg_path = get_package_share_directory('manipulator_control')
            config_file = os.path.join(pkg_path, 'config', 'action_servers.yaml')

            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if config and 'move_joint' in config:
                mj_config = config['move_joint']
                self.timeout_sec = float(mj_config.get('timeout_sec', self.timeout_sec))
                self.position_tolerance = float(mj_config.get('position_tolerance', self.position_tolerance))
                self.feedback_rate = float(mj_config.get('feedback_rate', self.feedback_rate))
                self.get_logger().info(f'Loaded config from {config_file}')
            else:
                self.get_logger().warning('No move_joint config found, using defaults')

        except FileNotFoundError:
            self.get_logger().warning(
                'action_servers.yaml not found, using defaults: '
                f'timeout={self.timeout_sec}s, tolerance={self.position_tolerance}m'
            )
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}, using defaults')

    def goal_callback(self, goal_request):
        """
        Accept or reject incoming goal - PREEMPT policy (AC-2, AC-11).

        Validation:
        - joint_name must exist in ControllerInterface
        - target_position must be within soft limits

        Valid goals are always accepted. If a goal is currently executing,
        it will be preempted (canceled) when the new goal starts executing.
        """
        joint_name = goal_request.joint_name
        target = goal_request.target_position

        # Validate joint exists (AC-2)
        if joint_name not in self.controller.get_all_joint_names():
            self.get_logger().warning(f'Rejecting goal: unknown joint "{joint_name}"')
            self.get_logger().info(f'Valid joints: {self.controller.get_all_joint_names()}')
            return GoalResponse.REJECT

        # Validate position within soft limits (AC-2)
        limits = self.controller.get_joint_limits(joint_name)
        if limits and not (limits[0] <= target <= limits[1]):
            self.get_logger().warning(
                f'Rejecting goal: position {target:.4f} outside limits '
                f'[{limits[0]:.4f}, {limits[1]:.4f}] for {joint_name}'
            )
            return GoalResponse.REJECT

        # PREEMPT policy: signal preemption if goal is active (AC-11)
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('Preempting current goal for new goal')
            self._preempt_requested = True

        self.get_logger().info(
            f'Accepting goal: {joint_name} -> {target:.4f} '
            f'(limits: [{limits[0]:.4f}, {limits[1]:.4f}])'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept cancel request (AC-7).

        All cancel requests are accepted.
        """
        self.get_logger().info('Received cancel request - accepting')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the motion goal with feedback (AC-3, AC-4, AC-5, AC-6, AC-7, AC-11).

        Execution loop:
        1. Get start position
        2. Send command via ControllerInterface
        3. Monitor position at feedback_rate Hz
        4. Publish feedback (current_position, progress_percent)
        5. Check for success (within tolerance), timeout, cancel, or preemption
        """
        # Store handle for preemption tracking
        self._current_goal_handle = goal_handle
        self._preempt_requested = False

        joint_name = goal_handle.request.joint_name
        target = goal_handle.request.target_position

        self.get_logger().info(f'Executing: {joint_name} -> {target:.4f}')

        # Get starting position (AC-5)
        start_pos = self.controller.get_joint_position(joint_name)
        if start_pos is None:
            self.get_logger().error(f'Cannot read position for {joint_name}')
            goal_handle.abort()
            return MoveJoint.Result(
                success=False,
                final_position=0.0,
                execution_time=0.0,
                message=f'Cannot read position for {joint_name}'
            )

        self.get_logger().info(f'Start position: {start_pos:.4f}')

        # Send command via ControllerInterface (AC-3)
        if not self.controller.command_joint(joint_name, target):
            self.get_logger().error('Command rejected by ControllerInterface')
            goal_handle.abort()
            return MoveJoint.Result(
                success=False,
                final_position=start_pos,
                execution_time=0.0,
                message='Command rejected by ControllerInterface'
            )

        # Monitor until complete (AC-4, AC-5, AC-6)
        start_time = self.get_clock().now()
        feedback_msg = MoveJoint.Feedback()
        sleep_duration = 1.0 / self.feedback_rate  # Convert Hz to seconds

        while rclpy.ok():
            # Check if goal was preempted by a new goal (AC-11)
            if self._preempt_requested or not goal_handle.is_active:
                current_pos = self.controller.get_joint_position(joint_name)
                if current_pos is None:
                    current_pos = start_pos
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'Goal preempted at position {current_pos:.4f} after {elapsed:.2f}s'
                )
                # Goal already marked as canceled by new goal acceptance
                if goal_handle.is_active:
                    goal_handle.canceled()
                return MoveJoint.Result(
                    success=False,
                    final_position=current_pos,
                    execution_time=elapsed,
                    message='Preempted by new goal'
                )

            # Check for cancel request (AC-7)
            if goal_handle.is_cancel_requested:
                current_pos = self.controller.get_joint_position(joint_name)
                if current_pos is None:
                    current_pos = start_pos
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                goal_handle.canceled()
                self.get_logger().info(
                    f'Goal canceled at position {current_pos:.4f} after {elapsed:.2f}s'
                )
                return MoveJoint.Result(
                    success=False,
                    final_position=current_pos,
                    execution_time=elapsed,
                    message='Canceled by client'
                )

            # Get current position (AC-5)
            current_pos = self.controller.get_joint_position(joint_name)
            if current_pos is None:
                current_pos = start_pos  # Fallback

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

            # Check success - within tolerance (AC-5)
            if abs(current_pos - target) <= self.position_tolerance:
                goal_handle.succeed()
                self.get_logger().info(
                    f'Goal succeeded: {joint_name} at {current_pos:.4f} '
                    f'(target: {target:.4f}, error: {abs(current_pos - target):.4f}m) '
                    f'in {elapsed:.2f}s'
                )
                return MoveJoint.Result(
                    success=True,
                    final_position=current_pos,
                    execution_time=elapsed,
                    message='Target reached'
                )

            # Check timeout (AC-6)
            if elapsed > self.timeout_sec:
                goal_handle.abort()
                error = abs(current_pos - target)
                self.get_logger().warning(
                    f'Goal timeout after {elapsed:.1f}s: '
                    f'{joint_name} at {current_pos:.4f} (target: {target:.4f}, error: {error:.4f}m)'
                )
                return MoveJoint.Result(
                    success=False,
                    final_position=current_pos,
                    execution_time=elapsed,
                    message=f'Timeout after {elapsed:.1f}s, error: {error:.4f}m'
                )

            # Publish feedback (AC-4)
            feedback_msg.current_position = current_pos
            feedback_msg.progress_percent = self._calc_progress(start_pos, current_pos, target)
            goal_handle.publish_feedback(feedback_msg)

            # Sleep using time.sleep() - MultiThreadedExecutor handles concurrency
            # IMPORTANT: asyncio.sleep() doesn't work with rclpy executors
            # See: https://github.com/ros2/rclpy/issues/279
            # The official ROS2 action server tutorial uses time.sleep()
            time.sleep(sleep_duration)

    def _calc_progress(self, start: float, current: float, target: float) -> float:
        """
        Calculate progress percentage (0.0 to 100.0).

        Progress = (distance_traveled / total_distance) * 100

        Args:
            start: Starting position
            current: Current position
            target: Target position

        Returns:
            Progress percentage clamped to [0.0, 100.0]
        """
        total = abs(target - start)
        if total < 0.001:  # Already at target
            return 100.0

        traveled = abs(current - start)
        progress = (traveled / total) * 100.0
        return min(100.0, max(0.0, progress))


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointServer()

    # Use MultiThreadedExecutor to allow concurrent callback processing
    # This is essential for action servers that need to process subscriptions
    # while executing goals
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MoveJoint action server')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
