#!/usr/bin/env python3
"""
MoveJointGroup Action Server

Commands groups of joints simultaneously to target positions with feedback.
Uses ControllerInterface for joint control and limit validation.
Supports predefined named groups from kinematic_chains.yaml.

AC-1: Server at /move_joint_group
AC-2: Loads joint groups from config/kinematic_chains.yaml
AC-3: Validates target_positions length and soft limits
AC-4: Commands all joints simultaneously via ControllerInterface
AC-5: Computes aggregate progress (average of individual joint progress)
AC-6: Success when ALL joints reach targets within 0.01m tolerance or timeout (30s)
AC-7: Feedback at 10 Hz with current_positions[] and progress_percent
AC-8: Supports named groups: navigation, gripper, picker, container
AC-9: Container group implements software mimic (left = -opening/2, right = +opening/2)
AC-10: Coordinated arrival - all joints must reach targets within 1 second of each other
"""

import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from manipulator_control.action import MoveJointGroup

# Import ControllerInterface from src directory
import sys
sys.path.insert(0, os.path.dirname(__file__))
from controller_interface import ControllerInterface


class MoveJointGroupServer(Node):
    """
    Action server for commanding groups of joints to target positions simultaneously.

    Uses ControllerInterface (Story 2.2) for:
    - Joint limit validation
    - Position feedback from /joint_states
    - Publishing commands to controllers

    Supports two usage modes:
    1. Named group: goal.joint_names = ["navigation"] (single name = predefined group)
    2. Explicit joints: goal.joint_names = ["joint1", "joint2"]
    """

    def __init__(self):
        super().__init__('move_joint_group_server')

        # Load configuration (AC-2, AC-8)
        self._load_config()
        self._load_joint_groups()

        # Initialize ControllerInterface (AC-4)
        self.controller = ControllerInterface(self)

        # Create action server (AC-1)
        self._action_server = ActionServer(
            self,
            MoveJointGroup,
            'move_joint_group',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('MoveJointGroup action server ready')
        self.get_logger().info(
            f'Config: timeout={self.timeout_sec}s, '
            f'tolerance={self.position_tolerance}m, '
            f'feedback_rate={self.feedback_rate}Hz'
        )
        self.get_logger().info(f'Loaded groups: {list(self.joint_groups.keys())}')

    def _load_config(self):
        """
        Load action server parameters from config/action_servers.yaml.

        Parameters:
        - timeout_sec: Max time to reach all targets (default 30.0)
        - position_tolerance: Success threshold in meters (default 0.01)
        - feedback_rate: Feedback publish rate in Hz (default 10.0)
        """
        self.timeout_sec = 30.0
        self.position_tolerance = 0.01
        self.feedback_rate = 10.0

        try:
            pkg_path = get_package_share_directory('manipulator_control')
            config_file = os.path.join(pkg_path, 'config', 'action_servers.yaml')

            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if config and 'move_joint_group' in config:
                mjg_config = config['move_joint_group']
                self.timeout_sec = float(mjg_config.get('timeout_sec', self.timeout_sec))
                self.position_tolerance = float(mjg_config.get('position_tolerance', self.position_tolerance))
                self.feedback_rate = float(mjg_config.get('feedback_rate', self.feedback_rate))
                self.get_logger().info(f'Loaded config from {config_file}')
            else:
                self.get_logger().warning('No move_joint_group config found, using defaults')

        except FileNotFoundError:
            self.get_logger().warning(
                'action_servers.yaml not found, using defaults: '
                f'timeout={self.timeout_sec}s, tolerance={self.position_tolerance}m'
            )
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}, using defaults')

    def _load_joint_groups(self):
        """
        Load joint group definitions from config/kinematic_chains.yaml (AC-2, AC-8).
        """
        self.joint_groups = {}

        try:
            pkg_path = get_package_share_directory('manipulator_control')
            config_file = os.path.join(pkg_path, 'config', 'kinematic_chains.yaml')

            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if config and 'joint_groups' in config:
                self.joint_groups = config['joint_groups']
                self.get_logger().info(f'Loaded {len(self.joint_groups)} joint groups from {config_file}')
            else:
                self.get_logger().warning('No joint_groups found in kinematic_chains.yaml')

        except FileNotFoundError:
            self.get_logger().error('kinematic_chains.yaml not found')
        except Exception as e:
            self.get_logger().error(f'Error loading joint groups: {e}')

    def _resolve_joint_group(self, joint_names, target_positions):
        """
        Resolve joint names and positions, handling named groups and mimic mode.

        Args:
            joint_names: List of joint names OR single-element list with group name
            target_positions: List of target positions

        Returns:
            Tuple (resolved_joint_names, resolved_positions, is_mimic_mode) or (None, None, False) on error
        """
        # Check if this is a named group (single name that matches a group)
        if len(joint_names) == 1 and joint_names[0] in self.joint_groups:
            group_name = joint_names[0]
            group = self.joint_groups[group_name]
            resolved_joints = group['joints']
            is_mimic = group.get('mimic_mode', False)

            # Container mimic mode (AC-9): single value -> symmetric positions
            if is_mimic:
                if len(target_positions) != 1:
                    self.get_logger().warning(
                        f'Mimic group "{group_name}" requires single opening value, '
                        f'got {len(target_positions)}'
                    )
                    return None, None, False

                opening = target_positions[0]
                left_target = -opening / 2.0
                right_target = opening / 2.0
                resolved_positions = [left_target, right_target]

                self.get_logger().info(
                    f'Mimic mode: opening={opening:.4f} -> '
                    f'left={left_target:.4f}, right={right_target:.4f}'
                )
                return resolved_joints, resolved_positions, True
            else:
                # Non-mimic named group - positions must match joint count
                if len(target_positions) != len(resolved_joints):
                    self.get_logger().warning(
                        f'Group "{group_name}" has {len(resolved_joints)} joints, '
                        f'got {len(target_positions)} positions'
                    )
                    return None, None, False
                return resolved_joints, list(target_positions), False

        # Explicit joint names - validate count matches
        if len(joint_names) != len(target_positions):
            self.get_logger().warning(
                f'Joint count ({len(joint_names)}) != position count ({len(target_positions)})'
            )
            return None, None, False

        return list(joint_names), list(target_positions), False

    def goal_callback(self, goal_request):
        """
        Accept or reject incoming goal (AC-3).

        Validation:
        - Resolves named groups or explicit joints
        - Validates position count matches joint count
        - Validates all positions within soft limits
        """
        joint_names = list(goal_request.joint_names)
        target_positions = list(goal_request.target_positions)

        # Resolve group names and mimic mode
        resolved_joints, resolved_positions, is_mimic = self._resolve_joint_group(
            joint_names, target_positions
        )

        if resolved_joints is None:
            self.get_logger().warning('Rejecting goal: failed to resolve joints/positions')
            return GoalResponse.REJECT

        # Validate all joints exist and positions within limits (AC-3)
        all_joints = self.controller.get_all_joint_names()
        for joint_name, position in zip(resolved_joints, resolved_positions):
            if joint_name not in all_joints:
                self.get_logger().warning(f'Rejecting goal: unknown joint "{joint_name}"')
                return GoalResponse.REJECT

            limits = self.controller.get_joint_limits(joint_name)
            if limits and not (limits[0] <= position <= limits[1]):
                self.get_logger().warning(
                    f'Rejecting goal: position {position:.4f} outside limits '
                    f'[{limits[0]:.4f}, {limits[1]:.4f}] for {joint_name}'
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting goal: {len(resolved_joints)} joints -> '
            f'{[f"{p:.3f}" for p in resolved_positions]}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept cancel request.
        """
        self.get_logger().info('Received cancel request - accepting')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the motion goal with feedback (AC-4, AC-5, AC-6, AC-7, AC-10).

        Execution:
        1. Resolve joints and positions
        2. Record start positions
        3. Command all joints simultaneously
        4. Monitor at feedback_rate Hz
        5. Track completion times for coordinated arrival
        6. Return success when all within tolerance or timeout
        """
        joint_names = list(goal_handle.request.joint_names)
        target_positions = list(goal_handle.request.target_positions)

        # Resolve joints and positions
        resolved_joints, resolved_positions, is_mimic = self._resolve_joint_group(
            joint_names, target_positions
        )

        self.get_logger().info(
            f'Executing: {resolved_joints} -> {[f"{p:.3f}" for p in resolved_positions]}'
        )

        # Get start positions (AC-5)
        start_positions = []
        for joint_name in resolved_joints:
            pos = self.controller.get_joint_position(joint_name)
            if pos is None:
                self.get_logger().error(f'Cannot read position for {joint_name}')
                goal_handle.abort()
                return MoveJointGroup.Result(
                    success=False,
                    final_positions=[],
                    position_error=0.0,
                    execution_time=0.0,
                    message=f'Cannot read position for {joint_name}'
                )
            start_positions.append(pos)

        self.get_logger().info(f'Start positions: {[f"{p:.4f}" for p in start_positions]}')

        # Command all joints simultaneously (AC-4)
        if not self.controller.command_joint_group(resolved_joints, resolved_positions):
            self.get_logger().error('Command rejected by ControllerInterface')
            goal_handle.abort()
            return MoveJointGroup.Result(
                success=False,
                final_positions=start_positions,
                position_error=0.0,
                execution_time=0.0,
                message='Command rejected by ControllerInterface'
            )

        # Monitor until complete (AC-5, AC-6, AC-7, AC-10)
        start_time = self.get_clock().now()
        feedback_msg = MoveJointGroup.Feedback()
        sleep_duration = 1.0 / self.feedback_rate

        # Track completion times for coordinated arrival (AC-10)
        completion_times = {}

        while rclpy.ok():
            # Check for cancel request
            if goal_handle.is_cancel_requested:
                current_positions = self._get_current_positions(resolved_joints, start_positions)
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                goal_handle.canceled()
                self.get_logger().info(f'Goal canceled after {elapsed:.2f}s')
                return MoveJointGroup.Result(
                    success=False,
                    final_positions=current_positions,
                    position_error=self._max_error(current_positions, resolved_positions),
                    execution_time=elapsed,
                    message='Canceled by client'
                )

            # Get current positions
            current_positions = self._get_current_positions(resolved_joints, start_positions)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            current_time = time.time()

            # Track joint completion times (AC-10)
            all_within_tolerance = True
            for i, (joint_name, current, target) in enumerate(
                zip(resolved_joints, current_positions, resolved_positions)
            ):
                error = abs(current - target)
                if error <= self.position_tolerance:
                    if joint_name not in completion_times:
                        completion_times[joint_name] = current_time
                        self.get_logger().debug(f'{joint_name} reached target at {elapsed:.2f}s')
                else:
                    all_within_tolerance = False

            # Check success - all joints within tolerance (AC-6)
            if all_within_tolerance:
                # Check coordinated arrival (AC-10)
                coordination_ok = True
                if len(completion_times) == len(resolved_joints):
                    earliest = min(completion_times.values())
                    latest = max(completion_times.values())
                    arrival_window = latest - earliest
                    if arrival_window > 1.0:
                        self.get_logger().warning(
                            f'Coordinated arrival exceeded: {arrival_window:.2f}s window '
                            f'(max 1.0s allowed)'
                        )
                        coordination_ok = False
                    else:
                        self.get_logger().info(
                            f'Coordinated arrival OK: {arrival_window:.2f}s window'
                        )

                goal_handle.succeed()
                max_error = self._max_error(current_positions, resolved_positions)
                coord_msg = '' if coordination_ok else ' (coordination warning)'
                self.get_logger().info(
                    f'Goal succeeded: all {len(resolved_joints)} joints at target '
                    f'(max error: {max_error:.4f}m) in {elapsed:.2f}s{coord_msg}'
                )
                return MoveJointGroup.Result(
                    success=True,
                    final_positions=current_positions,
                    position_error=max_error,
                    execution_time=elapsed,
                    message=f'All targets reached{coord_msg}'
                )

            # Check timeout (AC-6)
            if elapsed > self.timeout_sec:
                goal_handle.abort()
                max_error = self._max_error(current_positions, resolved_positions)
                self.get_logger().warning(
                    f'Goal timeout after {elapsed:.1f}s: max error {max_error:.4f}m'
                )
                return MoveJointGroup.Result(
                    success=False,
                    final_positions=current_positions,
                    position_error=max_error,
                    execution_time=elapsed,
                    message=f'Timeout after {elapsed:.1f}s, max error: {max_error:.4f}m'
                )

            # Publish feedback (AC-7)
            feedback_msg.current_positions = current_positions
            feedback_msg.progress_percent = self._calc_aggregate_progress(
                start_positions, current_positions, resolved_positions
            )
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(sleep_duration)

    def _get_current_positions(self, joint_names, fallback_positions):
        """
        Get current positions for all joints with fallback.
        """
        positions = []
        for i, joint_name in enumerate(joint_names):
            pos = self.controller.get_joint_position(joint_name)
            if pos is None:
                pos = fallback_positions[i]
            positions.append(pos)
        return positions

    def _max_error(self, current_positions, target_positions):
        """
        Calculate maximum position error across all joints.
        """
        errors = [abs(c - t) for c, t in zip(current_positions, target_positions)]
        return max(errors) if errors else 0.0

    def _calc_aggregate_progress(self, start_positions, current_positions, target_positions):
        """
        Calculate aggregate progress percentage (AC-5).

        Average of individual joint progress values.
        Individual progress = (traveled / total_distance) * 100
        """
        total_progress = 0.0
        for start, current, target in zip(start_positions, current_positions, target_positions):
            total_distance = abs(target - start)
            if total_distance < 0.001:
                individual_progress = 100.0
            else:
                traveled = abs(current - start)
                individual_progress = min(100.0, (traveled / total_distance) * 100.0)
            total_progress += individual_progress

        return total_progress / len(start_positions) if start_positions else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointGroupServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MoveJointGroup action server')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
