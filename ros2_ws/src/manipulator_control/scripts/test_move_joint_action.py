#!/usr/bin/env python3
"""
MoveJoint Action Test Script

Comprehensive test for the MoveJoint action server (Story 2.3).
Tests all acceptance criteria including validation, feedback, and preemption.

Usage:
    ros2 run manipulator_control test_move_joint_action

Prerequisites:
    - Simulation running: ros2 launch manipulator_control manipulator_simulation.launch.py
    - Wait 5-10 seconds for controllers and action server to initialize
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus

from manipulator_control.action import MoveJoint


class MoveJointTestClient(Node):
    """Test client for MoveJoint action server."""

    def __init__(self):
        super().__init__('move_joint_test_client')
        self._action_client = ActionClient(self, MoveJoint, 'move_joint')
        self._feedback_count = 0
        self._last_feedback = None

    def wait_for_server(self, timeout_sec=10.0):
        """Wait for action server to be available."""
        self.get_logger().info('Waiting for MoveJoint action server...')
        if not self._action_client.wait_for_server(timeout_sec):
            self.get_logger().error('Action server not available!')
            return False
        self.get_logger().info('Action server is available')
        return True

    def send_goal(self, joint_name: str, target_position: float,
                  max_velocity: float = 0.0, wait: bool = True):
        """
        Send a goal to the action server.

        Args:
            joint_name: Name of joint to move
            target_position: Target position in meters
            max_velocity: Max velocity (0 = use default)
            wait: If True, wait for result

        Returns:
            Goal handle if wait=False, Result if wait=True
        """
        goal_msg = MoveJoint.Goal()
        goal_msg.joint_name = joint_name
        goal_msg.target_position = target_position
        goal_msg.max_velocity = max_velocity

        self._feedback_count = 0
        self._last_feedback = None

        self.get_logger().info(
            f'Sending goal: {joint_name} -> {target_position:.4f}'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warning('Goal was rejected!')
            return None

        self.get_logger().info('Goal accepted')

        if not wait:
            return goal_handle

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        return result

    def _feedback_callback(self, feedback_msg):
        """Process feedback from action server."""
        feedback = feedback_msg.feedback
        self._feedback_count += 1
        self._last_feedback = feedback
        if self._feedback_count % 5 == 0:  # Log every 5th feedback
            self.get_logger().info(
                f'Feedback #{self._feedback_count}: '
                f'pos={feedback.current_position:.4f}, '
                f'progress={feedback.progress_percent:.1f}%'
            )


def test_valid_goal(client: MoveJointTestClient):
    """Test AC-3, AC-4, AC-5: Valid goal with feedback and success."""
    print('\n' + '=' * 60)
    print('TEST 1: Valid Goal (AC-3, AC-4, AC-5)')
    print('=' * 60)

    result = client.send_goal('base_main_frame_joint', 1.5)

    if result is None:
        print('FAIL: Goal rejected')
        return False

    status = result.status
    result_msg = result.result

    print(f'Status: {status}')
    print(f'Success: {result_msg.success}')
    print(f'Final position: {result_msg.final_position:.4f}')
    print(f'Execution time: {result_msg.execution_time:.2f}s')
    print(f'Message: {result_msg.message}')
    print(f'Feedback count: {client._feedback_count}')

    passed = (
        result_msg.success and
        abs(result_msg.final_position - 1.5) <= 0.01 and
        client._feedback_count > 0
    )

    print(f'RESULT: {"PASS" if passed else "FAIL"}')
    return passed


def test_invalid_joint(client: MoveJointTestClient):
    """Test AC-2: Invalid joint name rejection."""
    print('\n' + '=' * 60)
    print('TEST 2: Invalid Joint Name (AC-2)')
    print('=' * 60)

    result = client.send_goal('fake_joint', 1.0)

    passed = result is None
    print(f'Goal rejected: {passed}')
    print(f'RESULT: {"PASS" if passed else "FAIL"}')
    return passed


def test_out_of_range(client: MoveJointTestClient):
    """Test AC-2: Out-of-range position rejection."""
    print('\n' + '=' * 60)
    print('TEST 3: Out-of-Range Position (AC-2)')
    print('=' * 60)

    # base_main_frame_joint limits: [0.1, 3.9]
    result = client.send_goal('base_main_frame_joint', 99.0)

    passed = result is None
    print(f'Goal rejected: {passed}')
    print(f'RESULT: {"PASS" if passed else "FAIL"}')
    return passed


def test_position_tolerance(client: MoveJointTestClient):
    """Test AC-5: Position tolerance 0.01m."""
    print('\n' + '=' * 60)
    print('TEST 4: Position Tolerance (AC-5)')
    print('=' * 60)

    target = 0.75
    result = client.send_goal('main_frame_selector_frame_joint', target)

    if result is None:
        print('FAIL: Goal rejected')
        return False

    result_msg = result.result
    error = abs(result_msg.final_position - target)

    print(f'Target: {target:.4f}')
    print(f'Final: {result_msg.final_position:.4f}')
    print(f'Error: {error:.4f}m')

    passed = result_msg.success and error <= 0.01
    print(f'RESULT: {"PASS" if passed else "FAIL"}')
    return passed


def test_feedback_rate(client: MoveJointTestClient):
    """Test AC-4: Feedback at approximately 10 Hz."""
    print('\n' + '=' * 60)
    print('TEST 5: Feedback Rate (AC-4)')
    print('=' * 60)

    # Move a longer distance to get more feedback
    result = client.send_goal('base_main_frame_joint', 3.0)

    if result is None:
        print('FAIL: Goal rejected')
        return False

    result_msg = result.result
    execution_time = result_msg.execution_time
    expected_feedback = int(execution_time * 10)  # 10 Hz

    print(f'Execution time: {execution_time:.2f}s')
    print(f'Expected feedback (~10Hz): ~{expected_feedback}')
    print(f'Actual feedback count: {client._feedback_count}')

    # Allow some tolerance (within 50% of expected)
    passed = client._feedback_count >= expected_feedback * 0.5
    print(f'RESULT: {"PASS" if passed else "FAIL"}')
    return passed


def main():
    print('=' * 60)
    print('MoveJoint Action Server Test Suite')
    print('Story 2.3 - AC Validation')
    print('=' * 60)

    rclpy.init()
    client = MoveJointTestClient()

    if not client.wait_for_server():
        print('\nFATAL: Cannot run tests - action server not available')
        rclpy.shutdown()
        return

    results = {}

    # Run tests
    results['valid_goal'] = test_valid_goal(client)
    results['invalid_joint'] = test_invalid_joint(client)
    results['out_of_range'] = test_out_of_range(client)
    results['position_tolerance'] = test_position_tolerance(client)
    results['feedback_rate'] = test_feedback_rate(client)

    # Summary
    print('\n' + '=' * 60)
    print('TEST SUMMARY')
    print('=' * 60)
    passed = 0
    failed = 0
    for name, result in results.items():
        status = 'PASS' if result else 'FAIL'
        print(f'  {name}: {status}')
        if result:
            passed += 1
        else:
            failed += 1

    print(f'\nTotal: {passed} passed, {failed} failed')
    print('=' * 60)

    # Manual test instructions
    print('\n' + '=' * 60)
    print('MANUAL TESTS REQUIRED')
    print('=' * 60)
    print('''
The following tests require manual execution:

1. CANCEL TEST (AC-7):
   Terminal 1: ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \\
               "{joint_name: 'base_main_frame_joint', target_position: 3.5}" --feedback &
   Terminal 2: sleep 2 && ros2 action cancel /move_joint
   Expected: Goal status changes to CANCELED

2. PREEMPTION TEST (AC-11):
   Terminal 1: ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \\
               "{joint_name: 'base_main_frame_joint', target_position: 3.5}" --feedback &
   Wait 2 seconds, then in same terminal:
   ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \\
               "{joint_name: 'base_main_frame_joint', target_position: 0.5}" --feedback
   Expected: First goal CANCELED, second goal SUCCEEDED

3. CLI TEST (AC-9):
   ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \\
       "{joint_name: 'base_main_frame_joint', target_position: 2.0, max_velocity: 0.0}"
   Expected: Goal accepted and completed

4. TIMEOUT TEST (AC-6):
   Configure very short timeout, then test (requires config change)
''')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
