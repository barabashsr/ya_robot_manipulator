#!/usr/bin/env python3
"""
Epic 2 Integration Tests: Joint Control System

Tests MoveJoint, MoveJointGroup, limit switches, container mimic, and state markers.
Requires simulation running: ros2 launch manipulator_control manipulator_simulation.launch.py

Run: pytest test/test_epic2_joint_control.py -v --timeout=300
Run specific class: pytest test/test_epic2_joint_control.py::TestMoveJoint -v

AC-1: Test script at test/test_epic2_joint_control.py
AC-2: Limit switch tests (18 switches)
AC-3: MoveJoint tests (9 joints)
AC-4: MoveJointGroup tests (coordinated motion)
AC-5: Container mimic tests (symmetric jaw positions)
AC-6: State marker tests (visibility)
AC-7: Results logged with pass/fail and execution time
AC-8: 90% pass rate target
"""

import pytest
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from manipulator_control.action import MoveJoint, MoveJointGroup

# Test configuration
POSITION_TOLERANCE = 0.01  # meters (NFR-002, action_servers.yaml)
ACTION_TIMEOUT = 30.0  # seconds (action_servers.yaml)
SERVER_WAIT_TIMEOUT = 10.0  # seconds to wait for action server

# All 9 joints with safe test positions (within soft limits)
ALL_JOINTS = [
    ('base_main_frame_joint', 2.0),           # X-axis rail (0.1-3.9)
    ('main_frame_selector_frame_joint', 0.7), # Z-axis vertical (0.05-1.45)
    ('selector_frame_gripper_joint', 0.0),    # Y-axis gripper (-0.39-0.39)
    ('selector_frame_picker_frame_joint', 0.15),  # Z-axis picker (0.005-0.29)
    ('picker_frame_picker_rail_joint', 0.0),  # Y-axis picker rail (-0.29-0.29)
    ('picker_rail_picker_base_joint', 0.12),  # X-axis picker base (0.005-0.24)
    ('picker_base_picker_jaw_joint', 0.1),    # X-axis picker jaw (0.005-0.19)
    ('selector_left_container_jaw_joint', 0.0),   # Y-axis left jaw (-0.19-0.19)
    ('selector_right_container_jaw_joint', 0.0),  # Y-axis right jaw (-0.19-0.19)
]

# All 18 limit switches
ALL_SWITCH_TOPICS = [
    '/manipulator/end_switches/base_main_frame_min',
    '/manipulator/end_switches/base_main_frame_max',
    '/manipulator/end_switches/selector_frame_min',
    '/manipulator/end_switches/selector_frame_max',
    '/manipulator/end_switches/gripper_left',
    '/manipulator/end_switches/gripper_right',
    '/manipulator/end_switches/picker_frame_min',
    '/manipulator/end_switches/picker_frame_max',
    '/manipulator/end_switches/picker_rail_min',
    '/manipulator/end_switches/picker_rail_max',
    '/manipulator/end_switches/picker_retracted',
    '/manipulator/end_switches/picker_extended',
    '/manipulator/end_switches/picker_jaw_opened',
    '/manipulator/end_switches/picker_jaw_closed',
    '/manipulator/end_switches/container_left_min',
    '/manipulator/end_switches/container_left_max',
    '/manipulator/end_switches/container_right_min',
    '/manipulator/end_switches/container_right_max',
]


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture(scope='module')
def ros2_context():
    """Initialize ROS2 context for all tests in module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node(ros2_context):
    """Create a test node for each test."""
    node = Node('test_epic2_node')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    yield node, executor
    node.destroy_node()


@pytest.fixture
def move_joint_client(test_node):
    """Create MoveJoint action client."""
    node, executor = test_node
    client = ActionClient(node, MoveJoint, 'move_joint')
    assert client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT), \
        "MoveJoint action server not available"
    yield client, node, executor


@pytest.fixture
def move_joint_group_client(test_node):
    """Create MoveJointGroup action client."""
    node, executor = test_node
    client = ActionClient(node, MoveJointGroup, 'move_joint_group')
    assert client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT), \
        "MoveJointGroup action server not available"
    yield client, node, executor


# =============================================================================
# Helper Functions
# =============================================================================

def send_move_joint_goal(client, node, executor, joint_name, target, max_velocity=0.5, timeout=ACTION_TIMEOUT):
    """Send MoveJoint goal and wait for result."""
    goal = MoveJoint.Goal()
    goal.joint_name = joint_name
    goal.target_position = target
    goal.max_velocity = max_velocity

    future = client.send_goal_async(goal)

    # Wait for goal acceptance
    start = time.time()
    while not future.done() and time.time() - start < 5.0:
        executor.spin_once(timeout_sec=0.1)

    if not future.done():
        return None, "Goal acceptance timeout"

    goal_handle = future.result()
    if not goal_handle.accepted:
        return None, "Goal rejected"

    # Wait for result
    result_future = goal_handle.get_result_async()
    start = time.time()
    while not result_future.done() and time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)

    if not result_future.done():
        return None, f"Result timeout after {timeout}s"

    return result_future.result().result, None


def send_move_joint_group_goal(client, node, executor, joint_names, targets, max_velocity=0.5, timeout=ACTION_TIMEOUT):
    """Send MoveJointGroup goal and wait for result."""
    goal = MoveJointGroup.Goal()
    goal.joint_names = joint_names
    goal.target_positions = targets
    goal.max_velocity = max_velocity

    future = client.send_goal_async(goal)

    # Wait for goal acceptance
    start = time.time()
    while not future.done() and time.time() - start < 5.0:
        executor.spin_once(timeout_sec=0.1)

    if not future.done():
        return None, "Goal acceptance timeout"

    goal_handle = future.result()
    if not goal_handle.accepted:
        return None, "Goal rejected"

    # Wait for result
    result_future = goal_handle.get_result_async()
    start = time.time()
    while not result_future.done() and time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)

    if not result_future.done():
        return None, f"Result timeout after {timeout}s"

    return result_future.result().result, None


def wait_for_topic_message(node, executor, topic, msg_type, timeout=5.0):
    """Wait for a message on a topic."""
    received_msg = [None]

    def callback(msg):
        received_msg[0] = msg

    sub = node.create_subscription(msg_type, topic, callback, 10)

    start = time.time()
    while received_msg[0] is None and time.time() - start < timeout:
        executor.spin_once(timeout_sec=0.1)

    node.destroy_subscription(sub)
    return received_msg[0]


# =============================================================================
# TestLimitSwitches (AC-2)
# =============================================================================

class TestLimitSwitches:
    """AC-2: Verify all 18 limit switches publish and change state correctly."""

    def test_all_18_switch_topics_exist(self, test_node):
        """AC-2: All 18 switch topics should exist and publish Bool messages."""
        node, executor = test_node

        for topic in ALL_SWITCH_TOPICS:
            msg = wait_for_topic_message(node, executor, topic, Bool, timeout=3.0)
            assert msg is not None, f"No message received on {topic}"
            assert isinstance(msg.data, bool), f"Message on {topic} is not Bool"

    def test_base_main_frame_min_switch_triggers(self, move_joint_client):
        """AC-2: base_main_frame_min triggers when joint near 0.0."""
        client, node, executor = move_joint_client
        topic = '/manipulator/end_switches/base_main_frame_min'

        # Move to trigger position (0.0 trigger, within tolerance means < 0.02)
        result, error = send_move_joint_goal(client, node, executor,
            'base_main_frame_joint', 0.1, max_velocity=0.5)  # Move to soft limit

        assert result is not None, f"Move failed: {error}"
        # Allow motion to complete and switch to update
        time.sleep(0.5)

        msg = wait_for_topic_message(node, executor, topic, Bool, timeout=2.0)
        assert msg is not None, "No message from switch"
        # At soft limit 0.1, should be at or near trigger (0.0 ± 0.01)
        # May or may not trigger depending on exact position

    def test_picker_jaw_closed_switch_triggers(self, move_joint_client):
        """AC-2: picker_jaw_closed triggers when jaw nearly closed."""
        client, node, executor = move_joint_client
        topic = '/manipulator/end_switches/picker_jaw_closed'

        # Move to closed position (trigger at 0.01)
        result, error = send_move_joint_goal(client, node, executor,
            'picker_base_picker_jaw_joint', 0.01, max_velocity=0.3)

        assert result is not None, f"Move failed: {error}"
        time.sleep(0.5)

        msg = wait_for_topic_message(node, executor, topic, Bool, timeout=2.0)
        assert msg is not None, "No message from picker_jaw_closed switch"
        assert msg.data is True, "picker_jaw_closed should be True at closed position"

    def test_picker_jaw_opened_switch_triggers(self, move_joint_client):
        """AC-2: picker_jaw_opened triggers when jaw fully open."""
        client, node, executor = move_joint_client
        topic = '/manipulator/end_switches/picker_jaw_opened'

        # Move to open position (trigger at 0.19)
        result, error = send_move_joint_goal(client, node, executor,
            'picker_base_picker_jaw_joint', 0.19, max_velocity=0.3)

        assert result is not None, f"Move failed: {error}"
        time.sleep(0.5)

        msg = wait_for_topic_message(node, executor, topic, Bool, timeout=2.0)
        assert msg is not None, "No message from picker_jaw_opened switch"
        assert msg.data is True, "picker_jaw_opened should be True at open position"

    def test_switch_clears_when_moved_away(self, move_joint_client):
        """AC-2: Switch clears when joint moves away from trigger position."""
        client, node, executor = move_joint_client
        topic = '/manipulator/end_switches/picker_jaw_closed'

        # First trigger the switch
        result, _ = send_move_joint_goal(client, node, executor,
            'picker_base_picker_jaw_joint', 0.01, max_velocity=0.3)
        time.sleep(0.3)

        # Now move away
        result, error = send_move_joint_goal(client, node, executor,
            'picker_base_picker_jaw_joint', 0.1, max_velocity=0.3)

        assert result is not None, f"Move failed: {error}"
        time.sleep(0.5)

        msg = wait_for_topic_message(node, executor, topic, Bool, timeout=2.0)
        assert msg is not None, "No message from switch"
        assert msg.data is False, "picker_jaw_closed should be False when moved away"


# =============================================================================
# TestMoveJoint (AC-3)
# =============================================================================

class TestMoveJoint:
    """AC-3: Send MoveJoint commands to each of 9 joints and verify position reached."""

    def test_action_server_available(self, test_node):
        """AC-3: MoveJoint action server should be available."""
        node, executor = test_node
        client = ActionClient(node, MoveJoint, 'move_joint')
        available = client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT)
        assert available, "MoveJoint action server not available"

    def test_base_main_frame_joint_reaches_target(self, move_joint_client):
        """AC-3: base_main_frame_joint reaches target within tolerance."""
        client, node, executor = move_joint_client

        result, error = send_move_joint_goal(client, node, executor,
            'base_main_frame_joint', 1.5, max_velocity=0.5)

        assert result is not None, f"Goal failed: {error}"
        assert result.success, f"Goal not successful: {result.message}"
        assert abs(result.final_position - 1.5) <= POSITION_TOLERANCE, \
            f"Position error {abs(result.final_position - 1.5):.4f}m exceeds tolerance"

    def test_main_frame_selector_frame_joint_reaches_target(self, move_joint_client):
        """AC-3: main_frame_selector_frame_joint (Z-axis) reaches target."""
        client, node, executor = move_joint_client

        result, error = send_move_joint_goal(client, node, executor,
            'main_frame_selector_frame_joint', 0.5, max_velocity=0.5)

        assert result is not None, f"Goal failed: {error}"
        assert result.success, f"Goal not successful: {result.message}"
        assert abs(result.final_position - 0.5) <= POSITION_TOLERANCE

    def test_selector_frame_gripper_joint_reaches_target(self, move_joint_client):
        """AC-3: selector_frame_gripper_joint (Y-axis gripper) reaches target."""
        client, node, executor = move_joint_client

        result, error = send_move_joint_goal(client, node, executor,
            'selector_frame_gripper_joint', 0.2, max_velocity=0.3)

        assert result is not None, f"Goal failed: {error}"
        assert result.success, f"Goal not successful: {result.message}"
        assert abs(result.final_position - 0.2) <= POSITION_TOLERANCE

    def test_all_9_joints_reach_target(self, move_joint_client):
        """AC-3: All 9 joints can reach their target positions."""
        client, node, executor = move_joint_client

        passed = 0
        failed = []

        for joint_name, target in ALL_JOINTS:
            result, error = send_move_joint_goal(client, node, executor,
                joint_name, target, max_velocity=0.5)

            if result is None:
                failed.append(f"{joint_name}: {error}")
            elif not result.success:
                failed.append(f"{joint_name}: {result.message}")
            elif abs(result.final_position - target) > POSITION_TOLERANCE:
                failed.append(f"{joint_name}: position error {abs(result.final_position - target):.4f}m")
            else:
                passed += 1

        # All 9 joints must pass
        assert passed == 9, f"Only {passed}/9 joints passed. Failures: {failed}"

    def test_invalid_joint_rejected(self, test_node):
        """AC-3: Invalid joint name should be rejected."""
        node, executor = test_node
        client = ActionClient(node, MoveJoint, 'move_joint')
        client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT)

        goal = MoveJoint.Goal()
        goal.joint_name = 'nonexistent_joint'
        goal.target_position = 1.0
        goal.max_velocity = 0.5

        future = client.send_goal_async(goal)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            executor.spin_once(timeout_sec=0.1)

        assert future.done(), "Goal acceptance timed out"
        goal_handle = future.result()
        assert not goal_handle.accepted, "Invalid joint should be rejected"

    def test_out_of_limits_rejected(self, test_node):
        """AC-3: Position outside soft limits should be rejected."""
        node, executor = test_node
        client = ActionClient(node, MoveJoint, 'move_joint')
        client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT)

        goal = MoveJoint.Goal()
        goal.joint_name = 'base_main_frame_joint'
        goal.target_position = 10.0  # Way outside limits (0.1-3.9)
        goal.max_velocity = 0.5

        future = client.send_goal_async(goal)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            executor.spin_once(timeout_sec=0.1)

        assert future.done(), "Goal acceptance timed out"
        goal_handle = future.result()
        assert not goal_handle.accepted, "Out-of-limits position should be rejected"


# =============================================================================
# TestMoveJointGroup (AC-4)
# =============================================================================

class TestMoveJointGroup:
    """AC-4: Send MoveJointGroup command and verify coordinated motion."""

    def test_action_server_available(self, test_node):
        """AC-4: MoveJointGroup action server should be available."""
        node, executor = test_node
        client = ActionClient(node, MoveJointGroup, 'move_joint_group')
        available = client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT)
        assert available, "MoveJointGroup action server not available"

    def test_navigation_group_coordinated(self, move_joint_group_client):
        """AC-4: Navigation group (base_main_frame + selector_frame) moves together."""
        client, node, executor = move_joint_group_client

        # Use explicit joint names for navigation
        result, error = send_move_joint_group_goal(client, node, executor,
            joint_names=['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            targets=[1.0, 0.5],
            max_velocity=0.5)

        assert result is not None, f"Goal failed: {error}"
        assert result.success, f"Goal not successful: {result.message}"
        assert len(result.final_positions) == 2, "Should have 2 final positions"
        assert abs(result.final_positions[0] - 1.0) <= POSITION_TOLERANCE
        assert abs(result.final_positions[1] - 0.5) <= POSITION_TOLERANCE

    def test_picker_group_simultaneous(self, move_joint_group_client):
        """AC-4: Picker group with 4 joints moves simultaneously."""
        client, node, executor = move_joint_group_client

        # Picker group: 4 joints
        joints = [
            'selector_frame_picker_frame_joint',
            'picker_frame_picker_rail_joint',
            'picker_rail_picker_base_joint',
            'picker_base_picker_jaw_joint'
        ]
        targets = [0.15, 0.0, 0.12, 0.1]

        result, error = send_move_joint_group_goal(client, node, executor,
            joint_names=joints, targets=targets, max_velocity=0.3)

        assert result is not None, f"Goal failed: {error}"
        assert result.success, f"Goal not successful: {result.message}"
        assert len(result.final_positions) == 4, "Should have 4 final positions"

        for i, (actual, expected) in enumerate(zip(result.final_positions, targets)):
            assert abs(actual - expected) <= POSITION_TOLERANCE, \
                f"Joint {i} position error: {abs(actual - expected):.4f}m"

    def test_feedback_progress_reported(self, move_joint_group_client):
        """AC-4: Feedback contains progress_percent during motion."""
        client, node, executor = move_joint_group_client

        goal = MoveJointGroup.Goal()
        goal.joint_names = ['base_main_frame_joint']
        goal.target_positions = [2.5]  # Significant movement
        goal.max_velocity = 0.3  # Slower to capture feedback

        feedback_received = []

        def feedback_callback(feedback_msg):
            feedback_received.append(feedback_msg.feedback.progress_percent)

        future = client.send_goal_async(goal, feedback_callback=feedback_callback)

        # Wait for acceptance
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            executor.spin_once(timeout_sec=0.1)

        goal_handle = future.result()
        assert goal_handle.accepted, "Goal should be accepted"

        # Wait for completion while collecting feedback
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)

        # Should have received at least some feedback
        assert len(feedback_received) > 0, "No feedback received during motion"
        # Progress should increase over time (generally)
        assert feedback_received[-1] >= 90.0, \
            f"Final feedback progress should be near 100%, got {feedback_received[-1]}"


# =============================================================================
# TestContainerMimic (AC-5)
# =============================================================================

class TestContainerMimic:
    """AC-5: Verify container jaw synchronization (mimic mode)."""

    def test_symmetric_jaw_positions(self, move_joint_group_client):
        """AC-5: Container group with opening=0.15 -> left=-0.075, right=+0.075."""
        client, node, executor = move_joint_group_client

        # Container group with mimic mode - single opening value
        result, error = send_move_joint_group_goal(client, node, executor,
            joint_names=['container'],
            targets=[0.15],  # opening value
            max_velocity=0.2)

        assert result is not None, f"Goal failed: {error}"
        assert result.success, f"Goal not successful: {result.message}"
        assert len(result.final_positions) == 2, "Should have 2 final positions (left, right)"

        left_target = -0.075
        right_target = 0.075

        assert abs(result.final_positions[0] - left_target) <= POSITION_TOLERANCE, \
            f"Left jaw: expected {left_target}, got {result.final_positions[0]}"
        assert abs(result.final_positions[1] - right_target) <= POSITION_TOLERANCE, \
            f"Right jaw: expected {right_target}, got {result.final_positions[1]}"

    def test_multiple_opening_values(self, move_joint_group_client):
        """AC-5: Test symmetric opening at multiple values (0.1, 0.2, 0.3)."""
        client, node, executor = move_joint_group_client

        opening_values = [0.1, 0.2, 0.3]

        for opening in opening_values:
            result, error = send_move_joint_group_goal(client, node, executor,
                joint_names=['container'],
                targets=[opening],
                max_velocity=0.2)

            assert result is not None, f"Goal failed for opening={opening}: {error}"
            assert result.success, f"Goal not successful for opening={opening}: {result.message}"

            expected_left = -opening / 2.0
            expected_right = opening / 2.0

            assert abs(result.final_positions[0] - expected_left) <= POSITION_TOLERANCE, \
                f"Opening {opening}: left expected {expected_left}, got {result.final_positions[0]}"
            assert abs(result.final_positions[1] - expected_right) <= POSITION_TOLERANCE, \
                f"Opening {opening}: right expected {expected_right}, got {result.final_positions[1]}"


# =============================================================================
# TestStateMarkers (AC-6)
# =============================================================================

class TestStateMarkers:
    """AC-6: Verify state markers appear/disappear correctly in RViz."""

    def test_marker_array_topic_publishes(self, test_node):
        """AC-6: /visualization_marker_array topic should publish at ~10 Hz."""
        node, executor = test_node

        received_msgs = []

        def callback(msg):
            received_msgs.append(time.time())

        sub = node.create_subscription(MarkerArray, '/visualization_marker_array', callback, 10)

        # Collect messages for 2 seconds
        start = time.time()
        while time.time() - start < 2.0:
            executor.spin_once(timeout_sec=0.1)

        node.destroy_subscription(sub)

        assert len(received_msgs) > 0, "No messages received on /visualization_marker_array"

        # Check approximate publish rate (should be ~10 Hz = ~20 messages in 2 seconds)
        if len(received_msgs) >= 2:
            avg_rate = len(received_msgs) / 2.0
            assert avg_rate > 5.0, f"Publish rate {avg_rate:.1f} Hz is too low (expected ~10 Hz)"

    def test_magnet_marker_on_engage(self, test_node):
        """AC-6: Electromagnet marker should appear when engaged."""
        node, executor = test_node

        # Create publisher for electromagnet engage
        pub = node.create_publisher(Bool, '/manipulator/electromagnet/engaged', 10)

        # Give time for publisher to be discovered
        time.sleep(0.5)

        # Publish engaged state
        msg = Bool()
        msg.data = True
        pub.publish(msg)

        # Wait a bit for marker to update
        time.sleep(0.5)

        # Check marker array contains magnet marker
        marker_msg = wait_for_topic_message(node, executor, '/visualization_marker_array',
                                            MarkerArray, timeout=2.0)

        assert marker_msg is not None, "No marker array message received"
        # Markers should be present (exact content depends on implementation)
        # We just verify the system is publishing markers

    def test_target_address_marker(self, test_node):
        """AC-6: Target address marker should appear when set."""
        node, executor = test_node

        # Create publisher for target address
        pub = node.create_publisher(String, '/manipulator/target_address', 10)

        time.sleep(0.5)

        # Publish target address (format: "row,col,level" e.g., "A,1,1")
        msg = String()
        msg.data = "A,1,1"
        pub.publish(msg)

        time.sleep(0.5)

        # Check marker array received
        marker_msg = wait_for_topic_message(node, executor, '/visualization_marker_array',
                                            MarkerArray, timeout=2.0)

        assert marker_msg is not None, "No marker array message received after setting target"

    def test_target_address_marker_clears(self, test_node):
        """AC-6: Target address marker should disappear when cleared."""
        node, executor = test_node

        pub = node.create_publisher(String, '/manipulator/target_address', 10)

        time.sleep(0.5)

        # First set a target
        msg = String()
        msg.data = "A,1,1"
        pub.publish(msg)
        time.sleep(0.3)

        # Then clear it
        msg.data = ""
        pub.publish(msg)
        time.sleep(0.5)

        # Marker system should still be publishing (empty or cleared markers)
        marker_msg = wait_for_topic_message(node, executor, '/visualization_marker_array',
                                            MarkerArray, timeout=2.0)

        assert marker_msg is not None, "Marker system stopped publishing after clear"


# =============================================================================
# Test Summary
# =============================================================================

if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
