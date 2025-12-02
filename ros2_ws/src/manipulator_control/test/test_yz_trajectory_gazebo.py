#!/usr/bin/env python3
"""Gazebo Integration Tests for YZ Trajectory Generator.

Story 4A.1: Implement YZ Trajectory Generator Utility
Tests: AC7, AC9

Validates:
- Trajectory execution via JointTrajectoryController produces smooth motion
- Trajectories work for 10 different base positions (simulating addresses)

Prerequisites:
    1. Launch simulation: ros2 launch manipulator_control manipulator_simulation.launch.py
    2. Wait ~30 seconds for controllers and action servers
    3. Verify controllers ready: ros2 control list_controllers

How to Run:
    cd ros2_ws
    python3 -m pytest src/manipulator_control/test/test_yz_trajectory_gazebo.py -v -s

Expected Duration: ~5-10 minutes (includes trajectory execution waits)
"""
import asyncio
import math
from pathlib import Path

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory

# Add src to path for imports
import sys

PACKAGE_DIR = Path(__file__).parent.parent
SRC_DIR = PACKAGE_DIR / 'src'
CONFIG_DIR = PACKAGE_DIR / 'config'
sys.path.insert(0, str(SRC_DIR))

from utils.yz_trajectory_generator import YZTrajectoryGenerator


# Test configuration
ACTION_SERVER_TIMEOUT = 10.0
TRAJECTORY_TIMEOUT = 30.0

# 10 test base positions simulating different cabinet addresses
# Format: (base_y, base_z, side, description)
TEST_BASE_POSITIONS = [
    (0.0, 0.3, 'left', 'Left cabinet, low row'),
    (0.0, 0.5, 'left', 'Left cabinet, middle row'),
    (0.0, 0.7, 'left', 'Left cabinet, high row'),
    (0.0, 0.9, 'left', 'Left cabinet, top row'),
    (0.0, 1.1, 'left', 'Left cabinet, highest'),
    (0.0, 0.3, 'right', 'Right cabinet, low row'),
    (0.0, 0.5, 'right', 'Right cabinet, middle row'),
    (0.0, 0.7, 'right', 'Right cabinet, high row'),
    (0.0, 0.9, 'right', 'Right cabinet, top row'),
    (0.0, 1.1, 'right', 'Right cabinet, highest'),
]


@pytest.fixture(scope='module')
def ros2_context():
    """Initialize ROS2 context for all tests in module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def test_node(ros2_context):
    """Create a shared test node for the module."""
    node = Node('test_yz_trajectory_gazebo')
    yield node
    node.destroy_node()


@pytest.fixture(scope='module')
def y_action_client(test_node):
    """Create action client for Y-axis (gripper) trajectory controller."""
    client = ActionClient(
        test_node,
        FollowJointTrajectory,
        '/selector_frame_gripper_joint_controller/follow_joint_trajectory',
    )

    if not client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT):
        pytest.skip(
            'Y-axis trajectory action server not available. '
            'Launch simulation first: ros2 launch manipulator_control manipulator_simulation.launch.py'
        )

    return client


@pytest.fixture(scope='module')
def z_action_client(test_node):
    """Create action client for Z-axis (selector frame) trajectory controller."""
    client = ActionClient(
        test_node,
        FollowJointTrajectory,
        '/main_frame_selector_frame_joint_controller/follow_joint_trajectory',
    )

    if not client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT):
        pytest.skip(
            'Z-axis trajectory action server not available. '
            'Launch simulation first: ros2 launch manipulator_control manipulator_simulation.launch.py'
        )

    return client


@pytest.fixture(scope='module')
def trajectory_generator():
    """Create YZTrajectoryGenerator instance."""
    return YZTrajectoryGenerator(
        waypoints_path=str(CONFIG_DIR / 'extraction_trajectories.yaml'),
        config_path=str(CONFIG_DIR / 'trajectory_config.yaml'),
    )


class TestGazeboTrajectoryExecution:
    """Gazebo integration tests for trajectory execution."""

    @pytest.mark.parametrize(
        'base_y,base_z,side,description',
        TEST_BASE_POSITIONS,
        ids=[f'{p[2]}-z{p[1]}' for p in TEST_BASE_POSITIONS],
    )
    def test_insertion_trajectory(
        self,
        test_node,
        y_action_client,
        z_action_client,
        trajectory_generator,
        base_y,
        base_z,
        side,
        description,
    ):
        """AC9: Execute insertion trajectory at various base positions."""
        # Load trajectory
        waypoints = trajectory_generator.load_trajectory(
            name='insertion',
            side=side,
            base_y=base_y,
            base_z=base_z,
        )

        # Execute synchronously using asyncio
        async def execute():
            return await trajectory_generator.execute_trajectory(
                waypoints, y_action_client, z_action_client, timeout_sec=TRAJECTORY_TIMEOUT
            )

        loop = asyncio.new_event_loop()
        try:
            success = loop.run_until_complete(execute())
        finally:
            loop.close()

        assert success, f"Insertion trajectory failed for {description}"

    def test_extraction_trajectory_left(
        self, test_node, y_action_client, z_action_client, trajectory_generator
    ):
        """AC9: Execute extraction trajectory for left side."""
        waypoints = trajectory_generator.load_trajectory(
            name='extraction',
            side='left',
            base_y=0.0,
            base_z=0.5,
        )

        async def execute():
            return await trajectory_generator.execute_trajectory(
                waypoints, y_action_client, z_action_client, timeout_sec=TRAJECTORY_TIMEOUT
            )

        loop = asyncio.new_event_loop()
        try:
            success = loop.run_until_complete(execute())
        finally:
            loop.close()

        assert success, "Extraction trajectory failed for left side"

    def test_extraction_trajectory_right(
        self, test_node, y_action_client, z_action_client, trajectory_generator
    ):
        """AC9: Execute extraction trajectory for right side."""
        waypoints = trajectory_generator.load_trajectory(
            name='extraction',
            side='right',
            base_y=0.0,
            base_z=0.5,
        )

        async def execute():
            return await trajectory_generator.execute_trajectory(
                waypoints, y_action_client, z_action_client, timeout_sec=TRAJECTORY_TIMEOUT
            )

        loop = asyncio.new_event_loop()
        try:
            success = loop.run_until_complete(execute())
        finally:
            loop.close()

        assert success, "Extraction trajectory failed for right side"


class TestTrajectorySmoothnessValidation:
    """Validate trajectory produces smooth motion (no jerks)."""

    def test_waypoint_spacing_reasonable(self, trajectory_generator):
        """Verify waypoint Y spacing is reasonable for smooth interpolation."""
        waypoints = trajectory_generator.load_trajectory(
            name='insertion',
            side='left',
            base_y=0.0,
            base_z=0.5,
        )

        # Check that adjacent waypoints don't jump too far
        max_y_step = 0.0
        max_z_step = 0.0

        for i in range(1, len(waypoints)):
            y_step = abs(waypoints[i].y - waypoints[i - 1].y)
            z_step = abs(waypoints[i].z - waypoints[i - 1].z)
            max_y_step = max(max_y_step, y_step)
            max_z_step = max(max_z_step, z_step)

        # With 20 points over 0.4m, average step ~0.02m
        # Allow up to 0.05m per step for smooth motion
        assert max_y_step < 0.05, f"Y step too large: {max_y_step}m"
        assert max_z_step < 0.01, f"Z step too large: {max_z_step}m"

    def test_trajectory_monotonic_y_insertion(self, trajectory_generator):
        """Insertion trajectory Y should monotonically increase."""
        waypoints = trajectory_generator.load_trajectory(
            name='insertion',
            side='left',
            base_y=0.0,
            base_z=0.5,
        )

        for i in range(1, len(waypoints)):
            assert (
                waypoints[i].y >= waypoints[i - 1].y
            ), f"Y not monotonic at index {i}: {waypoints[i-1].y} -> {waypoints[i].y}"

    def test_trajectory_monotonic_y_extraction(self, trajectory_generator):
        """Extraction trajectory Y should monotonically decrease."""
        waypoints = trajectory_generator.load_trajectory(
            name='extraction',
            side='left',
            base_y=0.0,
            base_z=0.5,
        )

        for i in range(1, len(waypoints)):
            assert (
                waypoints[i].y <= waypoints[i - 1].y
            ), f"Y not monotonic at index {i}: {waypoints[i-1].y} -> {waypoints[i].y}"


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
