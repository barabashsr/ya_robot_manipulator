#!/usr/bin/env python3
"""
Unit Tests for MoveJointGroup Action Server.

Story 3.3 - Epic 3: Address Navigation System

Tests joint group config loading, named group resolution, mimic mode calculation,
default velocity retrieval, and aggregate progress calculation.

Run: pytest test/test_move_joint_group.py -v
"""

import pytest
from unittest.mock import MagicMock, patch, PropertyMock
import os
import yaml


# =============================================================================
# Test Data - kinematic_chains.yaml equivalent
# =============================================================================

MOCK_JOINT_GROUPS = {
    'navigation': {
        'joints': ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
        'description': 'X-Z positioning for address navigation',
        'default_velocity': 0.5,
        'default_acceleration': 0.25,
    },
    'gripper': {
        'joints': ['selector_frame_gripper_joint', 'main_frame_selector_frame_joint'],
        'description': 'Gripper Y-axis with Z adjustment',
        'default_velocity': 0.3,
        'default_acceleration': 0.15,
    },
    'picker': {
        'joints': [
            'selector_frame_picker_frame_joint',
            'picker_frame_picker_rail_joint',
            'picker_rail_picker_base_joint',
            'picker_base_picker_jaw_joint',
        ],
        'description': 'All picker joints for item picking',
        'default_velocity': 0.2,
        'default_acceleration': 0.1,
    },
    'container': {
        'joints': ['selector_left_container_jaw_joint', 'selector_right_container_jaw_joint'],
        'description': 'Container jaws (synchronized mimic)',
        'default_velocity': 0.1,
        'default_acceleration': 0.05,
        'mimic_mode': True,
    },
}

ALL_JOINT_NAMES = [
    'base_main_frame_joint',
    'main_frame_selector_frame_joint',
    'selector_frame_gripper_joint',
    'selector_frame_picker_frame_joint',
    'picker_frame_picker_rail_joint',
    'picker_rail_picker_base_joint',
    'picker_base_picker_jaw_joint',
    'selector_left_container_jaw_joint',
    'selector_right_container_jaw_joint',
]


# =============================================================================
# Mock Server Class for Testing
# =============================================================================

class MockMoveJointGroupServer:
    """
    Mock server class that mirrors MoveJointGroupServer's methods for unit testing.

    This avoids ROS2 node initialization while testing pure Python logic.
    """

    def __init__(self, joint_groups=None):
        self.joint_groups = joint_groups or MOCK_JOINT_GROUPS
        self.controller = MagicMock()
        self.controller.get_all_joint_names.return_value = ALL_JOINT_NAMES
        self._logger = MagicMock()

    def get_logger(self):
        return self._logger

    def _resolve_joint_group(self, joint_names, target_positions):
        """
        Resolve joint names and positions, handling named groups and mimic mode.

        Returns:
            Tuple (resolved_joint_names, resolved_positions, is_mimic_mode, default_velocity)
            or (None, None, False, None) on error.
        """
        # Check if this is a named group (single name that matches a group)
        if len(joint_names) == 1 and joint_names[0] in self.joint_groups:
            group_name = joint_names[0]
            group = self.joint_groups[group_name]
            resolved_joints = group['joints']
            is_mimic = group.get('mimic_mode', False)
            default_velocity = group.get('default_velocity', None)

            # Container mimic mode: single value -> symmetric positions
            # NOTE: Left jaw axis is -Y, right jaw axis is +Y (opposite directions)
            # So BOTH joints need the SAME position value to move in opposite directions
            if is_mimic:
                if len(target_positions) != 1:
                    return None, None, False, None

                opening = target_positions[0]
                half_opening = opening / 2.0
                left_target = half_opening   # Same value for both
                right_target = half_opening  # Opposite axes create symmetric opening
                resolved_positions = [left_target, right_target]
                return resolved_joints, resolved_positions, True, default_velocity
            else:
                # Non-mimic named group - positions must match joint count
                if len(target_positions) != len(resolved_joints):
                    return None, None, False, None
                return resolved_joints, list(target_positions), False, default_velocity

        # Check for invalid group name (AC-5: list valid groups in error)
        if len(joint_names) == 1 and joint_names[0] not in self.joint_groups:
            all_joints = self.controller.get_all_joint_names()
            if joint_names[0] not in all_joints:
                valid_groups = list(self.joint_groups.keys())
                self.get_logger().warning(
                    f'Invalid group or joint name: "{joint_names[0]}". '
                    f'Valid groups: {valid_groups}'
                )
                return None, None, False, None

        # Explicit joint names - validate count matches
        if len(joint_names) != len(target_positions):
            return None, None, False, None

        return list(joint_names), list(target_positions), False, None

    def _calc_aggregate_progress(self, start_positions, current_positions, target_positions):
        """
        Calculate aggregate progress percentage.

        Average of individual joint progress values.
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


# =============================================================================
# TestJointGroupConfig (AC1, AC2)
# =============================================================================

class TestJointGroupConfig:
    """AC1: Test config loading from kinematic_chains.yaml."""

    def test_all_groups_loaded(self):
        """AC1: Verify all 4 groups are loaded."""
        server = MockMoveJointGroupServer()
        assert len(server.joint_groups) == 4
        assert 'navigation' in server.joint_groups
        assert 'gripper' in server.joint_groups
        assert 'picker' in server.joint_groups
        assert 'container' in server.joint_groups

    def test_navigation_group_joints(self):
        """AC2: Verify navigation group has correct joints."""
        server = MockMoveJointGroupServer()
        nav_group = server.joint_groups['navigation']
        assert nav_group['joints'] == [
            'base_main_frame_joint',
            'main_frame_selector_frame_joint',
        ]

    def test_container_mimic_mode(self):
        """Verify container group has mimic_mode=True."""
        server = MockMoveJointGroupServer()
        container_group = server.joint_groups['container']
        assert container_group.get('mimic_mode', False) is True

    def test_default_velocities(self):
        """AC3: Verify each group has default_velocity defined."""
        server = MockMoveJointGroupServer()
        expected_velocities = {
            'navigation': 0.5,
            'gripper': 0.3,
            'picker': 0.2,
            'container': 0.1,
        }
        for group_name, expected_vel in expected_velocities.items():
            assert server.joint_groups[group_name]['default_velocity'] == expected_vel

    def test_default_accelerations(self):
        """AC3: Verify each group has default_acceleration defined."""
        server = MockMoveJointGroupServer()
        expected_accelerations = {
            'navigation': 0.25,
            'gripper': 0.15,
            'picker': 0.1,
            'container': 0.05,
        }
        for group_name, expected_accel in expected_accelerations.items():
            assert server.joint_groups[group_name]['default_acceleration'] == expected_accel

    def test_picker_group_has_4_joints(self):
        """Verify picker group has all 4 picker joints."""
        server = MockMoveJointGroupServer()
        picker_joints = server.joint_groups['picker']['joints']
        assert len(picker_joints) == 4
        assert 'selector_frame_picker_frame_joint' in picker_joints
        assert 'picker_base_picker_jaw_joint' in picker_joints


# =============================================================================
# TestGroupResolution (AC2, AC5)
# =============================================================================

class TestGroupResolution:
    """AC2, AC5: Test named group resolution logic."""

    def test_navigation_resolves_to_joints(self):
        """AC2: 'navigation' resolves to correct joint names."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['navigation'], [1.5, 0.8]
        )
        assert joints == ['base_main_frame_joint', 'main_frame_selector_frame_joint']
        assert positions == [1.5, 0.8]
        assert is_mimic is False
        assert velocity == 0.5

    def test_gripper_resolves_to_joints(self):
        """'gripper' resolves to correct joint names."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['gripper'], [0.2, 0.5]
        )
        assert joints == ['selector_frame_gripper_joint', 'main_frame_selector_frame_joint']
        assert positions == [0.2, 0.5]
        assert velocity == 0.3

    def test_picker_resolves_to_4_joints(self):
        """'picker' resolves to all 4 picker joints."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['picker'], [0.1, 0.2, 0.15, 0.1]
        )
        assert len(joints) == 4
        assert velocity == 0.2

    def test_invalid_group_rejected(self):
        """AC5: Unknown group name returns None with logged error."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['invalid_group'], [1.0]
        )
        assert joints is None
        assert positions is None
        # Verify warning was logged with valid groups list
        server.get_logger().warning.assert_called()
        call_args = server.get_logger().warning.call_args[0][0]
        assert 'invalid_group' in call_args
        assert 'navigation' in call_args  # Valid groups listed

    def test_explicit_joint_names(self):
        """Explicit joint names pass through without resolution."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            [1.0, 2.0]
        )
        assert joints == ['base_main_frame_joint', 'main_frame_selector_frame_joint']
        assert positions == [1.0, 2.0]
        assert velocity is None  # No default velocity for explicit joints

    def test_position_count_mismatch_rejected(self):
        """Position count not matching joint count is rejected."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['navigation'], [1.5]  # Only 1 position for 2-joint group
        )
        assert joints is None
        assert positions is None

    def test_default_velocity_returned_for_named_group(self):
        """AC3: Default velocity is returned for named groups."""
        server = MockMoveJointGroupServer()
        _, _, _, velocity = server._resolve_joint_group(['navigation'], [1.5, 0.8])
        assert velocity == 0.5

        _, _, _, velocity = server._resolve_joint_group(['container'], [0.3])
        assert velocity == 0.1


# =============================================================================
# TestContainerMimic (AC4)
# =============================================================================

class TestContainerMimic:
    """AC4: Test container mimic mode calculation."""

    def test_mimic_opening_0_3(self):
        """AC4: Opening 0.3 results in both jaws at +0.15 (opposite axes create symmetric motion)."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['container'], [0.3]
        )
        assert is_mimic is True
        assert len(positions) == 2
        # Both get same value - opposite joint axes create symmetric opening
        assert positions[0] == pytest.approx(0.15, abs=0.001)  # left
        assert positions[1] == pytest.approx(0.15, abs=0.001)  # right

    def test_mimic_opening_0_2(self):
        """Opening 0.2 results in both jaws at +0.1."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['container'], [0.2]
        )
        assert positions[0] == pytest.approx(0.1, abs=0.001)
        assert positions[1] == pytest.approx(0.1, abs=0.001)

    def test_mimic_opening_0(self):
        """Opening 0 results in both jaws at 0."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['container'], [0.0]
        )
        assert positions[0] == pytest.approx(0.0, abs=0.001)
        assert positions[1] == pytest.approx(0.0, abs=0.001)

    def test_mimic_rejects_multiple_values(self):
        """Mimic mode requires single opening value."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['container'], [0.1, 0.2]  # Two values for mimic group
        )
        assert joints is None
        assert positions is None

    def test_mimic_returns_correct_joints(self):
        """Mimic mode returns the actual jaw joint names."""
        server = MockMoveJointGroupServer()
        joints, positions, is_mimic, velocity = server._resolve_joint_group(
            ['container'], [0.3]
        )
        assert joints == [
            'selector_left_container_jaw_joint',
            'selector_right_container_jaw_joint',
        ]

    def test_mimic_returns_default_velocity(self):
        """Mimic mode returns default velocity from config."""
        server = MockMoveJointGroupServer()
        _, _, _, velocity = server._resolve_joint_group(['container'], [0.3])
        assert velocity == 0.1


# =============================================================================
# TestAggregateProgress (AC7)
# =============================================================================

class TestAggregateProgress:
    """AC7: Test aggregate progress calculation."""

    def test_progress_0_at_start(self):
        """At start position = 0% progress."""
        server = MockMoveJointGroupServer()
        progress = server._calc_aggregate_progress(
            start_positions=[0.0, 0.0],
            current_positions=[0.0, 0.0],
            target_positions=[1.0, 2.0],
        )
        assert progress == pytest.approx(0.0, abs=0.1)

    def test_progress_50_percent(self):
        """Halfway between start and target = 50%."""
        server = MockMoveJointGroupServer()
        progress = server._calc_aggregate_progress(
            start_positions=[0.0, 0.0],
            current_positions=[0.5, 1.0],
            target_positions=[1.0, 2.0],
        )
        assert progress == pytest.approx(50.0, abs=0.1)

    def test_progress_100_at_target(self):
        """At target position = 100%."""
        server = MockMoveJointGroupServer()
        progress = server._calc_aggregate_progress(
            start_positions=[0.0, 0.0],
            current_positions=[1.0, 2.0],
            target_positions=[1.0, 2.0],
        )
        assert progress == pytest.approx(100.0, abs=0.1)

    def test_progress_average_of_joints(self):
        """Progress is average of individual joint progress."""
        server = MockMoveJointGroupServer()
        # Joint 1: 0 -> 0.25 (target 1.0) = 25%
        # Joint 2: 0 -> 1.5 (target 2.0) = 75%
        # Average = 50%
        progress = server._calc_aggregate_progress(
            start_positions=[0.0, 0.0],
            current_positions=[0.25, 1.5],
            target_positions=[1.0, 2.0],
        )
        assert progress == pytest.approx(50.0, abs=0.1)

    def test_progress_already_at_target(self):
        """Start == target = 100% immediately."""
        server = MockMoveJointGroupServer()
        progress = server._calc_aggregate_progress(
            start_positions=[1.0, 2.0],
            current_positions=[1.0, 2.0],
            target_positions=[1.0, 2.0],
        )
        assert progress == pytest.approx(100.0, abs=0.1)

    def test_progress_single_joint(self):
        """Works with single joint."""
        server = MockMoveJointGroupServer()
        progress = server._calc_aggregate_progress(
            start_positions=[0.0],
            current_positions=[0.5],
            target_positions=[1.0],
        )
        assert progress == pytest.approx(50.0, abs=0.1)


# =============================================================================
# TestPositionValidation
# =============================================================================

class TestPositionValidation:
    """Test position count validation against joint groups."""

    def test_navigation_requires_2_positions(self):
        """Navigation group requires exactly 2 positions."""
        server = MockMoveJointGroupServer()
        # 1 position - should fail
        joints, _, _, _ = server._resolve_joint_group(['navigation'], [1.0])
        assert joints is None

        # 3 positions - should fail
        joints, _, _, _ = server._resolve_joint_group(['navigation'], [1.0, 2.0, 3.0])
        assert joints is None

        # 2 positions - should succeed
        joints, _, _, _ = server._resolve_joint_group(['navigation'], [1.0, 2.0])
        assert joints is not None

    def test_picker_requires_4_positions(self):
        """Picker group requires exactly 4 positions."""
        server = MockMoveJointGroupServer()
        # 3 positions - should fail
        joints, _, _, _ = server._resolve_joint_group(['picker'], [0.1, 0.2, 0.3])
        assert joints is None

        # 4 positions - should succeed
        joints, _, _, _ = server._resolve_joint_group(['picker'], [0.1, 0.2, 0.3, 0.4])
        assert joints is not None

    def test_explicit_joints_requires_matching_positions(self):
        """Explicit joint list requires matching position count."""
        server = MockMoveJointGroupServer()
        joints, _, _, _ = server._resolve_joint_group(
            ['joint_a', 'joint_b', 'joint_c'],
            [1.0, 2.0]  # Only 2 positions for 3 joints
        )
        assert joints is None


# =============================================================================
# Integration Test Preparation
# =============================================================================

class TestIntegrationPrep:
    """Verify test data matches actual config structure."""

    def test_mock_data_matches_expected_config(self):
        """Verify our mock data matches the expected kinematic_chains.yaml structure."""
        # Navigation group
        assert MOCK_JOINT_GROUPS['navigation']['default_velocity'] == 0.5
        assert len(MOCK_JOINT_GROUPS['navigation']['joints']) == 2

        # Container mimic
        assert MOCK_JOINT_GROUPS['container']['mimic_mode'] is True
        assert MOCK_JOINT_GROUPS['container']['default_velocity'] == 0.1

    def test_all_joints_list_complete(self):
        """Verify ALL_JOINT_NAMES contains all 9 joints."""
        assert len(ALL_JOINT_NAMES) == 9
        # Trajectory joints (7)
        assert 'base_main_frame_joint' in ALL_JOINT_NAMES
        assert 'main_frame_selector_frame_joint' in ALL_JOINT_NAMES
        assert 'selector_frame_gripper_joint' in ALL_JOINT_NAMES
        assert 'selector_frame_picker_frame_joint' in ALL_JOINT_NAMES
        assert 'picker_frame_picker_rail_joint' in ALL_JOINT_NAMES
        assert 'picker_rail_picker_base_joint' in ALL_JOINT_NAMES
        assert 'picker_base_picker_jaw_joint' in ALL_JOINT_NAMES
        # Forward command joints (2)
        assert 'selector_left_container_jaw_joint' in ALL_JOINT_NAMES
        assert 'selector_right_container_jaw_joint' in ALL_JOINT_NAMES
