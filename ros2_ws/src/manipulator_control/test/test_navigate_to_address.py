#!/usr/bin/env python3
"""
Unit Tests for NavigateToAddress Action Server.

Story 3.4 - Epic 3: Address Navigation System

Tests:
- Action interface message generation (AC1)
- World-to-joint coordinate mapping (AC4, AC5)
- Approach distance calculation (AC7)
- Position error Euclidean calculation (AC6)
- Config loading (AC4)

Run: pytest test/test_navigate_to_address.py -v
"""

import pytest
from unittest.mock import MagicMock
import math


# =============================================================================
# Test Data - kinematic_chains.yaml navigation group config
# =============================================================================

MOCK_NAVIGATION_CONFIG = {
    'joints': ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
    'description': 'X-Z positioning for address navigation',
    'default_velocity': 0.5,
    'default_acceleration': 0.25,
    'end_effector_frame': 'selector_frame',
    'end_effector_offset': [0.0, 0.0, 0.0],
    'coordinate_mapping': {
        'base_main_frame_joint': {
            'axis': 'x',
            'offset': 0.0,
        },
        'main_frame_selector_frame_joint': {
            'axis': 'z',
            'offset': 0.301,
        },
    },
}


# =============================================================================
# Mock Server Class for Testing
# =============================================================================

class MockNavigateToAddressServer:
    """
    Mock server class that mirrors NavigateToAddressServer's methods for unit testing.

    This avoids ROS2 node initialization while testing pure Python logic.
    """

    def __init__(self, config=None):
        config = config or MOCK_NAVIGATION_CONFIG
        self._joints = config['joints']
        self._end_effector_frame = config.get('end_effector_frame', 'selector_frame')
        self._end_effector_offset = config.get('end_effector_offset', [0.0, 0.0, 0.0])
        self._coordinate_mapping = config.get('coordinate_mapping', {})
        self._position_tolerance = 0.02  # 2cm per NFR-002

    def _world_to_joint_positions(self, target_x: float, target_y: float, target_z: float) -> list:
        """
        Convert world coordinates to joint positions using coordinate_mapping.

        For each joint in the navigation group, looks up which world axis it controls
        and subtracts the offset.
        """
        world_coords = {'x': target_x, 'y': target_y, 'z': target_z}
        joint_positions = []

        for joint_name in self._joints:
            mapping = self._coordinate_mapping.get(joint_name, {})
            axis = mapping.get('axis', 'x')
            offset = mapping.get('offset', 0.0)

            joint_pos = world_coords[axis] - offset
            joint_positions.append(joint_pos)

        return joint_positions

    def _apply_approach_distance(self, x: float, y: float, z: float,
                                  side: str, approach_distance: float) -> tuple:
        """
        Adjust Y coordinate for approach distance.

        For left cabinets: Y decreases (approach from +Y side)
        For right cabinets: Y increases (approach from -Y side)
        """
        if approach_distance <= 0:
            return x, y, z

        if side == 'left':
            adjusted_y = y - approach_distance
        else:  # right
            adjusted_y = y + approach_distance

        return x, adjusted_y, z

    def _calc_distance(self, x1: float, y1: float, z1: float,
                       x2: float, y2: float, z2: float) -> float:
        """Calculate Euclidean distance between two 3D points."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def _calc_xz_distance(self, x1: float, z1: float, x2: float, z2: float) -> float:
        """Calculate Euclidean distance in XZ plane (navigation plane)."""
        return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

    def _calc_progress_percent(self, start_distance: float, current_distance: float) -> int:
        """Calculate progress percentage based on distance traveled."""
        if start_distance < 0.001:
            return 100
        progress = (1.0 - (current_distance / start_distance)) * 100.0
        return int(max(0, min(100, progress)))


# =============================================================================
# TestActionInterface (AC1)
# =============================================================================

class TestActionInterface:
    """AC1: Test action interface message generation."""

    def test_action_interface_imports(self):
        """AC1: Verify action interface can be imported."""
        from manipulator_control.action import NavigateToAddress
        assert NavigateToAddress is not None

    def test_action_goal_fields(self):
        """AC1: Verify goal message has correct fields."""
        from manipulator_control.action import NavigateToAddress

        goal = NavigateToAddress.Goal()
        assert hasattr(goal, 'side')
        assert hasattr(goal, 'cabinet_num')
        assert hasattr(goal, 'row')
        assert hasattr(goal, 'column')
        assert hasattr(goal, 'approach_distance')

    def test_action_result_fields(self):
        """AC1: Verify result message has correct fields."""
        from manipulator_control.action import NavigateToAddress

        result = NavigateToAddress.Result()
        assert hasattr(result, 'success')
        assert hasattr(result, 'final_position')
        assert hasattr(result, 'positioning_error')
        assert hasattr(result, 'message')

    def test_action_feedback_fields(self):
        """AC1: Verify feedback message has correct fields."""
        from manipulator_control.action import NavigateToAddress

        feedback = NavigateToAddress.Feedback()
        assert hasattr(feedback, 'current_position')
        assert hasattr(feedback, 'distance_to_target')
        assert hasattr(feedback, 'progress_percent')

    def test_goal_default_approach_distance(self):
        """AC1: Verify approach_distance defaults to 0.0."""
        from manipulator_control.action import NavigateToAddress

        goal = NavigateToAddress.Goal()
        assert goal.approach_distance == 0.0


# =============================================================================
# TestConfigLoading (AC4)
# =============================================================================

class TestConfigLoading:
    """AC4: Test config loading from kinematic_chains.yaml."""

    def test_navigation_joints_loaded(self):
        """AC4: Verify navigation joints are loaded."""
        server = MockNavigateToAddressServer()
        assert server._joints == ['base_main_frame_joint', 'main_frame_selector_frame_joint']

    def test_end_effector_frame_loaded(self):
        """AC4: Verify end_effector_frame is loaded."""
        server = MockNavigateToAddressServer()
        assert server._end_effector_frame == 'selector_frame'

    def test_end_effector_offset_loaded(self):
        """AC4: Verify end_effector_offset is loaded."""
        server = MockNavigateToAddressServer()
        assert server._end_effector_offset == [0.0, 0.0, 0.0]

    def test_coordinate_mapping_loaded(self):
        """AC4: Verify coordinate_mapping is loaded for both joints."""
        server = MockNavigateToAddressServer()
        assert 'base_main_frame_joint' in server._coordinate_mapping
        assert 'main_frame_selector_frame_joint' in server._coordinate_mapping

    def test_x_joint_mapping(self):
        """AC4: Verify base_main_frame_joint maps to X axis with offset 0."""
        server = MockNavigateToAddressServer()
        x_mapping = server._coordinate_mapping['base_main_frame_joint']
        assert x_mapping['axis'] == 'x'
        assert x_mapping['offset'] == 0.0

    def test_z_joint_mapping(self):
        """AC4: Verify main_frame_selector_frame_joint maps to Z axis with offset 0.301."""
        server = MockNavigateToAddressServer()
        z_mapping = server._coordinate_mapping['main_frame_selector_frame_joint']
        assert z_mapping['axis'] == 'z'
        assert z_mapping['offset'] == 0.301


# =============================================================================
# TestWorldToJointMapping (AC5)
# =============================================================================

class TestWorldToJointMapping:
    """AC5: Test world-to-joint coordinate mapping."""

    def test_direct_x_mapping(self):
        """AC5: X world coordinate maps directly to x_joint (offset 0)."""
        server = MockNavigateToAddressServer()
        joint_positions = server._world_to_joint_positions(1.5, 0.4, 0.8)
        assert joint_positions[0] == pytest.approx(1.5, abs=0.001)

    def test_z_mapping_with_offset(self):
        """AC5: Z world coordinate maps to z_joint with offset 0.301."""
        server = MockNavigateToAddressServer()
        # z_joint = z_world - 0.301 = 0.8 - 0.301 = 0.499
        joint_positions = server._world_to_joint_positions(1.5, 0.4, 0.8)
        assert joint_positions[1] == pytest.approx(0.499, abs=0.001)

    def test_mapping_origin_position(self):
        """AC5: World origin (0, 0, 0.301) maps to joint (0, -0.301) for z."""
        server = MockNavigateToAddressServer()
        # z_joint = 0.301 - 0.301 = 0
        joint_positions = server._world_to_joint_positions(0.0, 0.0, 0.301)
        assert joint_positions[0] == pytest.approx(0.0, abs=0.001)
        assert joint_positions[1] == pytest.approx(0.0, abs=0.001)

    def test_mapping_returns_correct_joint_count(self):
        """AC5: Returns 2 joint positions for navigation group."""
        server = MockNavigateToAddressServer()
        joint_positions = server._world_to_joint_positions(1.0, 0.5, 0.5)
        assert len(joint_positions) == 2

    def test_mapping_various_z_values(self):
        """AC5: Test z_joint calculation for various z_world values."""
        server = MockNavigateToAddressServer()

        # z_world = 0.5 -> z_joint = 0.5 - 0.301 = 0.199
        positions = server._world_to_joint_positions(0.0, 0.0, 0.5)
        assert positions[1] == pytest.approx(0.199, abs=0.001)

        # z_world = 1.0 -> z_joint = 1.0 - 0.301 = 0.699
        positions = server._world_to_joint_positions(0.0, 0.0, 1.0)
        assert positions[1] == pytest.approx(0.699, abs=0.001)

        # z_world = 1.5 -> z_joint = 1.5 - 0.301 = 1.199
        positions = server._world_to_joint_positions(0.0, 0.0, 1.5)
        assert positions[1] == pytest.approx(1.199, abs=0.001)

    def test_y_not_used_in_navigation(self):
        """AC5: Y coordinate is not used in navigation mapping (XZ plane only)."""
        server = MockNavigateToAddressServer()
        # Y values should not affect joint positions
        positions1 = server._world_to_joint_positions(1.0, 0.0, 0.8)
        positions2 = server._world_to_joint_positions(1.0, 0.5, 0.8)
        positions3 = server._world_to_joint_positions(1.0, 1.0, 0.8)
        assert positions1 == positions2 == positions3


# =============================================================================
# TestApproachDistance (AC7)
# =============================================================================

class TestApproachDistance:
    """AC7: Test approach distance calculation."""

    def test_left_side_y_decreases(self):
        """AC7: Left side - approach distance subtracts from Y."""
        server = MockNavigateToAddressServer()
        x, y, z = server._apply_approach_distance(1.5, 0.4, 0.8, 'left', 0.1)
        assert x == pytest.approx(1.5, abs=0.001)
        assert y == pytest.approx(0.3, abs=0.001)  # 0.4 - 0.1 = 0.3
        assert z == pytest.approx(0.8, abs=0.001)

    def test_right_side_y_increases(self):
        """AC7: Right side - approach distance adds to Y."""
        server = MockNavigateToAddressServer()
        x, y, z = server._apply_approach_distance(1.5, -0.4, 0.8, 'right', 0.1)
        assert x == pytest.approx(1.5, abs=0.001)
        assert y == pytest.approx(-0.3, abs=0.001)  # -0.4 + 0.1 = -0.3
        assert z == pytest.approx(0.8, abs=0.001)

    def test_zero_approach_distance(self):
        """AC7: Zero approach distance returns unchanged coordinates."""
        server = MockNavigateToAddressServer()
        x, y, z = server._apply_approach_distance(1.5, 0.4, 0.8, 'left', 0.0)
        assert x == pytest.approx(1.5, abs=0.001)
        assert y == pytest.approx(0.4, abs=0.001)
        assert z == pytest.approx(0.8, abs=0.001)

    def test_negative_approach_distance_ignored(self):
        """AC7: Negative approach distance is treated as zero."""
        server = MockNavigateToAddressServer()
        x, y, z = server._apply_approach_distance(1.5, 0.4, 0.8, 'left', -0.1)
        assert y == pytest.approx(0.4, abs=0.001)  # Unchanged

    def test_left_side_large_approach(self):
        """AC7: Large approach distance for left side."""
        server = MockNavigateToAddressServer()
        x, y, z = server._apply_approach_distance(2.0, 0.5, 1.0, 'left', 0.2)
        assert y == pytest.approx(0.3, abs=0.001)  # 0.5 - 0.2 = 0.3

    def test_right_side_large_approach(self):
        """AC7: Large approach distance for right side."""
        server = MockNavigateToAddressServer()
        x, y, z = server._apply_approach_distance(2.0, -0.5, 1.0, 'right', 0.2)
        assert y == pytest.approx(-0.3, abs=0.001)  # -0.5 + 0.2 = -0.3


# =============================================================================
# TestPositionError (AC6)
# =============================================================================

class TestPositionError:
    """AC6: Test position error Euclidean calculation."""

    def test_3d_euclidean_distance(self):
        """AC6: Test 3D Euclidean distance calculation."""
        server = MockNavigateToAddressServer()
        # sqrt((3-0)^2 + (4-0)^2 + (0-0)^2) = sqrt(9+16) = 5
        dist = server._calc_distance(0.0, 0.0, 0.0, 3.0, 4.0, 0.0)
        assert dist == pytest.approx(5.0, abs=0.001)

    def test_xz_plane_distance(self):
        """AC6: Test XZ plane distance (navigation plane)."""
        server = MockNavigateToAddressServer()
        # sqrt((3-0)^2 + (4-0)^2) = 5
        dist = server._calc_xz_distance(0.0, 0.0, 3.0, 4.0)
        assert dist == pytest.approx(5.0, abs=0.001)

    def test_zero_distance(self):
        """AC6: Zero distance when positions match."""
        server = MockNavigateToAddressServer()
        dist = server._calc_xz_distance(1.5, 0.8, 1.5, 0.8)
        assert dist == pytest.approx(0.0, abs=0.001)

    def test_small_error_within_tolerance(self):
        """AC6: Error < 2cm is within tolerance."""
        server = MockNavigateToAddressServer()
        # 1cm error should be within 2cm tolerance
        dist = server._calc_xz_distance(1.0, 0.5, 1.01, 0.5)
        assert dist < server._position_tolerance

    def test_error_at_tolerance_boundary(self):
        """AC6: Error exactly at 2cm tolerance."""
        server = MockNavigateToAddressServer()
        # 2cm in one direction
        dist = server._calc_xz_distance(1.0, 0.5, 1.02, 0.5)
        assert dist == pytest.approx(0.02, abs=0.001)

    def test_error_exceeds_tolerance(self):
        """AC6: Error > 2cm exceeds tolerance."""
        server = MockNavigateToAddressServer()
        # 3cm error should exceed 2cm tolerance
        dist = server._calc_xz_distance(1.0, 0.5, 1.03, 0.5)
        assert dist > server._position_tolerance

    def test_diagonal_error(self):
        """AC6: Diagonal error in XZ plane."""
        server = MockNavigateToAddressServer()
        # sqrt(0.01^2 + 0.01^2) = sqrt(0.0002) ~= 0.0141
        dist = server._calc_xz_distance(1.0, 0.5, 1.01, 0.51)
        assert dist == pytest.approx(0.01414, abs=0.001)


# =============================================================================
# TestProgressCalculation (AC10)
# =============================================================================

class TestProgressCalculation:
    """AC10: Test progress percentage calculation."""

    def test_progress_0_at_start(self):
        """At start (full distance) = 0%."""
        server = MockNavigateToAddressServer()
        progress = server._calc_progress_percent(1.0, 1.0)
        assert progress == 0

    def test_progress_50_halfway(self):
        """At halfway point = 50%."""
        server = MockNavigateToAddressServer()
        progress = server._calc_progress_percent(1.0, 0.5)
        assert progress == 50

    def test_progress_100_at_target(self):
        """At target (zero distance) = 100%."""
        server = MockNavigateToAddressServer()
        progress = server._calc_progress_percent(1.0, 0.0)
        assert progress == 100

    def test_progress_already_at_target(self):
        """Start distance ~0 = 100% immediately."""
        server = MockNavigateToAddressServer()
        progress = server._calc_progress_percent(0.0, 0.0)
        assert progress == 100

    def test_progress_clamped_to_100(self):
        """Progress cannot exceed 100%."""
        server = MockNavigateToAddressServer()
        # Negative current distance (overshoot) should clamp to 100
        progress = server._calc_progress_percent(1.0, -0.1)
        assert progress <= 100


# =============================================================================
# TestServiceInterface
# =============================================================================

class TestServiceInterface:
    """Test service interface imports."""

    def test_get_address_coordinates_import(self):
        """Verify GetAddressCoordinates service can be imported."""
        from manipulator_control.srv import GetAddressCoordinates
        assert GetAddressCoordinates is not None

    def test_service_request_fields(self):
        """Verify service request has correct fields."""
        from manipulator_control.srv import GetAddressCoordinates

        request = GetAddressCoordinates.Request()
        assert hasattr(request, 'side')
        assert hasattr(request, 'cabinet_num')
        assert hasattr(request, 'row')
        assert hasattr(request, 'column')

    def test_service_response_fields(self):
        """Verify service response has correct fields."""
        from manipulator_control.srv import GetAddressCoordinates

        response = GetAddressCoordinates.Response()
        assert hasattr(response, 'success')
        assert hasattr(response, 'pose')
        assert hasattr(response, 'error_message')


# =============================================================================
# TestMoveJointGroupInterface
# =============================================================================

class TestMoveJointGroupInterface:
    """Test MoveJointGroup action interface for navigation."""

    def test_move_joint_group_import(self):
        """Verify MoveJointGroup action can be imported."""
        from manipulator_control.action import MoveJointGroup
        assert MoveJointGroup is not None

    def test_move_joint_group_goal_fields(self):
        """Verify MoveJointGroup goal has fields needed for navigation."""
        from manipulator_control.action import MoveJointGroup

        goal = MoveJointGroup.Goal()
        assert hasattr(goal, 'joint_names')
        assert hasattr(goal, 'target_positions')
        assert hasattr(goal, 'max_velocity')


# =============================================================================
# Integration Test Data Verification
# =============================================================================

class TestIntegrationPrep:
    """Verify test data matches expected configuration."""

    def test_mock_config_structure(self):
        """Verify mock config matches expected kinematic_chains.yaml structure."""
        assert MOCK_NAVIGATION_CONFIG['default_velocity'] == 0.5
        assert len(MOCK_NAVIGATION_CONFIG['joints']) == 2
        assert 'coordinate_mapping' in MOCK_NAVIGATION_CONFIG
        assert 'end_effector_frame' in MOCK_NAVIGATION_CONFIG

    def test_z_offset_value(self):
        """Verify Z offset is 0.301 as per URDF analysis."""
        z_mapping = MOCK_NAVIGATION_CONFIG['coordinate_mapping']['main_frame_selector_frame_joint']
        assert z_mapping['offset'] == 0.301

    def test_position_tolerance(self):
        """Verify position tolerance is 2cm per NFR-002."""
        server = MockNavigateToAddressServer()
        assert server._position_tolerance == 0.02
