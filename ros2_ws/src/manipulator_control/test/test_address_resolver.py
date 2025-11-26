#!/usr/bin/env python3
"""
Unit Tests for AddressResolver Utility.

Story 3.1 - Epic 3: Address Navigation System

Tests TF frame name construction, address validation, cabinet config loading,
and TF coordinate resolution with mock buffer.

Run: pytest test/test_address_resolver.py -v
"""

import pytest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from tf2_ros import LookupException


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
    node = Node('test_address_resolver_node')
    yield node
    node.destroy_node()


@pytest.fixture
def mock_tf_buffer():
    """Create a mock TF buffer for testing."""
    buffer = MagicMock()
    return buffer


@pytest.fixture
def address_resolver(test_node, mock_tf_buffer):
    """Create AddressResolver with mock TF buffer."""
    from manipulator_utils.address_resolver import AddressResolver
    return AddressResolver(test_node, tf_buffer=mock_tf_buffer)


def create_mock_transform(x: float, y: float, z: float) -> TransformStamped:
    """Create a mock TransformStamped for testing."""
    transform = TransformStamped()
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation.w = 1.0
    return transform


# =============================================================================
# TestFrameNameConstruction (AC1, AC2)
# =============================================================================

class TestFrameNameConstruction:
    """AC1, AC2: Test TF frame name construction with side abbreviations."""

    def test_left_side_abbreviation(self, address_resolver):
        """AC2: 'left' maps to 'l' in frame name."""
        frame = address_resolver._construct_frame_name('left', 1, 2, 3)
        assert frame == 'addr_l_1_2_3'

    def test_right_side_abbreviation(self, address_resolver):
        """AC2: 'right' maps to 'r' in frame name."""
        frame = address_resolver._construct_frame_name('right', 4, 10, 5)
        assert frame == 'addr_r_4_10_5'

    def test_frame_format_correct(self, address_resolver):
        """AC1: Frame name follows addr_{abbrev}_{cabinet}_{row}_{column} format."""
        # Test various combinations
        assert address_resolver._construct_frame_name('left', 1, 1, 1) == 'addr_l_1_1_1'
        assert address_resolver._construct_frame_name('right', 3, 14, 6) == 'addr_r_3_14_6'
        assert address_resolver._construct_frame_name('left', 4, 12, 5) == 'addr_l_4_12_5'

    def test_case_insensitive_side(self, address_resolver):
        """Side parameter should be case-insensitive."""
        frame1 = address_resolver._construct_frame_name('LEFT', 1, 1, 1)
        frame2 = address_resolver._construct_frame_name('Left', 1, 1, 1)
        frame3 = address_resolver._construct_frame_name('left', 1, 1, 1)
        assert frame1 == frame2 == frame3 == 'addr_l_1_1_1'


# =============================================================================
# TestAddressValidation (AC2, AC3, AC4, AC5)
# =============================================================================

class TestAddressValidation:
    """AC2-5: Test address validation against cabinet configurations."""

    def test_valid_address_left_cabinet_1(self, address_resolver):
        """Valid address for left cabinet 1 (4x10)."""
        valid, error = address_resolver.validate_address('left', 1, 5, 2)
        assert valid is True
        assert error == ''

    def test_valid_address_right_cabinet_3(self, address_resolver):
        """Valid address for right cabinet 3 (6x14)."""
        valid, error = address_resolver.validate_address('right', 3, 14, 6)
        assert valid is True
        assert error == ''

    def test_invalid_side(self, address_resolver):
        """AC2: Invalid side rejected with correct error message."""
        valid, error = address_resolver.validate_address('center', 1, 1, 1)
        assert valid is False
        assert "Side must be 'left' or 'right'" in error
        assert 'center' in error

    def test_invalid_cabinet_zero(self, address_resolver):
        """AC3: Cabinet 0 rejected."""
        valid, error = address_resolver.validate_address('left', 0, 1, 1)
        assert valid is False
        assert 'Cabinet 0 does not exist' in error

    def test_invalid_cabinet_five(self, address_resolver):
        """AC3: Cabinet 5 rejected."""
        valid, error = address_resolver.validate_address('right', 5, 1, 1)
        assert valid is False
        assert 'Cabinet 5 does not exist' in error

    def test_invalid_cabinet_negative(self, address_resolver):
        """AC3: Negative cabinet rejected."""
        valid, error = address_resolver.validate_address('left', -1, 1, 1)
        assert valid is False
        assert 'Cabinet -1 does not exist' in error

    def test_row_out_of_range_left_cabinet_3(self, address_resolver):
        """AC4: Left cabinet 3 has 6 rows, row 7 rejected."""
        valid, error = address_resolver.validate_address('left', 3, 7, 1)
        assert valid is False
        assert 'Row 7 exceeds cabinet 3 max of 6 rows' in error

    def test_row_out_of_range_right_cabinet_2(self, address_resolver):
        """AC4: Right cabinet 2 has 8 rows, row 9 rejected."""
        valid, error = address_resolver.validate_address('right', 2, 9, 1)
        assert valid is False
        assert 'Row 9 exceeds cabinet 2 max of 8 rows' in error

    def test_column_out_of_range_left_cabinet_1(self, address_resolver):
        """AC5: Left cabinet 1 has 4 columns, column 5 rejected."""
        valid, error = address_resolver.validate_address('left', 1, 1, 5)
        assert valid is False
        assert 'Column 5 exceeds cabinet 1 max of 4 columns' in error

    def test_column_out_of_range_right_cabinet_3(self, address_resolver):
        """AC5: Right cabinet 3 has 6 columns, column 7 rejected."""
        valid, error = address_resolver.validate_address('right', 3, 1, 7)
        assert valid is False
        assert 'Column 7 exceeds cabinet 3 max of 6 columns' in error

    def test_row_zero_rejected(self, address_resolver):
        """AC4: Row 0 rejected (1-indexed)."""
        valid, error = address_resolver.validate_address('left', 1, 0, 1)
        assert valid is False
        assert 'Row 0 exceeds' in error

    def test_column_zero_rejected(self, address_resolver):
        """AC5: Column 0 rejected (1-indexed)."""
        valid, error = address_resolver.validate_address('left', 1, 1, 0)
        assert valid is False
        assert 'Column 0 exceeds' in error


# =============================================================================
# TestCabinetConfig (AC7, AC8)
# =============================================================================

class TestCabinetConfig:
    """AC7, AC8: Test cabinet configuration loading and accessor."""

    def test_config_loaded_left_cabinet_1(self, address_resolver):
        """AC8: Left cabinet 1 config loaded correctly (4x10x10)."""
        config = address_resolver.get_cabinet_config('left', 1)
        assert config is not None
        assert config['columns'] == 4
        assert config['rows'] == 10
        assert config['departments'] == 10

    def test_config_loaded_left_cabinet_3(self, address_resolver):
        """AC8: Left cabinet 3 config loaded correctly (4x6x10)."""
        config = address_resolver.get_cabinet_config('left', 3)
        assert config is not None
        assert config['columns'] == 4
        assert config['rows'] == 6
        assert config['departments'] == 10

    def test_config_loaded_left_cabinet_4(self, address_resolver):
        """AC8: Left cabinet 4 config loaded correctly (5x12x14)."""
        config = address_resolver.get_cabinet_config('left', 4)
        assert config is not None
        assert config['columns'] == 5
        assert config['rows'] == 12
        assert config['departments'] == 14

    def test_config_loaded_right_cabinet_1(self, address_resolver):
        """AC8: Right cabinet 1 config loaded correctly (5x12x14)."""
        config = address_resolver.get_cabinet_config('right', 1)
        assert config is not None
        assert config['columns'] == 5
        assert config['rows'] == 12
        assert config['departments'] == 14

    def test_config_loaded_right_cabinet_3(self, address_resolver):
        """AC8: Right cabinet 3 config loaded correctly (6x14x16)."""
        config = address_resolver.get_cabinet_config('right', 3)
        assert config is not None
        assert config['columns'] == 6
        assert config['rows'] == 14
        assert config['departments'] == 16

    def test_config_loaded_right_cabinet_4(self, address_resolver):
        """AC8: Right cabinet 4 config loaded correctly (4x10x10)."""
        config = address_resolver.get_cabinet_config('right', 4)
        assert config is not None
        assert config['columns'] == 4
        assert config['rows'] == 10
        assert config['departments'] == 10

    def test_invalid_cabinet_returns_none(self, address_resolver):
        """AC7: get_cabinet_config returns None for invalid cabinet."""
        config = address_resolver.get_cabinet_config('left', 5)
        assert config is None

    def test_invalid_side_returns_none(self, address_resolver):
        """AC7: get_cabinet_config returns None for invalid side."""
        config = address_resolver.get_cabinet_config('center', 1)
        assert config is None

    def test_all_8_cabinets_loaded(self, address_resolver):
        """AC8: All 8 cabinet configs loaded (4 left + 4 right)."""
        count = 0
        for side in ['left', 'right']:
            for cabinet in range(1, 5):
                if address_resolver.get_cabinet_config(side, cabinet) is not None:
                    count += 1
        assert count == 8


# =============================================================================
# TestCoordinateResolution (AC1, AC6, AC9)
# =============================================================================

class TestCoordinateResolution:
    """AC1, AC6, AC9: Test TF coordinate resolution."""

    def test_successful_lookup(self, address_resolver, mock_tf_buffer):
        """AC1, AC9: Successful TF lookup returns coordinates in world frame."""
        mock_tf_buffer.lookup_transform.return_value = create_mock_transform(1.5, 0.4, 1.2)

        x, y, z, success, error = address_resolver.get_address_coordinates('left', 1, 1, 1)

        assert success is True
        assert error == ''
        assert x == 1.5
        assert y == 0.4
        assert z == 1.2

        # Verify lookup was called with 'world' frame (AC9)
        call_args = mock_tf_buffer.lookup_transform.call_args
        assert call_args[0][0] == 'world'  # target frame
        assert call_args[0][1] == 'addr_l_1_1_1'  # source frame

    def test_lookup_uses_world_frame(self, address_resolver, mock_tf_buffer):
        """AC9: All lookups use 'world' as target frame."""
        mock_tf_buffer.lookup_transform.return_value = create_mock_transform(0, 0, 0)

        address_resolver.get_address_coordinates('right', 2, 3, 4)

        call_args = mock_tf_buffer.lookup_transform.call_args
        assert call_args[0][0] == 'world'

    def test_validation_failure_returns_error(self, address_resolver, mock_tf_buffer):
        """AC1: Invalid address returns error without TF lookup."""
        x, y, z, success, error = address_resolver.get_address_coordinates('invalid', 1, 1, 1)

        assert success is False
        assert "Side must be 'left' or 'right'" in error
        assert x == 0.0
        assert y == 0.0
        assert z == 0.0
        mock_tf_buffer.lookup_transform.assert_not_called()

    def test_tf_lookup_exception_handled(self, address_resolver, mock_tf_buffer):
        """AC6: LookupException handled gracefully."""
        mock_tf_buffer.lookup_transform.side_effect = LookupException('Frame not found')

        x, y, z, success, error = address_resolver.get_address_coordinates('left', 1, 1, 1)

        assert success is False
        assert 'TF lookup failed for frame addr_l_1_1_1' in error
        assert x == 0.0
        assert y == 0.0
        assert z == 0.0

    def test_tf_timeout_handled(self, address_resolver, mock_tf_buffer):
        """AC6: TF timeout handled gracefully."""
        mock_tf_buffer.lookup_transform.side_effect = Exception('timeout')

        x, y, z, success, error = address_resolver.get_address_coordinates('left', 2, 5, 3)

        assert success is False
        assert 'TF lookup failed for frame addr_l_2_5_3' in error

    def test_constructs_correct_frame_name_for_lookup(self, address_resolver, mock_tf_buffer):
        """AC1, AC2: Correct frame name constructed for TF lookup."""
        mock_tf_buffer.lookup_transform.return_value = create_mock_transform(0, 0, 0)

        # Test left side
        address_resolver.get_address_coordinates('left', 4, 12, 5)
        assert mock_tf_buffer.lookup_transform.call_args[0][1] == 'addr_l_4_12_5'

        # Test right side
        address_resolver.get_address_coordinates('right', 3, 14, 6)
        assert mock_tf_buffer.lookup_transform.call_args[0][1] == 'addr_r_3_14_6'


# =============================================================================
# TestMockBufferInjection (AC10)
# =============================================================================

class TestMockBufferInjection:
    """AC10: Verify utility can be unit tested with mock TF buffer."""

    def test_mock_buffer_used(self, test_node, mock_tf_buffer):
        """AC10: Mock buffer is used instead of real TF system."""
        from manipulator_utils.address_resolver import AddressResolver
        resolver = AddressResolver(test_node, tf_buffer=mock_tf_buffer)

        mock_tf_buffer.lookup_transform.return_value = create_mock_transform(2.0, -0.5, 1.8)
        x, y, z, success, error = resolver.get_address_coordinates('right', 1, 1, 1)

        mock_tf_buffer.lookup_transform.assert_called_once()
        assert success is True
        assert x == 2.0
        assert y == -0.5
        assert z == 1.8

    def test_mock_buffer_returns_controlled_values(self, test_node, mock_tf_buffer):
        """AC10: Mock buffer returns controlled test values."""
        from manipulator_utils.address_resolver import AddressResolver
        resolver = AddressResolver(test_node, tf_buffer=mock_tf_buffer)

        # Set up mock to return specific values
        test_coords = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        mock_tf_buffer.lookup_transform.side_effect = [
            create_mock_transform(*test_coords[0]),
            create_mock_transform(*test_coords[1])
        ]

        # First call
        x1, y1, z1, _, _ = resolver.get_address_coordinates('left', 1, 1, 1)
        assert (x1, y1, z1) == test_coords[0]

        # Second call
        x2, y2, z2, _, _ = resolver.get_address_coordinates('left', 1, 1, 2)
        assert (x2, y2, z2) == test_coords[1]


# =============================================================================
# TestAPISignatures (AC7)
# =============================================================================

class TestAPISignatures:
    """AC7: Verify all three methods exist with correct signatures."""

    def test_get_address_coordinates_signature(self, address_resolver, mock_tf_buffer):
        """AC7: get_address_coordinates returns (x, y, z, success, error_msg)."""
        mock_tf_buffer.lookup_transform.return_value = create_mock_transform(1.0, 2.0, 3.0)
        result = address_resolver.get_address_coordinates('left', 1, 1, 1)

        assert isinstance(result, tuple)
        assert len(result) == 5
        assert isinstance(result[0], float)  # x
        assert isinstance(result[1], float)  # y
        assert isinstance(result[2], float)  # z
        assert isinstance(result[3], bool)   # success
        assert isinstance(result[4], str)    # error_msg

    def test_validate_address_signature(self, address_resolver):
        """AC7: validate_address returns (valid, error_msg)."""
        result = address_resolver.validate_address('left', 1, 1, 1)

        assert isinstance(result, tuple)
        assert len(result) == 2
        assert isinstance(result[0], bool)  # valid
        assert isinstance(result[1], str)   # error_msg

    def test_get_cabinet_config_signature(self, address_resolver):
        """AC7: get_cabinet_config returns dict or None."""
        result = address_resolver.get_cabinet_config('left', 1)

        assert result is None or isinstance(result, dict)
        if result is not None:
            assert 'columns' in result
            assert 'rows' in result
            assert 'departments' in result


# =============================================================================
# Test Summary
# =============================================================================

if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
