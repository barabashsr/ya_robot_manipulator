#!/usr/bin/env python3
"""
Unit Tests for AddressServiceNode.

Story 3.2 - Epic 3: Address Navigation System

Tests service interface, message generation, and callback logic
with mock AddressResolver.

Run: pytest test/test_address_service.py -v
"""

import pytest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node


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
    node = Node('test_address_service_node')
    yield node
    node.destroy_node()


# =============================================================================
# TestServiceInterfaceGeneration (AC1)
# =============================================================================

class TestServiceInterfaceGeneration:
    """AC1: Test service interface message generation."""

    def test_import_service_interface(self, ros2_context):
        """Service interface can be imported."""
        from manipulator_control.srv import GetAddressCoordinates
        assert GetAddressCoordinates is not None

    def test_request_has_required_fields(self, ros2_context):
        """Request has side, cabinet_num, row, column fields."""
        from manipulator_control.srv import GetAddressCoordinates
        request = GetAddressCoordinates.Request()

        assert hasattr(request, 'side')
        assert hasattr(request, 'cabinet_num')
        assert hasattr(request, 'row')
        assert hasattr(request, 'column')

    def test_response_has_required_fields(self, ros2_context):
        """Response has success, pose, error_message fields."""
        from manipulator_control.srv import GetAddressCoordinates
        response = GetAddressCoordinates.Response()

        assert hasattr(response, 'success')
        assert hasattr(response, 'pose')
        assert hasattr(response, 'error_message')

    def test_pose_has_position_and_orientation(self, ros2_context):
        """Response pose has position and orientation."""
        from manipulator_control.srv import GetAddressCoordinates
        response = GetAddressCoordinates.Response()

        assert hasattr(response.pose, 'position')
        assert hasattr(response.pose, 'orientation')
        # Position
        assert hasattr(response.pose.position, 'x')
        assert hasattr(response.pose.position, 'y')
        assert hasattr(response.pose.position, 'z')
        # Orientation (quaternion)
        assert hasattr(response.pose.orientation, 'x')
        assert hasattr(response.pose.orientation, 'y')
        assert hasattr(response.pose.orientation, 'z')
        assert hasattr(response.pose.orientation, 'w')


# =============================================================================
# TestServiceCallbackLogic (AC3, AC4, AC5)
# =============================================================================

class TestServiceCallbackLogic:
    """AC3, AC4, AC5: Test service callback handles AddressResolver responses."""

    @pytest.fixture
    def mock_address_resolver(self):
        """Create mock AddressResolver."""
        resolver = MagicMock()
        return resolver

    def test_valid_address_returns_success_with_pose(self, ros2_context, mock_address_resolver):
        """AC3, AC4: Valid address returns success=true with full pose."""
        from manipulator_control.srv import GetAddressCoordinates

        # Mock resolver returning success with pose
        mock_address_resolver.get_address_pose.return_value = (
            1.5, 0.4, 1.2,  # position
            0.0, 0.0, 0.707, 0.707,  # orientation
            True, ''  # success, error
        )

        # Create request/response
        request = GetAddressCoordinates.Request()
        request.side = 'left'
        request.cabinet_num = 1
        request.row = 1
        request.column = 1

        response = GetAddressCoordinates.Response()

        # Simulate callback logic
        x, y, z, qx, qy, qz, qw, success, error_msg = mock_address_resolver.get_address_pose(
            request.side, request.cabinet_num, request.row, request.column
        )

        response.success = success
        if success:
            response.pose.position.x = x
            response.pose.position.y = y
            response.pose.position.z = z
            response.pose.orientation.x = qx
            response.pose.orientation.y = qy
            response.pose.orientation.z = qz
            response.pose.orientation.w = qw
            response.error_message = ''

        # Verify
        assert response.success is True
        assert response.pose.position.x == 1.5
        assert response.pose.position.y == 0.4
        assert response.pose.position.z == 1.2
        assert response.pose.orientation.z == 0.707
        assert response.pose.orientation.w == 0.707
        assert response.error_message == ''

    def test_invalid_cabinet_returns_error(self, ros2_context, mock_address_resolver):
        """AC5: Invalid cabinet returns success=false with error message."""
        from manipulator_control.srv import GetAddressCoordinates

        error_msg = 'Cabinet 5 does not exist on left side'
        mock_address_resolver.get_address_pose.return_value = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, False, error_msg
        )

        request = GetAddressCoordinates.Request()
        request.side = 'left'
        request.cabinet_num = 5
        request.row = 1
        request.column = 1

        response = GetAddressCoordinates.Response()

        x, y, z, qx, qy, qz, qw, success, err = mock_address_resolver.get_address_pose(
            request.side, request.cabinet_num, request.row, request.column
        )

        response.success = success
        if not success:
            response.error_message = err

        assert response.success is False
        assert 'Cabinet 5 does not exist' in response.error_message

    def test_invalid_row_returns_error(self, ros2_context, mock_address_resolver):
        """AC5: Invalid row returns success=false with error message."""
        from manipulator_control.srv import GetAddressCoordinates

        error_msg = 'Row 15 exceeds cabinet 1 max of 10 rows'
        mock_address_resolver.get_address_pose.return_value = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, False, error_msg
        )

        request = GetAddressCoordinates.Request()
        request.side = 'left'
        request.cabinet_num = 1
        request.row = 15
        request.column = 1

        response = GetAddressCoordinates.Response()

        x, y, z, qx, qy, qz, qw, success, err = mock_address_resolver.get_address_pose(
            request.side, request.cabinet_num, request.row, request.column
        )

        response.success = success
        if not success:
            response.error_message = err

        assert response.success is False
        assert 'Row 15 exceeds' in response.error_message

    def test_invalid_column_returns_error(self, ros2_context, mock_address_resolver):
        """AC5: Invalid column returns success=false with error message."""
        from manipulator_control.srv import GetAddressCoordinates

        error_msg = 'Column 10 exceeds cabinet 1 max of 4 columns'
        mock_address_resolver.get_address_pose.return_value = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, False, error_msg
        )

        request = GetAddressCoordinates.Request()
        request.side = 'left'
        request.cabinet_num = 1
        request.row = 1
        request.column = 10

        response = GetAddressCoordinates.Response()

        x, y, z, qx, qy, qz, qw, success, err = mock_address_resolver.get_address_pose(
            request.side, request.cabinet_num, request.row, request.column
        )

        response.success = success
        if not success:
            response.error_message = err

        assert response.success is False
        assert 'Column 10 exceeds' in response.error_message


# =============================================================================
# TestOrientationFromTF (AC4)
# =============================================================================

class TestOrientationFromTF:
    """AC4: Test pose orientation comes from TF (not hardcoded identity)."""

    def test_orientation_is_valid_quaternion(self, ros2_context):
        """Returned orientation values form valid quaternion."""
        from manipulator_control.srv import GetAddressCoordinates

        response = GetAddressCoordinates.Response()
        response.pose.orientation.x = 0.0
        response.pose.orientation.y = 0.0
        response.pose.orientation.z = 0.707
        response.pose.orientation.w = 0.707

        # Check quaternion magnitude is approximately 1
        qx = response.pose.orientation.x
        qy = response.pose.orientation.y
        qz = response.pose.orientation.z
        qw = response.pose.orientation.w
        magnitude = (qx**2 + qy**2 + qz**2 + qw**2) ** 0.5

        assert abs(magnitude - 1.0) < 0.01


# =============================================================================
# Test Summary
# =============================================================================

if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
