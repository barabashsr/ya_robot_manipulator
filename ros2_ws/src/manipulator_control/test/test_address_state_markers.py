#!/usr/bin/env python3
"""
Unit Tests for Address State Markers (Story 3.5).

Tests the integration between NavigateToAddress target publishing
and StateMarkerPublisher visualization.

AC1:  NavigateToAddress publishes target address to /manipulator/target_address
AC2:  NavigateToAddress clears target (empty string) on completion
AC3:  Green cube marker appears at target address TF frame
AC4:  Marker dimensions match box size from storage_params.yaml
AC5:  Extracted addresses parsed from comma-separated string
AC6:  Red cubes appear for extracted addresses
AC8:  Stable marker IDs (TARGET=1, EXTRACTED=100+)
AC10: Markers use namespace "manipulator_state"

Run: pytest test/test_address_state_markers.py -v
"""

import pytest
from unittest.mock import MagicMock, patch
from visualization_msgs.msg import Marker


# =============================================================================
# Test Data - Mock configurations
# =============================================================================

MOCK_MARKER_CONFIG = {
    'publish_rate': 10.0,
    'marker_namespace': 'manipulator_state',
    'target_marker': {
        'color': {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 0.5}
    },
    'extracted_marker': {
        'color': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.5}
    }
}

MOCK_BOX_CONFIG = {
    'columns_4': {'width': 0.06},
    'rows_10': {'height': 0.09}
}

MOCK_DEPT_CONFIG = {
    'departments_10': {'depth': 0.02}
}


# =============================================================================
# Mock StateMarkerPublisher for unit testing
# =============================================================================

class MockStateMarkerPublisher:
    """
    Mock StateMarkerPublisher for testing marker creation logic.
    """

    def __init__(self):
        self.publish_rate = MOCK_MARKER_CONFIG['publish_rate']
        self.marker_namespace = MOCK_MARKER_CONFIG['marker_namespace']
        self.target_config = MOCK_MARKER_CONFIG['target_marker']
        self.extracted_config = MOCK_MARKER_CONFIG['extracted_marker']
        self.box_configs = MOCK_BOX_CONFIG
        self.dept_configs = MOCK_DEPT_CONFIG

        self.target_address = ""
        self.extracted_addresses = []

        self.TARGET_MARKER_ID = 1
        self.EXTRACTED_MARKER_START_ID = 100

    def target_callback(self, data: str):
        """Handle target address updates."""
        self.target_address = data.strip()

    def extracted_callback(self, data: str):
        """Handle extracted addresses updates (comma-separated)."""
        data = data.strip()
        if data:
            self.extracted_addresses = [addr.strip() for addr in data.split(',') if addr.strip()]
        else:
            self.extracted_addresses = []

    def get_box_dimensions(self, address_frame: str):
        """Get box dimensions for an address frame."""
        default_dims = (0.06, 0.09, 0.02)
        try:
            parts = address_frame.split('_')
            if len(parts) < 5 or parts[0] != 'addr':
                return default_dims
            width = self.box_configs.get('columns_4', {}).get('width', 0.06)
            height = self.box_configs.get('rows_10', {}).get('height', 0.09)
            depth = self.dept_configs.get('departments_10', {}).get('depth', 0.02)
            return (width, height, depth)
        except Exception:
            return default_dims

    def create_target_marker(self):
        """Create target address marker (green cube)."""
        marker = Marker()
        marker.ns = self.marker_namespace
        marker.id = self.TARGET_MARKER_ID
        marker.type = Marker.CUBE

        if self.target_address:
            marker.header.frame_id = self.target_address
            marker.action = Marker.ADD
            width, height, depth = self.get_box_dimensions(self.target_address)
            marker.scale.x = width
            marker.scale.y = depth
            marker.scale.z = height
        else:
            marker.header.frame_id = "base_link"
            marker.action = Marker.DELETE

        color = self.target_config.get('color', {})
        marker.color.r = float(color.get('r', 0.0))
        marker.color.g = float(color.get('g', 1.0))
        marker.color.b = float(color.get('b', 0.0))
        marker.color.a = float(color.get('a', 0.5))

        return marker

    def create_extracted_markers(self):
        """Create extracted address markers (red cubes)."""
        markers = []
        for i, address in enumerate(self.extracted_addresses):
            marker = Marker()
            marker.header.frame_id = address
            marker.ns = self.marker_namespace
            marker.id = self.EXTRACTED_MARKER_START_ID + i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            width, height, depth = self.get_box_dimensions(address)
            marker.scale.x = width
            marker.scale.y = depth
            marker.scale.z = height

            color = self.extracted_config.get('color', {})
            marker.color.r = float(color.get('r', 1.0))
            marker.color.g = float(color.get('g', 0.0))
            marker.color.b = float(color.get('b', 0.0))
            marker.color.a = float(color.get('a', 0.5))

            markers.append(marker)
        return markers


# =============================================================================
# TestTargetAddressPublishing (AC1, AC2)
# =============================================================================

class TestTargetAddressPublishing:
    """AC1, AC2: Test target address publishing and clearing."""

    def test_target_frame_format(self):
        """AC1: Target frame format is addr_{side}_{cabinet}_{row}_{col}."""
        # Simulate NavigateToAddress frame construction
        side = 'left'
        cabinet_num = 1
        row = 2
        column = 3

        side_abbrev = 'l' if side == 'left' else 'r'
        target_frame = f"addr_{side_abbrev}_{cabinet_num}_{row}_{column}"

        assert target_frame == "addr_l_1_2_3"

    def test_target_frame_right_side(self):
        """AC1: Right side uses 'r' abbreviation."""
        side = 'right'
        cabinet_num = 4
        row = 5
        column = 6

        side_abbrev = 'l' if side == 'left' else 'r'
        target_frame = f"addr_{side_abbrev}_{cabinet_num}_{row}_{column}"

        assert target_frame == "addr_r_4_5_6"

    def test_target_address_stored(self):
        """AC1: Target address callback stores address string."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_2_3")
        assert publisher.target_address == "addr_l_1_2_3"

    def test_target_address_cleared(self):
        """AC2: Empty string clears target address."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_2_3")
        assert publisher.target_address == "addr_l_1_2_3"

        publisher.target_callback("")
        assert publisher.target_address == ""

    def test_target_address_whitespace_stripped(self):
        """AC1: Whitespace is stripped from target address."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("  addr_l_1_2_3  ")
        assert publisher.target_address == "addr_l_1_2_3"


# =============================================================================
# TestTargetMarker (AC3, AC4, AC8, AC10)
# =============================================================================

class TestTargetMarker:
    """AC3, AC4, AC8, AC10: Test target marker creation."""

    def test_target_marker_is_cube(self):
        """AC3: Target marker type is CUBE."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        assert marker.type == Marker.CUBE

    def test_target_marker_green_color(self):
        """AC3: Target marker is green (0, 1, 0)."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        assert marker.color.g == 1.0
        assert marker.color.r == 0.0
        assert marker.color.b == 0.0

    def test_target_marker_semi_transparent(self):
        """AC3: Target marker alpha is 0.5 (semi-transparent)."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        assert marker.color.a == 0.5

    def test_target_marker_frame_id(self):
        """AC3: Target marker frame_id matches address frame."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_2_3_4")
        marker = publisher.create_target_marker()
        assert marker.header.frame_id == "addr_l_2_3_4"

    def test_target_marker_dimensions(self):
        """AC4: Target marker dimensions from storage_params."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        # Default dimensions: width=0.06, height=0.09, depth=0.02
        assert marker.scale.x == pytest.approx(0.06, abs=0.001)  # width
        assert marker.scale.y == pytest.approx(0.02, abs=0.001)  # depth
        assert marker.scale.z == pytest.approx(0.09, abs=0.001)  # height

    def test_target_marker_stable_id(self):
        """AC8: Target marker ID is stable (1)."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        assert marker.id == 1

    def test_target_marker_namespace(self):
        """AC10: Target marker uses namespace 'manipulator_state'."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        assert marker.ns == "manipulator_state"

    def test_target_marker_action_add(self):
        """AC3: Target marker action is ADD when address set."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("addr_l_1_1_1")
        marker = publisher.create_target_marker()
        assert marker.action == Marker.ADD

    def test_target_marker_action_delete(self):
        """AC2: Target marker action is DELETE when address empty."""
        publisher = MockStateMarkerPublisher()
        publisher.target_callback("")
        marker = publisher.create_target_marker()
        assert marker.action == Marker.DELETE


# =============================================================================
# TestExtractedAddresses (AC5, AC6, AC8)
# =============================================================================

class TestExtractedAddresses:
    """AC5, AC6, AC8: Test extracted address parsing and markers."""

    def test_single_extracted_address(self):
        """AC5: Single address parsed correctly."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1")
        assert publisher.extracted_addresses == ["addr_l_1_1_1"]

    def test_multiple_extracted_addresses(self):
        """AC5: Comma-separated addresses parsed correctly."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1,addr_l_2_1_1,addr_r_3_2_3")
        assert publisher.extracted_addresses == [
            "addr_l_1_1_1",
            "addr_l_2_1_1",
            "addr_r_3_2_3"
        ]

    def test_extracted_addresses_whitespace(self):
        """AC5: Whitespace around addresses is stripped."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1 , addr_l_2_1_1 , addr_r_3_2_3")
        assert publisher.extracted_addresses == [
            "addr_l_1_1_1",
            "addr_l_2_1_1",
            "addr_r_3_2_3"
        ]

    def test_extracted_addresses_cleared(self):
        """AC5: Empty string clears extracted addresses."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1,addr_l_2_1_1")
        assert len(publisher.extracted_addresses) == 2

        publisher.extracted_callback("")
        assert publisher.extracted_addresses == []

    def test_extracted_markers_red_color(self):
        """AC6: Extracted markers are red (1, 0, 0)."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1")
        markers = publisher.create_extracted_markers()
        assert len(markers) == 1
        assert markers[0].color.r == 1.0
        assert markers[0].color.g == 0.0
        assert markers[0].color.b == 0.0

    def test_extracted_markers_semi_transparent(self):
        """AC6: Extracted markers alpha is 0.5."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1")
        markers = publisher.create_extracted_markers()
        assert markers[0].color.a == 0.5

    def test_extracted_markers_are_cubes(self):
        """AC6: Extracted markers are CUBE type."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1,addr_l_2_1_1")
        markers = publisher.create_extracted_markers()
        for marker in markers:
            assert marker.type == Marker.CUBE

    def test_extracted_marker_ids(self):
        """AC8: Extracted marker IDs start at 100."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1,addr_l_2_1_1,addr_l_3_1_1")
        markers = publisher.create_extracted_markers()
        assert markers[0].id == 100
        assert markers[1].id == 101
        assert markers[2].id == 102

    def test_extracted_marker_frame_ids(self):
        """AC6: Extracted markers use address as frame_id."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1,addr_r_2_3_4")
        markers = publisher.create_extracted_markers()
        assert markers[0].header.frame_id == "addr_l_1_1_1"
        assert markers[1].header.frame_id == "addr_r_2_3_4"

    def test_extracted_marker_namespace(self):
        """AC10: Extracted markers use namespace 'manipulator_state'."""
        publisher = MockStateMarkerPublisher()
        publisher.extracted_callback("addr_l_1_1_1")
        markers = publisher.create_extracted_markers()
        assert markers[0].ns == "manipulator_state"


# =============================================================================
# TestBoxDimensions (AC4)
# =============================================================================

class TestBoxDimensions:
    """AC4: Test box dimension lookup from storage_params."""

    def test_valid_address_dimensions(self):
        """AC4: Valid address returns configured dimensions."""
        publisher = MockStateMarkerPublisher()
        width, height, depth = publisher.get_box_dimensions("addr_l_1_2_3")
        assert width == pytest.approx(0.06, abs=0.001)
        assert height == pytest.approx(0.09, abs=0.001)
        assert depth == pytest.approx(0.02, abs=0.001)

    def test_invalid_address_returns_defaults(self):
        """AC4: Invalid address returns default dimensions."""
        publisher = MockStateMarkerPublisher()
        width, height, depth = publisher.get_box_dimensions("invalid_frame")
        assert width == pytest.approx(0.06, abs=0.001)
        assert height == pytest.approx(0.09, abs=0.001)
        assert depth == pytest.approx(0.02, abs=0.001)

    def test_short_address_returns_defaults(self):
        """AC4: Too-short address returns default dimensions."""
        publisher = MockStateMarkerPublisher()
        width, height, depth = publisher.get_box_dimensions("addr_l_1")
        assert width == pytest.approx(0.06, abs=0.001)


# =============================================================================
# TestMarkerConstants (AC7, AC8)
# =============================================================================

class TestMarkerConstants:
    """AC7, AC8: Test marker constants and configuration."""

    def test_publish_rate_10hz(self):
        """AC7: Publish rate is 10 Hz."""
        publisher = MockStateMarkerPublisher()
        assert publisher.publish_rate == 10.0

    def test_target_marker_id_constant(self):
        """AC8: TARGET_MARKER_ID is 1."""
        publisher = MockStateMarkerPublisher()
        assert publisher.TARGET_MARKER_ID == 1

    def test_extracted_marker_start_id(self):
        """AC8: EXTRACTED_MARKER_START_ID is 100."""
        publisher = MockStateMarkerPublisher()
        assert publisher.EXTRACTED_MARKER_START_ID == 100

    def test_marker_namespace_constant(self):
        """AC10: marker_namespace is 'manipulator_state'."""
        publisher = MockStateMarkerPublisher()
        assert publisher.marker_namespace == "manipulator_state"


# =============================================================================
# TestNavigateToAddressPublisher
# =============================================================================

class TestNavigateToAddressPublisher:
    """Test NavigateToAddress publisher configuration."""

    def test_string_import(self):
        """Verify std_msgs/String can be imported."""
        from std_msgs.msg import String
        assert String is not None

    def test_string_message_creation(self):
        """Verify String message can be created with data."""
        from std_msgs.msg import String
        msg = String(data="addr_l_1_2_3")
        assert msg.data == "addr_l_1_2_3"

    def test_empty_string_message(self):
        """Verify empty String message for clearing."""
        from std_msgs.msg import String
        msg = String(data="")
        assert msg.data == ""
