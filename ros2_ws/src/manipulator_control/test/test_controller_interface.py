#!/usr/bin/env python3
"""
Unit tests for ControllerInterface utility.

Tests joint limit loading, command validation, and position caching
without requiring a running ROS2 system (mocked).

Run: python3 -m pytest test/test_controller_interface.py -v
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add src directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


class TestControllerInterface:
    """Unit tests for ControllerInterface class."""

    @pytest.fixture
    def mock_node(self):
        """Create a mock ROS2 node with unique publishers."""
        node = Mock()
        node.get_logger.return_value = Mock()
        # Each create_publisher call returns a unique Mock
        node.create_publisher.side_effect = lambda *args, **kwargs: Mock()
        node.create_subscription.return_value = Mock()
        return node

    @pytest.fixture
    def mock_params_yaml(self, tmp_path):
        """Create a temporary manipulator_params.yaml for testing."""
        yaml_content = """
base_assembly:
  base_main_frame_joint:
    limits: {lower: 0.0, upper: 4.0, effort: 2000.0, velocity: 2.0}
    safety_controller: {soft_lower: 0.1, soft_upper: 3.9, k_velocity: 0}

selector_assembly:
  main_frame_selector_frame_joint:
    limits: {lower: -0.01, upper: 1.5, effort: 2000.0, velocity: 2.0}
    safety_controller: {soft_lower: 0.05, soft_upper: 1.45, k_velocity: 0}
  selector_frame_gripper_joint:
    limits: {lower: -0.4, upper: 0.4, effort: 2000.0, velocity: 1.0}
    safety_controller: {soft_lower: -0.39, soft_upper: 0.39, k_velocity: 0}

picker_assembly:
  picker_frame_picker_rail_joint:
    limits: {lower: -0.3, upper: 0.3, effort: 2000.0, velocity: 1.0}
    safety_controller: {soft_lower: -0.29, soft_upper: 0.29, k_velocity: 0}
"""
        config_dir = tmp_path / "config"
        config_dir.mkdir()
        params_file = config_dir / "manipulator_params.yaml"
        params_file.write_text(yaml_content)
        return str(tmp_path)

    @pytest.fixture
    def controller_interface(self, mock_node, mock_params_yaml):
        """Create ControllerInterface with mocked dependencies."""
        with patch('controller_interface.get_package_share_directory') as mock_get_pkg:
            mock_get_pkg.return_value = mock_params_yaml
            from controller_interface import ControllerInterface
            return ControllerInterface(mock_node)

    def test_loads_joint_limits_from_params(self, controller_interface):
        """AC-8: Joint limits loaded from manipulator_params.yaml."""
        assert 'base_main_frame_joint' in controller_interface.joint_limits
        assert 'main_frame_selector_frame_joint' in controller_interface.joint_limits
        assert 'selector_frame_gripper_joint' in controller_interface.joint_limits
        assert 'picker_frame_picker_rail_joint' in controller_interface.joint_limits

    def test_soft_limits_parsed_correctly(self, controller_interface):
        """AC-8: Soft limits are correctly parsed from safety_controller."""
        limits = controller_interface.joint_limits['base_main_frame_joint']
        assert limits['min'] == 0.1
        assert limits['max'] == 3.9

        limits = controller_interface.joint_limits['main_frame_selector_frame_joint']
        assert limits['min'] == 0.05
        assert limits['max'] == 1.45

    def test_creates_publishers_for_all_joints(self, controller_interface, mock_node):
        """AC-2: Publishers created for all joint controllers."""
        # Check publishers dict has entries
        assert len(controller_interface.publishers) == 4

        # Verify create_publisher was called for each joint
        calls = mock_node.create_publisher.call_args_list
        topic_names = [call[0][1] for call in calls]

        assert '/base_main_frame_joint_controller/commands' in topic_names
        assert '/main_frame_selector_frame_joint_controller/commands' in topic_names
        assert '/selector_frame_gripper_joint_controller/commands' in topic_names
        assert '/picker_frame_picker_rail_joint_controller/commands' in topic_names

    def test_command_joint_valid_position(self, controller_interface):
        """AC-3: command_joint publishes to correct topic with valid position."""
        result = controller_interface.command_joint('base_main_frame_joint', 2.0)
        assert result is True

        # Verify publish was called
        publisher = controller_interface.publishers['base_main_frame_joint']
        publisher.publish.assert_called_once()

    def test_command_joint_unknown_joint_returns_false(self, controller_interface):
        """AC-7: Unknown joint returns False with warning."""
        result = controller_interface.command_joint('fake_joint', 1.0)
        assert result is False

        # Verify warning logged
        controller_interface.logger.warning.assert_called()

    def test_command_joint_outside_soft_limits_returns_false(self, controller_interface):
        """AC-6, AC-7: Position outside soft limits returns False with warning."""
        # base_main_frame_joint soft limits: 0.1 to 3.9
        result = controller_interface.command_joint('base_main_frame_joint', 10.0)
        assert result is False

        result = controller_interface.command_joint('base_main_frame_joint', -1.0)
        assert result is False

        # Verify no publish called
        publisher = controller_interface.publishers['base_main_frame_joint']
        publisher.publish.assert_not_called()

    def test_command_joint_at_soft_limits(self, controller_interface):
        """AC-6: Position at soft limits is accepted."""
        # Test at lower soft limit
        result = controller_interface.command_joint('base_main_frame_joint', 0.1)
        assert result is True

        # Test at upper soft limit
        result = controller_interface.command_joint('base_main_frame_joint', 3.9)
        assert result is True

    def test_command_joint_group_valid(self, controller_interface):
        """AC-4: command_joint_group commands multiple joints."""
        result = controller_interface.command_joint_group(
            ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            [2.0, 0.5]
        )
        assert result is True

        # Both publishers should have publish called
        controller_interface.publishers['base_main_frame_joint'].publish.assert_called_once()
        controller_interface.publishers['main_frame_selector_frame_joint'].publish.assert_called_once()

    def test_command_joint_group_fails_on_invalid_joint(self, controller_interface):
        """AC-7: Group command fails if any joint invalid (no partial commands)."""
        result = controller_interface.command_joint_group(
            ['base_main_frame_joint', 'fake_joint'],
            [2.0, 1.0]
        )
        assert result is False

        # No publishers should have publish called (fail fast)
        controller_interface.publishers['base_main_frame_joint'].publish.assert_not_called()

    def test_command_joint_group_fails_on_invalid_position(self, controller_interface):
        """AC-6, AC-7: Group command fails if any position outside limits."""
        result = controller_interface.command_joint_group(
            ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            [2.0, 10.0]  # Second position outside limits
        )
        assert result is False

    def test_command_joint_group_mismatched_lengths(self, controller_interface):
        """AC-7: Mismatched joint/position lists return False."""
        result = controller_interface.command_joint_group(
            ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            [2.0]  # Only one position for two joints
        )
        assert result is False

    def test_get_joint_position_returns_cached_value(self, controller_interface):
        """AC-5: get_joint_position returns value from cache."""
        # Simulate joint_state callback updating cache
        controller_interface.joint_positions['base_main_frame_joint'] = 1.5

        result = controller_interface.get_joint_position('base_main_frame_joint')
        assert result == 1.5

    def test_get_joint_position_returns_none_if_unknown(self, controller_interface):
        """AC-5: get_joint_position returns None for unknown joint."""
        result = controller_interface.get_joint_position('fake_joint')
        assert result is None

    def test_get_joint_position_returns_none_before_callback(self, controller_interface):
        """AC-5: get_joint_position returns None before any joint_states received."""
        result = controller_interface.get_joint_position('base_main_frame_joint')
        assert result is None

    def test_get_joint_limits_returns_tuple(self, controller_interface):
        """get_joint_limits returns (min, max) tuple."""
        result = controller_interface.get_joint_limits('base_main_frame_joint')
        assert result == (0.1, 3.9)

    def test_get_joint_limits_returns_none_for_unknown(self, controller_interface):
        """get_joint_limits returns None for unknown joint."""
        result = controller_interface.get_joint_limits('fake_joint')
        assert result is None

    def test_get_all_joint_names(self, controller_interface):
        """get_all_joint_names returns list of known joints."""
        result = controller_interface.get_all_joint_names()
        assert 'base_main_frame_joint' in result
        assert 'main_frame_selector_frame_joint' in result
        assert len(result) == 4

    def test_joint_state_callback_updates_cache(self, controller_interface):
        """_joint_state_cb updates joint_positions cache."""
        # Create mock JointState message
        msg = Mock()
        msg.name = ['base_main_frame_joint', 'main_frame_selector_frame_joint']
        msg.position = [1.5, 0.75]

        controller_interface._joint_state_cb(msg)

        assert controller_interface.joint_positions['base_main_frame_joint'] == 1.5
        assert controller_interface.joint_positions['main_frame_selector_frame_joint'] == 0.75


class TestControllerInterfaceEdgeCases:
    """Edge case tests for ControllerInterface."""

    @pytest.fixture
    def mock_node(self):
        """Create a mock ROS2 node."""
        node = Mock()
        node.get_logger.return_value = Mock()
        node.create_publisher.return_value = Mock()
        node.create_subscription.return_value = Mock()
        return node

    def test_handles_missing_params_file(self, mock_node):
        """Handles missing manipulator_params.yaml gracefully."""
        with patch('controller_interface.get_package_share_directory') as mock_get_pkg:
            mock_get_pkg.return_value = '/nonexistent/path'
            from controller_interface import ControllerInterface
            ci = ControllerInterface(mock_node)

            # Should have empty limits and log error
            assert len(ci.joint_limits) == 0
            ci.logger.error.assert_called()

    def test_command_joint_with_float_conversion(self, mock_node):
        """Ensures position is converted to float for message."""
        with patch('controller_interface.get_package_share_directory') as mock_get_pkg:
            # Create minimal valid config
            import tempfile
            with tempfile.TemporaryDirectory() as tmpdir:
                config_dir = os.path.join(tmpdir, 'config')
                os.makedirs(config_dir)
                params_file = os.path.join(config_dir, 'manipulator_params.yaml')
                with open(params_file, 'w') as f:
                    f.write("""
test_assembly:
  test_joint:
    limits: {velocity: 1.0}
    safety_controller: {soft_lower: 0.0, soft_upper: 10.0, k_velocity: 0}
""")
                mock_get_pkg.return_value = tmpdir

                from controller_interface import ControllerInterface
                ci = ControllerInterface(mock_node)

                # Command with integer should work (converted to float)
                result = ci.command_joint('test_joint', 5)
                assert result is True


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
