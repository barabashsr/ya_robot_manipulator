#!/usr/bin/env python3
"""Unit tests for YZTrajectoryGenerator class.

Story 4A.1: Implement YZ Trajectory Generator Utility
Tests: AC4, AC5, AC6

Validates:
- load_trajectory returns correct waypoint count
- Left side produces positive Y values
- Right side produces negative Y values (sign flip)
- Base position offsets applied correctly
- time_from_start uses waypoint_duration from config
"""
import tempfile
from pathlib import Path

import pytest
import yaml


# Path to package config directory
PACKAGE_DIR = Path(__file__).parent.parent
CONFIG_DIR = PACKAGE_DIR / 'config'
SRC_DIR = PACKAGE_DIR / 'src'

# Add src to path for imports
import sys

sys.path.insert(0, str(SRC_DIR))

from utils.yz_trajectory_generator import TransformedWaypoint, YZTrajectoryGenerator


class TestYZTrajectoryGenerator:
    """Unit tests for YZTrajectoryGenerator class."""

    @pytest.fixture
    def generator(self):
        """Create generator with actual config files."""
        return YZTrajectoryGenerator(
            waypoints_path=str(CONFIG_DIR / 'extraction_trajectories.yaml'),
            config_path=str(CONFIG_DIR / 'trajectory_config.yaml'),
        )

    @pytest.fixture
    def temp_config(self):
        """Create temporary config with known values for precise testing."""
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)

            # Create waypoints with known values
            waypoints = {
                'source_svg': 'test.svg',
                'config_used': 'test_config.yaml',
                'generated': '2025-01-01T00:00:00',
                'trajectories': {
                    'test_traj': [
                        {'y': 0.0, 'z': 0.0},
                        {'y': 0.1, 'z': 0.001},
                        {'y': 0.2, 'z': 0.002},
                        {'y': 0.3, 'z': 0.001},
                        {'y': 0.4, 'z': 0.0},
                    ]
                },
            }

            config = {
                'trajectories': {
                    'test': {
                        'svg_file': 'test.svg',
                        'mapping': {
                            'x_range': [0, 100],
                            'y_output': [0.0, 0.4],
                            'y_center': 50,
                            'z_scale': 0.001,
                        },
                        'sampling': {
                            'num_points': 5,
                            'waypoint_duration': 0.25,
                        },
                    }
                }
            }

            waypoints_path = tmpdir / 'waypoints.yaml'
            config_path = tmpdir / 'config.yaml'

            with open(waypoints_path, 'w') as f:
                yaml.dump(waypoints, f)
            with open(config_path, 'w') as f:
                yaml.dump(config, f)

            yield str(waypoints_path), str(config_path)

    def test_load_trajectory_returns_correct_count(self, generator):
        """AC4: load_trajectory returns correct number of waypoints."""
        waypoints = generator.load_trajectory(
            name='insertion', side='left', base_y=0.0, base_z=0.5
        )

        # Config specifies num_points: 20
        assert len(waypoints) == 20

    def test_left_side_produces_positive_y(self, generator):
        """AC4: Left side produces positive Y values."""
        waypoints = generator.load_trajectory(
            name='insertion', side='left', base_y=0.0, base_z=0.5
        )

        # All relative Y values in insertion trajectory are positive (0 to 0.4)
        # With base_y=0 and left side (sign=+1), final Y should be positive
        for wp in waypoints:
            assert wp.y >= 0.0, f"Left side Y should be >= 0, got {wp.y}"

        # Last waypoint should have largest Y (insertion goes deeper)
        assert waypoints[-1].y > waypoints[0].y

    def test_right_side_produces_negative_y(self, generator):
        """AC4: Right side produces negative Y values (sign flip)."""
        waypoints = generator.load_trajectory(
            name='insertion', side='right', base_y=0.0, base_z=0.5
        )

        # With base_y=0 and right side (sign=-1), Y values should be negative
        for wp in waypoints:
            assert wp.y <= 0.0, f"Right side Y should be <= 0, got {wp.y}"

        # Last waypoint should have most negative Y (insertion goes deeper)
        assert waypoints[-1].y < waypoints[0].y

    def test_base_position_offset_y(self, temp_config):
        """AC5: Base Y position offset applied correctly."""
        waypoints_path, config_path = temp_config
        generator = YZTrajectoryGenerator(waypoints_path, config_path)

        base_y = 1.5

        waypoints = generator.load_trajectory(
            name='test_traj', side='left', base_y=base_y, base_z=0.0, trajectory_name='test'
        )

        # First waypoint: relative Y=0.0, so final Y = base_y + 0.0 = 1.5
        assert waypoints[0].y == pytest.approx(base_y + 0.0, abs=0.001)

        # Last waypoint: relative Y=0.4, so final Y = base_y + 0.4 = 1.9
        assert waypoints[-1].y == pytest.approx(base_y + 0.4, abs=0.001)

    def test_base_position_offset_z(self, temp_config):
        """AC5: Base Z position offset applied correctly."""
        waypoints_path, config_path = temp_config
        generator = YZTrajectoryGenerator(waypoints_path, config_path)

        base_z = 0.75

        waypoints = generator.load_trajectory(
            name='test_traj', side='left', base_y=0.0, base_z=base_z, trajectory_name='test'
        )

        # First waypoint: relative Z=0.0, so final Z = base_z + 0.0 = 0.75
        assert waypoints[0].z == pytest.approx(base_z + 0.0, abs=0.001)

        # Middle waypoint (index 2): relative Z=0.002, so final Z = base_z + 0.002 = 0.752
        assert waypoints[2].z == pytest.approx(base_z + 0.002, abs=0.001)

    def test_time_from_start_uses_config_duration(self, temp_config):
        """AC6: time_from_start calculated from waypoint_duration in config."""
        waypoints_path, config_path = temp_config
        generator = YZTrajectoryGenerator(waypoints_path, config_path)

        waypoints = generator.load_trajectory(
            name='test_traj', side='left', base_y=0.0, base_z=0.0, trajectory_name='test'
        )

        # Config specifies waypoint_duration: 0.25
        # Timing starts at (i+1)*duration so first waypoint is at t=0.25, not t=0
        # This gives the robot time to reach the first point
        duration = 0.25

        for i, wp in enumerate(waypoints):
            expected_time = (i + 1) * duration
            assert wp.time_from_start == pytest.approx(
                expected_time, abs=0.001
            ), f"Waypoint {i}: expected time {expected_time}, got {wp.time_from_start}"

    def test_unknown_trajectory_raises_error(self, generator):
        """load_trajectory raises ValueError for unknown trajectory name."""
        with pytest.raises(ValueError) as excinfo:
            generator.load_trajectory(
                name='nonexistent', side='left', base_y=0.0, base_z=0.0
            )

        assert 'Unknown trajectory' in str(excinfo.value)
        assert 'nonexistent' in str(excinfo.value)

    def test_transformed_waypoint_dataclass(self):
        """TransformedWaypoint stores correct values."""
        wp = TransformedWaypoint(y=0.25, z=0.5, time_from_start=1.5)

        assert wp.y == 0.25
        assert wp.z == 0.5
        assert wp.time_from_start == 1.5


class TestBuildJointTrajectory:
    """Tests for build_joint_trajectory method."""

    @pytest.fixture
    def generator(self):
        """Create generator with actual config files."""
        return YZTrajectoryGenerator(
            waypoints_path=str(CONFIG_DIR / 'extraction_trajectories.yaml'),
            config_path=str(CONFIG_DIR / 'trajectory_config.yaml'),
        )

    def test_joint_names_correct(self, generator):
        """Trajectory has correct joint names."""
        waypoints = [
            TransformedWaypoint(y=0.1, z=0.5, time_from_start=0.0),
            TransformedWaypoint(y=0.2, z=0.6, time_from_start=0.5),
        ]

        traj = generator.build_joint_trajectory(waypoints)

        assert traj.joint_names == [
            'selector_frame_gripper_joint',
            'main_frame_selector_frame_joint',
        ]

    def test_positions_mapped_correctly(self, generator):
        """Trajectory points have correct position mapping."""
        waypoints = [
            TransformedWaypoint(y=0.1, z=0.5, time_from_start=0.0),
            TransformedWaypoint(y=0.2, z=0.6, time_from_start=0.5),
        ]

        traj = generator.build_joint_trajectory(waypoints)

        # positions = [y, z] (array type in ROS2)
        assert list(traj.points[0].positions) == [0.1, 0.5]
        assert list(traj.points[1].positions) == [0.2, 0.6]

    def test_timing_converted_correctly(self, generator):
        """time_from_start converted to Duration correctly."""
        waypoints = [
            TransformedWaypoint(y=0.1, z=0.5, time_from_start=0.0),
            TransformedWaypoint(y=0.2, z=0.6, time_from_start=1.5),
            TransformedWaypoint(y=0.3, z=0.7, time_from_start=2.75),
        ]

        traj = generator.build_joint_trajectory(waypoints)

        # Check first point (0.0 sec)
        assert traj.points[0].time_from_start.sec == 0
        assert traj.points[0].time_from_start.nanosec == 0

        # Check second point (1.5 sec)
        assert traj.points[1].time_from_start.sec == 1
        assert traj.points[1].time_from_start.nanosec == 500000000

        # Check third point (2.75 sec)
        assert traj.points[2].time_from_start.sec == 2
        assert traj.points[2].time_from_start.nanosec == 750000000


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
