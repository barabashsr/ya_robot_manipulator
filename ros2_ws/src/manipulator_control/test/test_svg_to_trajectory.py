#!/usr/bin/env python3
"""Unit tests for SVG-to-trajectory converter.

Story 4A.1: Implement YZ Trajectory Generator Utility
Tests: AC2, AC3

Validates:
- Converter reads scaling from config (not hardcoded)
- SVG with multiple paths produces multiple trajectories
- Y/Z scaling matches config values
- Output YAML includes config_used reference
"""
import os
import tempfile
from pathlib import Path

import pytest
import yaml


# Path to package config directory
PACKAGE_DIR = Path(__file__).parent.parent
CONFIG_DIR = PACKAGE_DIR / 'config'
SCRIPTS_DIR = PACKAGE_DIR / 'scripts'


class TestSvgToTrajectoryConverter:
    """Tests for svg_to_trajectory.py converter script."""

    @pytest.fixture
    def temp_output_dir(self):
        """Create a temporary directory for test outputs."""
        with tempfile.TemporaryDirectory() as tmpdir:
            yield Path(tmpdir)

    @pytest.fixture
    def test_config(self, temp_output_dir):
        """Create a test config with custom scaling values."""
        config = {
            'trajectories': {
                'test_trajectory': {
                    'svg_file': 'trajectories/extract_left.svg',
                    'mapping': {
                        'x_range': [0, 200],  # Different from default
                        'y_output': [0.0, 0.8],  # Different from default
                        'y_center': 100,  # Different from default
                        'z_scale': 0.002,  # Different from default
                    },
                    'sampling': {
                        'num_points': 10,  # Fewer points for testing
                        'waypoint_duration': 0.25,
                    },
                }
            }
        }
        config_path = temp_output_dir / 'test_config.yaml'

        # Copy SVG to temp dir structure
        svg_dir = temp_output_dir / 'trajectories'
        svg_dir.mkdir(parents=True, exist_ok=True)

        # Read actual SVG and copy it
        src_svg = CONFIG_DIR / 'trajectories' / 'extract_left.svg'
        dst_svg = svg_dir / 'extract_left.svg'
        dst_svg.write_text(src_svg.read_text())

        with open(config_path, 'w') as f:
            yaml.dump(config, f)

        return config_path, config

    def test_converter_reads_scaling_from_config(self, temp_output_dir, test_config):
        """AC2: Converter uses scaling from config, not hardcoded values."""
        import subprocess
        import sys

        config_path, config = test_config
        output_path = temp_output_dir / 'output.yaml'

        # Run converter with test config
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS_DIR / 'svg_to_trajectory.py'),
                '--config',
                str(config_path),
                '--trajectory',
                'test_trajectory',
                '-o',
                str(output_path),
            ],
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0, f"Converter failed: {result.stderr}"

        # Load output
        with open(output_path) as f:
            output = yaml.safe_load(f)

        # Verify output uses config scaling
        # With x_range [0, 200] and y_output [0.0, 0.8]:
        # SVG X=0 -> Y=0.0, SVG X=200 -> Y=0.8
        # But SVG has X range 0-100, so actual max Y should be 0.4
        insertion = output['trajectories']['insertion']

        # First point should start at 0
        assert insertion[0]['y'] == pytest.approx(0.0, abs=0.01)

        # Last point: SVG X=100 maps to Y = (100-0)/(200-0) * 0.8 = 0.4
        assert insertion[-1]['y'] == pytest.approx(0.4, abs=0.01)

    def test_multiple_paths_produce_multiple_trajectories(self, temp_output_dir):
        """AC3: SVG with multiple paths produces multiple trajectories."""
        import subprocess
        import sys

        output_path = temp_output_dir / 'output.yaml'

        # Run converter with actual config
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS_DIR / 'svg_to_trajectory.py'),
                '--config',
                str(CONFIG_DIR / 'trajectory_config.yaml'),
                '--trajectory',
                'extract_left',
                '-o',
                str(output_path),
            ],
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0, f"Converter failed: {result.stderr}"

        with open(output_path) as f:
            output = yaml.safe_load(f)

        # SVG has 'insertion' and 'extraction' paths
        assert 'insertion' in output['trajectories']
        assert 'extraction' in output['trajectories']
        assert len(output['trajectories']) == 2

    def test_yz_scaling_matches_config(self, temp_output_dir):
        """AC3: Y/Z values match expected output from config mapping."""
        import subprocess
        import sys

        output_path = temp_output_dir / 'output.yaml'

        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS_DIR / 'svg_to_trajectory.py'),
                '--config',
                str(CONFIG_DIR / 'trajectory_config.yaml'),
                '--trajectory',
                'extract_left',
                '-o',
                str(output_path),
            ],
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0

        with open(output_path) as f:
            output = yaml.safe_load(f)

        insertion = output['trajectories']['insertion']

        # Config: x_range [0,100] -> y_output [0.0, 0.4]
        # SVG insertion path: M 0,50 C ... 100,50
        # First point: X=0 -> Y=0.0
        assert insertion[0]['y'] == pytest.approx(0.0, abs=0.001)
        # Last point: X=100 -> Y=0.4
        assert insertion[-1]['y'] == pytest.approx(0.4, abs=0.001)

        # Config: y_center=50, z_scale=0.001
        # SVG Y=50 -> Z=0
        # First/last points have Y=50 in SVG, so Z should be ~0
        assert insertion[0]['z'] == pytest.approx(0.0, abs=0.001)
        assert insertion[-1]['z'] == pytest.approx(0.0, abs=0.001)

    def test_output_includes_config_reference(self, temp_output_dir):
        """AC3: Output YAML includes source_svg and config_used references."""
        import subprocess
        import sys

        output_path = temp_output_dir / 'output.yaml'

        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS_DIR / 'svg_to_trajectory.py'),
                '--config',
                str(CONFIG_DIR / 'trajectory_config.yaml'),
                '--trajectory',
                'extract_left',
                '-o',
                str(output_path),
            ],
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0

        with open(output_path) as f:
            output = yaml.safe_load(f)

        # AC3: Required fields
        assert 'source_svg' in output
        assert 'config_used' in output
        assert 'trajectories' in output
        assert 'generated' in output

        # Verify references are paths
        assert 'extract_left.svg' in output['source_svg']
        assert 'trajectory_config.yaml' in output['config_used']

    def test_waypoint_count_matches_config(self, temp_output_dir, test_config):
        """Verify waypoint count matches num_points in config."""
        import subprocess
        import sys

        config_path, config = test_config
        output_path = temp_output_dir / 'output.yaml'

        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS_DIR / 'svg_to_trajectory.py'),
                '--config',
                str(config_path),
                '--trajectory',
                'test_trajectory',
                '-o',
                str(output_path),
            ],
            capture_output=True,
            text=True,
        )

        assert result.returncode == 0

        with open(output_path) as f:
            output = yaml.safe_load(f)

        # Config specifies num_points: 10
        expected_points = config['trajectories']['test_trajectory']['sampling'][
            'num_points'
        ]
        for traj_name, waypoints in output['trajectories'].items():
            assert (
                len(waypoints) == expected_points
            ), f"{traj_name} has {len(waypoints)} points, expected {expected_points}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
