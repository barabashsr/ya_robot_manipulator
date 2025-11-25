"""
Launch file for Virtual Limit Switch Simulation Node

Starts the VirtualLimitSwitchNode which simulates 18 limit switches for the manipulator.
Loads configuration from config/limit_switches.yaml in the package share directory.

Usage:
    ros2 launch manipulator_control virtual_limit_switches.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for virtual limit switch node."""

    # Get package share directory
    pkg_share = get_package_share_directory('manipulator_control')

    # Path to limit switch configuration file
    config_file = os.path.join(pkg_share, 'config', 'limit_switches.yaml')

    # Virtual limit switch node
    virtual_limit_switches_node = Node(
        package='manipulator_control',
        executable='virtual_limit_switches',
        name='virtual_limit_switches',
        output='screen',
        parameters=[{
            'config_file': config_file
        }]
    )

    return LaunchDescription([
        virtual_limit_switches_node
    ])
