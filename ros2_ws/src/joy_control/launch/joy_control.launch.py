#!/usr/bin/env python3
"""
Joy Control Launch File

Launches joystick control system with YAML configuration

Usage:
  ros2 launch joy_control joy_control.launch.py

  # With custom config:
  ros2 launch joy_control joy_control.launch.py \
    config_file:=/path/to/custom_config.yaml

  # With custom joystick device:
  ros2 launch joy_control joy_control.launch.py joy_dev:=/dev/input/js1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get package share directory
    pkg_share = FindPackageShare('joy_control')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'manipulator_joy_config.yaml'
        ]),
        description='Path to joy control configuration file'
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    joy_dev = LaunchConfiguration('joy_dev')

    # Joy node - reads joystick hardware
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'dev': joy_dev,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Joy controller node - maps joy to commands with YAML config
    joy_controller_node = Node(
        package='joy_control',
        executable='joy_controller_node.py',
        name='joy_controller_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        config_file_arg,
        joy_dev_arg,

        # Nodes
        joy_node,
        joy_controller_node,
    ])
