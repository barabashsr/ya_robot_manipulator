#!/usr/bin/env python3
"""
Launch file for manipulator joystick control

Launches:
  - Joy node (reads joystick inputs)
  - Manipulator joy control node (converts joy to joint commands)

Usage:
  ros2 launch manipulator_description manipulator_joy_control.launch.py

  # With custom joystick device:
  ros2 launch manipulator_description manipulator_joy_control.launch.py joy_dev:=/dev/input/js0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    scale_linear_arg = DeclareLaunchArgument(
        'scale_linear',
        default_value='0.5',
        description='Linear velocity scale (0.0 to 1.0)'
    )

    scale_angular_arg = DeclareLaunchArgument(
        'scale_angular',
        default_value='0.3',
        description='Angular velocity scale (0.0 to 1.0)'
    )

    # Get launch configurations
    joy_dev = LaunchConfiguration('joy_dev')
    scale_linear = LaunchConfiguration('scale_linear')
    scale_angular = LaunchConfiguration('scale_angular')

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

    # Manipulator joy control - converts joy messages to joint commands
    joy_control_node = Node(
        package='manipulator_description',
        executable='joy_control.py',
        name='manipulator_joy_control',
        parameters=[{
            'scale_linear': scale_linear,
            'scale_angular': scale_angular,
            'deadzone': 0.1,
        }],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        joy_dev_arg,
        scale_linear_arg,
        scale_angular_arg,

        # Nodes
        joy_node,
        joy_control_node,
    ])
