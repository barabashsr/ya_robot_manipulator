#!/usr/bin/env python3
"""
Launch file for manipulator GUI control using RQT

Launches:
  - RQT Publisher for manual joint control
  - RQT Reconfigure for dynamic parameter adjustment

Usage:
  ros2 launch manipulator_description manipulator_gui_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # RQT Publisher - for publishing commands to controllers
    rqt_publisher = Node(
        package='rqt_publisher',
        executable='rqt_publisher',
        name='rqt_publisher',
        output='screen'
    )

    # RQT Reconfigure - for dynamic parameter tuning
    rqt_reconfigure = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen'
    )

    return LaunchDescription([
        rqt_publisher,
        rqt_reconfigure,
    ])
