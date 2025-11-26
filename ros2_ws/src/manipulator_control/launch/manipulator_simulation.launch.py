#!/usr/bin/env python3
"""
Unified launch file for manipulator simulation with virtual limit switches and joystick control

This launch file combines:
- Gazebo simulation (robot, controllers, RViz)
- Virtual limit switch monitoring
- Joystick control (optional)

Usage:
    ros2 launch manipulator_control manipulator_simulation.launch.py
    ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true

Arguments:
    sim: true (default) - Use Gazebo simulation
         false - Use mock hardware interface
    enable_joy: false (default) - Enable joystick control
                true - Launch joystick control node
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    enable_joy_arg = DeclareLaunchArgument(
        'enable_joy',
        default_value='false',
        description='Enable joystick control'
    )

    enable_joy = LaunchConfiguration('enable_joy')

    # Get package paths
    manipulator_description_share = FindPackageShare('manipulator_description')
    manipulator_control_share = FindPackageShare('manipulator_control')
    joy_control_share = FindPackageShare('joy_control')

    # Include the main manipulator control launch file (Gazebo + controllers + RViz)
    manipulator_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                manipulator_description_share,
                'launch',
                'manipulator_control.launch.py'
            ])
        ]),
        launch_arguments={}.items()
    )

    # Virtual limit switches configuration
    limit_switches_config = PathJoinSubstitution([
        manipulator_control_share,
        'config',
        'limit_switches.yaml'
    ])

    # Virtual limit switches node
    # Delayed start to ensure /joint_states is available
    virtual_limit_switches_node = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to initialize
        actions=[
            Node(
                package='manipulator_control',
                executable='virtual_limit_switches',
                name='virtual_limit_switches',
                output='screen',
                parameters=[{'config_file': limit_switches_config}],
                remappings=[]
            )
        ]
    )

    # MoveJoint action server (Story 2.3)
    # Common node - runs in both sim and hardware modes
    # 3 second delay to allow controllers to initialize
    move_joint_server_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='manipulator_control',
                executable='move_joint_server',
                name='move_joint_server',
                output='screen'
            )
        ]
    )

    # State marker publisher (Story 2.4)
    # Common node - runs in both sim and hardware modes
    # 3 second delay to allow Gazebo to initialize
    state_marker_publisher_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='manipulator_control',
                executable='state_marker_publisher',
                name='state_marker_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # MoveJointGroup action server (Story 2.5)
    # Common node - runs in both sim and hardware modes
    # 3 second delay to allow controllers to initialize
    move_joint_group_server_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='manipulator_control',
                executable='move_joint_group_server',
                name='move_joint_group_server',
                output='screen'
            )
        ]
    )

    # Joystick control launch (optional)
    joy_control_launch = TimerAction(
        period=3.0,  # Wait for controllers to be ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        joy_control_share,
                        'launch',
                        'joy_control.launch.py'
                    ])
                ]),
                launch_arguments={}.items(),
                condition=IfCondition(enable_joy)
            )
        ]
    )

    return LaunchDescription([
        # Declare arguments
        enable_joy_arg,

        # Start Gazebo simulation with controllers
        manipulator_control_launch,

        # Start virtual limit switches (delayed)
        virtual_limit_switches_node,

        # Start MoveJoint action server (delayed) - Story 2.3
        move_joint_server_node,

        # Start state marker publisher (delayed) - Story 2.4
        state_marker_publisher_node,

        # Start MoveJointGroup action server (delayed) - Story 2.5
        move_joint_group_server_node,

        # Start joystick control (optional, delayed)
        joy_control_launch
    ])
