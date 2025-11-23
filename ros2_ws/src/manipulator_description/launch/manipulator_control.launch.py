#!/usr/bin/env python3
"""
Launch file for manipulator with ROS2 control

Arguments:
  sim: true (default) - Use Gazebo simulation
       false - Use mock hardware interface

Usage:
  ros2 launch manipulator_description manipulator_control.launch.py
  ros2 launch manipulator_description manipulator_control.launch.py sim:=false
"""

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Set Gazebo model path for meshes
    pkg_share_path = get_package_share_directory('manipulator_description')
    gz_models_path = os.pathsep + pkg_share_path

    set_gz_model_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_models_path
    )

    # Launch arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use Gazebo simulation (true) or mock hardware (false)'
    )

    use_sim = LaunchConfiguration('sim')

    # Get package paths
    pkg_share = FindPackageShare('manipulator_description')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    # Robot description (URDF from xacro with sim parameter)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro']),
        ' sim:=', use_sim
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_share, 'config', 'manipulator_controllers.yaml'
    ])

    # ============================================================
    # SIMULATION NODES (when sim=true)
    # ============================================================

    # Gazebo (new Ignition-based)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',  # Empty world
            'on_exit_shutdown': 'true'
        }.items(),
        condition=IfCondition(use_sim)
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'manipulator',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen',
        condition=IfCondition(use_sim)
    )

    # Clock bridge (Gazebo -> ROS2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_sim)
    )

    # ============================================================
    # COMMON NODES (both sim and mock)
    # ============================================================

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim}
        ]
    )

    # Joint State Publisher GUI (for manual testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim}],
        condition=UnlessCondition(use_sim)  # Only when not using Gazebo
    )

    # ============================================================
    # ROS2 CONTROL
    # ============================================================

    # Controller Manager (only for mock hardware)
    controller_manager_mock = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
            {'use_sim_time': use_sim}
        ],
        output='screen',
        condition=UnlessCondition(use_sim)
    )

    # Spawn controllers
    def create_controller_spawner(controller_name, delay=0.0):
        """Create a controller spawner node"""
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '100'
            ],
            parameters=[{'use_sim_time': use_sim}],
            output='screen'
        )

    # Joint State Broadcaster (always first)
    joint_state_broadcaster_spawner = create_controller_spawner('joint_state_broadcaster')

    # Individual joint controllers
    controllers_to_spawn = [
        'base_main_frame_joint_controller',
        'main_frame_selector_frame_joint_controller',
        'selector_left_container_jaw_joint_controller',
        'selector_frame_gripper_joint_controller',
        'selector_frame_picker_frame_joint_controller',
        'picker_frame_picker_rail_joint_controller',
        'picker_rail_picker_base_joint_controller',
        'picker_base_picker_jaw_joint_controller'
    ]

    controller_spawners = [create_controller_spawner(name) for name in controllers_to_spawn]

    # ============================================================
    # VISUALIZATION
    # ============================================================

    # RViz2
    rviz_config = PathJoinSubstitution([
        pkg_share, 'rviz', 'manipulator_control.rviz'
    ])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim}],
        output='screen'
    )

    # ============================================================
    # EVENT HANDLERS
    # ============================================================

    # For simulation: spawn controllers after entity is spawned
    spawn_controllers_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner] + controller_spawners
        ),
        condition=IfCondition(use_sim)
    )

    # For mock: spawn controllers after controller manager starts
    spawn_controllers_after_mock = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_mock,
            on_start=[joint_state_broadcaster_spawner] + controller_spawners
        ),
        condition=UnlessCondition(use_sim)
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    nodes = [
        # Environment
        set_gz_model_path,

        # Arguments
        sim_arg,

        # Core nodes
        robot_state_publisher,

        # Simulation
        gazebo_launch,
        spawn_entity,
        clock_bridge,

        # Mock hardware
        controller_manager_mock,
        joint_state_publisher_gui,

        # Controllers
        spawn_controllers_after_gazebo,
        spawn_controllers_after_mock,

        # Visualization
        rviz2
    ]

    return LaunchDescription(nodes)