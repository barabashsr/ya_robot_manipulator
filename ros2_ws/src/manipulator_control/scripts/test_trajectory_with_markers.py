#!/usr/bin/env python3
"""Test trajectory execution with RViz visualization.

Demonstrates:
- Loading a trajectory
- Publishing trajectory markers to RViz in a fixed address frame
- Executing the trajectory

Usage:
    1. Launch simulation: ros2 launch manipulator_control manipulator_simulation.launch.py
    2. In RViz, add MarkerArray for /trajectory_markers topic
    3. Run: python3 scripts/test_trajectory_with_markers.py --side left --address addr_l_1_5_2

The script will:
1. Publish the trajectory path as markers in the address frame (fixed in world)
2. Wait for you to see it
3. Execute the trajectory
4. Clear markers after completion

Story 4A-1a: Markers publish in address frame (e.g., addr_l_1_5_2) so they stay
fixed in world space while the gripper moves along the trajectory path.
If no address is specified, markers use 'odom' frame as default.
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import rclpy
import rclpy.parameter
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from visualization_msgs.msg import MarkerArray

# Add src to path
PACKAGE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(PACKAGE_DIR / 'src'))

from utils.yz_trajectory_generator import YZTrajectoryGenerator


class TrajectoryTestNode(Node):
    """Node for testing trajectory execution with visualization."""

    # End effector frames
    END_EFFECTOR_FRAMES = {
        'left': 'left_gripper_magnet',
        'right': 'right_gripper_magnet',
    }

    def __init__(self, side: str = 'left'):
        super().__init__('trajectory_test_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.side = side
        self.frame_id = self.END_EFFECTOR_FRAMES.get(side, 'gripper')

        # Marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray, '/trajectory_markers', 10
        )

        # Action clients
        self.y_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/selector_frame_gripper_joint_controller/follow_joint_trajectory',
        )
        self.z_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/main_frame_selector_frame_joint_controller/follow_joint_trajectory',
        )

        # Trajectory generator
        config_dir = PACKAGE_DIR / 'config'
        self.generator = YZTrajectoryGenerator(
            waypoints_path=str(config_dir / 'extraction_trajectories.yaml'),
            config_path=str(config_dir / 'trajectory_config.yaml'),
        )

        self.get_logger().info(f'Trajectory test node initialized for {side} side')


def execute_trajectory_sync(
    node: TrajectoryTestNode,
    waypoints: list,
    timeout_sec: float = 30.0,
) -> bool:
    """Execute trajectory synchronously with callback-based approach."""
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    result_event = threading.Event()
    results = {'y': False, 'z': False}

    def build_traj(joint_name: str, axis: str) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = [joint_name]
        for i, wp in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = [wp.y if axis == 'y' else wp.z]
            if i == 0 or i == len(waypoints) - 1:
                point.velocities = [0.0]
            secs = int(wp.time_from_start)
            nsecs = int((wp.time_from_start - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            traj.points.append(point)
        return traj

    def send_and_wait(client, traj, axis):
        def goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                results[axis] = False
                check_done()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: result_callback(f, axis))

        def result_callback(future, ax):
            result = future.result()
            results[ax] = (
                result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            check_done()

        def check_done():
            if results['y'] is not False or results['z'] is not False:
                # Check if both have finished (either True or explicitly False)
                y_done = results['y'] is not False or results.get('y_failed', False)
                z_done = results['z'] is not False or results.get('z_failed', False)
                # Simple check: if both are True, we're done
                if results['y'] and results['z']:
                    result_event.set()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = client.send_goal_async(goal)
        future.add_done_callback(goal_response)

    # Build and send trajectories
    y_traj = build_traj('selector_frame_gripper_joint', 'y')
    z_traj = build_traj('main_frame_selector_frame_joint', 'z')

    send_and_wait(node.y_client, y_traj, 'y')
    send_and_wait(node.z_client, z_traj, 'z')

    # Wait for completion
    result_event.wait(timeout=timeout_sec)
    return results['y'] and results['z']


def main():
    parser = argparse.ArgumentParser(description='Test trajectory with visualization')
    parser.add_argument('--side', choices=['left', 'right'], default='left',
                        help='Cabinet side')
    parser.add_argument('--trajectory', choices=['insertion', 'extraction'],
                        default='insertion', help='Trajectory type')
    parser.add_argument('--base-z', type=float, default=0.5,
                        help='Base Z position')
    parser.add_argument('--address', type=str, default=None,
                        help='Target address frame for markers (e.g., addr_l_1_5_2). If not specified, uses odom frame.')
    parser.add_argument('--no-execute', action='store_true',
                        help='Only show markers, do not execute')
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryTestNode(side=args.side)

    # Start executor in background
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for action servers
    node.get_logger().info('Waiting for action servers...')
    if not node.y_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Y-axis action server not available')
        return
    if not node.z_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Z-axis action server not available')
        return

    node.get_logger().info('Action servers ready')

    # Load trajectory
    waypoints = node.generator.load_trajectory(
        name=args.trajectory,
        side=args.side,
        base_y=0.0,
        base_z=args.base_z,
    )

    node.get_logger().info(
        f'Loaded {len(waypoints)} waypoints for {args.trajectory} trajectory'
    )

    # Publish markers in address frame (fixed in world space)
    frame_id = args.address if args.address else 'odom'
    node.get_logger().info(f'Publishing trajectory markers in frame: {frame_id}')
    node.generator.publish_trajectory_markers(
        waypoints=waypoints,
        marker_publisher=node.marker_pub,
        node=node,
        side=args.side,
        address_frame=args.address,
    )

    if args.no_execute:
        node.get_logger().info('Markers published. Press Ctrl+C to exit.')
        try:
            while rclpy.ok():
                time.sleep(1.0)
                # Re-publish markers periodically
                node.generator.publish_trajectory_markers(
                    waypoints=waypoints,
                    marker_publisher=node.marker_pub,
                    node=node,
                    side=args.side,
                    address_frame=args.address,
                )
        except KeyboardInterrupt:
            pass
    else:
        # Create a timer to continuously publish markers
        def publish_markers():
            node.generator.publish_trajectory_markers(
                waypoints=waypoints,
                marker_publisher=node.marker_pub,
                node=node,
                side=args.side,
                address_frame=args.address,
            )

        # Start marker publishing timer (publish every 0.5 seconds)
        marker_timer = node.create_timer(0.5, publish_markers)

        # Initial publish
        publish_markers()

        node.get_logger().info('Trajectory visible in RViz. Executing in 2 seconds...')
        time.sleep(2.0)

        # Execute trajectory
        node.get_logger().info('Executing trajectory...')
        success = execute_trajectory_sync(node, waypoints, timeout_sec=30.0)

        if success:
            node.get_logger().info('Trajectory completed successfully!')
        else:
            node.get_logger().error('Trajectory execution failed')

        # Keep markers visible briefly after completion
        time.sleep(2.0)

        # Stop marker publishing and clear markers
        marker_timer.cancel()
        node.generator.clear_trajectory_markers(node.marker_pub, node)

    # Cleanup
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
