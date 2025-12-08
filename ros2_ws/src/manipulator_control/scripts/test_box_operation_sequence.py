#!/usr/bin/env python3
"""Test full box operation sequence with trajectory visualization.

Demonstrates the complete workflow for box extraction/insertion:

PHASES:
1. NAVIGATE: Position robot so end effector (magnet) can reach address
2. APPROACH: Straight line Y movement INTO cabinet to align magnet with address origin
3. EXTRACT: Execute curved trajectory to pull box OUT of cabinet
4. INSERT: Execute curved trajectory to push box back INTO cabinet
5. RETRACT: Straight line Y movement back to center (gripper joint at Y=0)

KEY CONCEPT - End Effector vs Joint Position:
- The magnet (end effector) is 0.040m AHEAD of the gripper joint in Y direction
- Trajectory waypoints define END EFFECTOR positions, not joint positions
- Joint position = End effector position - MAGNET_OFFSET
- Example: To get magnet at Y=0.4 (at address), joint must be at Y=0.36

COORDINATE SYSTEM (Address Frame):
- Address frame origin is at the box location (inside cabinet)
- Y=0 in address frame = at the box
- Y=-0.4 in address frame = outside cabinet (at robot center)
- Extraction markers: Y=0 (green start) → Y=-0.4 (red end)
- Insertion markers: Y=-0.4 (green start) → Y=0 (red end)

TRAJECTORY SHAPE:
- Both extraction and insertion follow the same curved path (UP arc)
- Extraction goes OUT (Y decreasing in world)
- Insertion goes IN (Y increasing in world)

Usage:
    1. Launch simulation: ros2 launch manipulator_control manipulator_simulation.launch.py
    2. In RViz, add MarkerArray for /trajectory_markers topic
    3. Run: python3 scripts/test_box_operation_sequence.py --side left --cabinet 1 --row 5 --col 2

    Options:
        --side: Cabinet side (left/right)
        --cabinet: Cabinet number (1-4)
        --row: Row number
        --col: Column number
        --no-execute: Only show markers, do not execute trajectories
        --pause: Pause between phases for inspection
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
from manipulator_control.action import NavigateToAddress
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

# Add src to path
PACKAGE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(PACKAGE_DIR / 'src'))

from utils.yz_trajectory_generator import YZTrajectoryGenerator


class BoxOperationTestNode(Node):
    """Node for testing full box operation sequence with visualization."""

    # End effector frames per side (loaded from config)
    END_EFFECTOR_FRAMES = {
        'left': 'left_gripper_magnet',
        'right': 'right_gripper_magnet',
    }

    # These will be loaded from config files
    MAGNET_OFFSET = 0.040  # Default, overridden by config
    CABINET_DEPTH = 0.4    # Default, overridden by config

    def __init__(self, side: str = 'left'):
        super().__init__('box_operation_test_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.side = side
        config_dir = PACKAGE_DIR / 'config'

        # Load config files
        self._load_config(config_dir)

        self.end_effector_frame = self.END_EFFECTOR_FRAMES.get(side, 'left_gripper_magnet')

        # Current joint states for trajectory base positions
        self._current_joint_positions = {}
        self._joint_states_lock = threading.Lock()
        self._joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10
        )

        # Marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray, '/trajectory_markers', 10
        )

        # Action clients
        self.navigate_client = ActionClient(
            self, NavigateToAddress, '/navigate_to_address'
        )
        self.y_client = ActionClient(
            self, FollowJointTrajectory,
            '/selector_frame_gripper_joint_controller/follow_joint_trajectory'
        )
        self.z_client = ActionClient(
            self, FollowJointTrajectory,
            '/main_frame_selector_frame_joint_controller/follow_joint_trajectory'
        )

        # Trajectory generator
        self.generator = YZTrajectoryGenerator(
            waypoints_path=str(config_dir / 'extraction_trajectories.yaml'),
            config_path=str(config_dir / 'trajectory_config.yaml'),
        )

        self.get_logger().info(
            f'BoxOperationTest initialized for {side} side, '
            f'end effector: {self.end_effector_frame}'
        )
        self.get_logger().info(
            f'Config: magnet_offset={self.magnet_offset}m, cabinet_depth={self.cabinet_depth}m'
        )

    def _load_config(self, config_dir: Path):
        """Load configuration from YAML files."""
        import yaml

        # Load kinematic_chains.yaml for magnet_offset
        kinematic_chains_path = config_dir / 'kinematic_chains.yaml'
        with open(kinematic_chains_path) as f:
            kinematic_config = yaml.safe_load(f)

        gripper_config = kinematic_config.get('joint_groups', {}).get('gripper', {})
        self.magnet_offset = gripper_config.get('magnet_offset', self.MAGNET_OFFSET)

        # Load end effector frames from config
        ee_frames = gripper_config.get('end_effector_frames', {})
        if ee_frames:
            self.END_EFFECTOR_FRAMES.update(ee_frames)

        # Load trajectory_config.yaml for cabinet_depth (y_output max)
        trajectory_config_path = config_dir / 'trajectory_config.yaml'
        with open(trajectory_config_path) as f:
            trajectory_config = yaml.safe_load(f)

        # Get cabinet depth from y_output range (max value)
        extract_left = trajectory_config.get('trajectories', {}).get('extract_left', {})
        mapping = extract_left.get('mapping', {})
        y_output = mapping.get('y_output', [0.0, self.CABINET_DEPTH])
        self.cabinet_depth = max(y_output)  # Max Y value is the cabinet depth

    def _joint_states_callback(self, msg: JointState):
        """Cache current joint positions."""
        with self._joint_states_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._current_joint_positions[name] = msg.position[i]

    def get_current_joint_position(self, joint_name: str) -> float:
        """Get current position of a specific joint."""
        with self._joint_states_lock:
            return self._current_joint_positions.get(joint_name, 0.0)


def wait_for_action_servers(node: BoxOperationTestNode, timeout_sec: float = 15.0) -> bool:
    """Wait for all required action servers to be available."""
    node.get_logger().info('Waiting for action servers...')

    servers = [
        ('navigate_to_address', node.navigate_client),
        ('Y-axis trajectory', node.y_client),
        ('Z-axis trajectory', node.z_client),
    ]

    for name, client in servers:
        if not client.wait_for_server(timeout_sec=timeout_sec):
            node.get_logger().error(f'{name} action server not available')
            return False
        node.get_logger().info(f'  {name}: OK')

    return True


def navigate_to_address(
    node: BoxOperationTestNode,
    side: str,
    cabinet_num: int,
    row: int,
    column: int,
    approach_distance: float = 0.0,
    timeout_sec: float = 30.0,
) -> bool:
    """Navigate to address using NavigateToAddress action."""
    node.get_logger().info(
        f'Navigating to {side}-{cabinet_num}-{row}-{column} '
        f'(approach: {approach_distance:.3f}m)...'
    )

    goal = NavigateToAddress.Goal()
    goal.side = side
    goal.cabinet_num = cabinet_num
    goal.row = row
    goal.column = column
    goal.approach_distance = approach_distance

    # Send goal and wait
    result_event = threading.Event()
    result_holder = {'success': False, 'message': ''}

    def goal_response_callback(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            result_holder['message'] = 'Goal rejected'
            result_event.set()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_callback)

    def result_callback(future):
        result = future.result().result
        result_holder['success'] = result.success
        result_holder['message'] = result.message
        result_event.set()

    def feedback_callback(feedback_msg):
        fb = feedback_msg.feedback
        node.get_logger().info(
            f'  Navigation progress: {fb.progress_percent}%, '
            f'distance: {fb.distance_to_target:.3f}m'
        )

    send_future = node.navigate_client.send_goal_async(
        goal, feedback_callback=feedback_callback
    )
    send_future.add_done_callback(goal_response_callback)

    if not result_event.wait(timeout=timeout_sec):
        node.get_logger().error(f'Navigation timeout ({timeout_sec}s)')
        return False

    if result_holder['success']:
        node.get_logger().info(f'Navigation complete: {result_holder["message"]}')
    else:
        node.get_logger().error(f'Navigation failed: {result_holder["message"]}')

    return result_holder['success']


def execute_trajectory_sync(
    node: BoxOperationTestNode,
    waypoints: list,
    timeout_sec: float = 30.0,
) -> bool:
    """Execute trajectory synchronously via Y and Z controllers."""
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    result_event = threading.Event()
    results = {'y': None, 'z': None}

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
            if results['y'] is not None and results['z'] is not None:
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


def move_gripper_y(
    node: BoxOperationTestNode,
    target_y: float,
    duration_sec: float = 2.0,
    timeout_sec: float = 10.0,
) -> bool:
    """Move gripper joint to target Y position (straight line).

    Args:
        node: The test node with action clients
        target_y: Target joint Y position in meters
        duration_sec: Time to complete the movement
        timeout_sec: Timeout for the action

    Returns:
        True if movement completed successfully
    """
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    current_y = node.get_current_joint_position('selector_frame_gripper_joint')
    if abs(current_y - target_y) < 0.005:
        node.get_logger().info(f'Gripper already at target Y={target_y:.3f}')
        return True

    node.get_logger().info(f'Moving gripper joint Y: {current_y:.3f} -> {target_y:.3f}')

    result_event = threading.Event()
    result_holder = {'success': False}

    traj = JointTrajectory()
    traj.joint_names = ['selector_frame_gripper_joint']
    point = JointTrajectoryPoint()
    point.positions = [target_y]
    point.velocities = [0.0]
    secs = int(duration_sec)
    nsecs = int((duration_sec - secs) * 1e9)
    point.time_from_start = Duration(sec=secs, nanosec=nsecs)
    traj.points.append(point)

    def goal_response(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            result_event.set()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_callback)

    def result_callback(future):
        result = future.result()
        result_holder['success'] = (
            result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        result_event.set()

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    future = node.y_client.send_goal_async(goal)
    future.add_done_callback(goal_response)

    result_event.wait(timeout=timeout_sec)
    return result_holder['success']


def approach_address(node: BoxOperationTestNode, side: str, depth: float = None, timeout_sec: float = 15.0) -> bool:
    """APPROACH phase: Move gripper straight INTO cabinet to reach address origin.

    After NAVIGATE positions the robot at X-Z, the magnet is at Y~0 (center).
    The address origin (where the box is) is cabinet_depth INTO the cabinet.
    This phase moves the gripper in a straight line to reach the box.

    The gripper joint needs to move to: depth - magnet_offset
    Because: magnet_position = joint_position + magnet_offset
    So: joint_position = magnet_position - magnet_offset = depth - magnet_offset

    For LEFT cabinet: joint Y = 0.4 - 0.040 = 0.36 (positive Y into cabinet)
    For RIGHT cabinet: joint Y = -(0.4 - 0.040) = -0.36 (negative Y into cabinet)
    """
    # Use cabinet_depth from config if not specified
    if depth is None:
        depth = node.cabinet_depth

    # Calculate target joint position
    # magnet needs to reach Y=depth (at address origin)
    # joint position = magnet position - magnet_offset
    sign = 1.0 if side == 'left' else -1.0
    magnet_offset = node.magnet_offset
    target_joint_y = sign * (depth - magnet_offset)

    node.get_logger().info(
        f'APPROACH: Moving magnet to address origin (depth={depth}m)'
    )
    node.get_logger().info(
        f'  Magnet target: Y={sign * depth:.3f}, Joint target: Y={target_joint_y:.3f}'
    )

    # Calculate duration based on distance (slower for longer moves)
    current_y = node.get_current_joint_position('selector_frame_gripper_joint')
    distance = abs(target_joint_y - current_y)
    duration = max(2.0, distance / 0.1)  # ~0.1 m/s speed

    return move_gripper_y(node, target_joint_y, duration_sec=duration, timeout_sec=timeout_sec)


def retract_to_center(node: BoxOperationTestNode, timeout_sec: float = 10.0) -> bool:
    """RETRACT phase: Move gripper joint back to Y=0 (center position).

    After extraction, the gripper is extended. This retracts it back to
    the center position for safe navigation.
    """
    node.get_logger().info('RETRACT: Moving gripper joint back to Y=0 (center)')
    return move_gripper_y(node, 0.0, duration_sec=2.0, timeout_sec=timeout_sec)


def run_box_operation_sequence(
    node: BoxOperationTestNode,
    side: str,
    cabinet_num: int,
    row: int,
    column: int,
    no_execute: bool = False,
    pause_between_phases: bool = False,
) -> bool:
    """Run the complete box operation sequence.

    PHASES:
    1. NAVIGATE: Position robot X-Z so end effector can reach address
    2. APPROACH: Straight line Y movement INTO cabinet to reach box at address
    3. EXTRACT: Execute curved trajectory to pull box OUT of cabinet
    4. INSERT: Execute curved trajectory to push box back IN
    5. RETRACT: Straight line Y movement back to center (gripper joint at Y=0)

    TRAJECTORY COORDINATE SYSTEM:
    - Address frame origin is AT the box location (inside cabinet)
    - Trajectory waypoints are in END EFFECTOR (magnet) coordinates
    - Y=0 means magnet at address origin (touching box)
    - Y=0.4 means magnet 0.4m AWAY from address (outside cabinet)

    So extraction trajectory (Y=0.4→0 in waypoints) is BACKWARDS - it goes from
    outside to inside, not inside to outside. The waypoints define the PATH SHAPE,
    and extraction goes OUT along that path (Y decreasing in world), while
    insertion goes IN along the same path (Y increasing in world).

    After APPROACH, the magnet is at Y=0.4 depth (at address origin).
    - EXTRACT trajectory starts at Y=0.4 (where we are), goes to Y=0 (out)
    - INSERT trajectory starts at Y=0 (where we ended), goes to Y=0.4 (in)
    """

    # Build address frame name for markers
    side_abbrev = 'l' if side == 'left' else 'r'
    address_frame = f'addr_{side_abbrev}_{cabinet_num}_{row}_{column}'

    # Get config values from node (loaded from YAML files)
    cabinet_depth = node.cabinet_depth
    magnet_offset = node.magnet_offset

    # Sign for direction: left cabinet = +Y into cabinet, right cabinet = -Y
    sign = 1.0 if side == 'left' else -1.0

    node.get_logger().info('=' * 60)
    node.get_logger().info(f'BOX OPERATION SEQUENCE: {side}-{cabinet_num}-{row}-{column}')
    node.get_logger().info(f'End effector frame: {node.end_effector_frame}')
    node.get_logger().info(f'Address frame: {address_frame}')
    node.get_logger().info(f'Config: magnet_offset={magnet_offset}m, cabinet_depth={cabinet_depth}m')
    node.get_logger().info('=' * 60)

    # ========================================
    # PHASE 1: Navigate to address (X-Z positioning)
    # ========================================
    node.get_logger().info('')
    node.get_logger().info('PHASE 1: NAVIGATE - Position at address X-Z')
    node.get_logger().info('-' * 40)

    # First retract gripper to center (Y=0) for safe navigation
    if not no_execute:
        if not retract_to_center(node):
            node.get_logger().error('Failed to retract gripper for navigation')
            return False
        time.sleep(0.3)

    if not navigate_to_address(node, side, cabinet_num, row, column):
        node.get_logger().error('Failed to navigate to address')
        return False

    if pause_between_phases:
        node.get_logger().info('Press Enter to continue to Phase 2 (APPROACH)...')
        input()

    # ========================================
    # PHASE 2: APPROACH - Straight line INTO cabinet to reach box
    # ========================================
    node.get_logger().info('')
    node.get_logger().info('PHASE 2: APPROACH - Straight line into cabinet')
    node.get_logger().info('-' * 40)
    node.get_logger().info(f'Moving magnet {cabinet_depth}m into cabinet to reach address')

    if not no_execute:
        if not approach_address(node, side, depth=cabinet_depth):
            node.get_logger().error('Failed to approach address')
            return False
        time.sleep(0.3)

    if pause_between_phases:
        node.get_logger().info('Press Enter to continue to Phase 3 (EXTRACT)...')
        input()

    # ========================================
    # PHASE 3: EXTRACT - Curved trajectory to pull box OUT
    # ========================================
    node.get_logger().info('')
    node.get_logger().info('PHASE 3: EXTRACT - Pull box out of cabinet')
    node.get_logger().info('-' * 40)

    # Current positions - magnet should be at address (depth into cabinet)
    base_z = node.get_current_joint_position('main_frame_selector_frame_joint')

    # The extraction trajectory in the YAML goes from Y=0.4 to Y=0
    # This represents: start at depth, end at outside
    # base_y is the offset applied to convert end effector Y to joint Y
    # For left: joint = end_effector - magnet_offset (because magnet is ahead)
    base_y = -sign * magnet_offset

    node.get_logger().info(
        f'Loading extraction trajectory: base_y={base_y:.3f}, base_z={base_z:.3f}'
    )

    extraction_waypoints = node.generator.load_trajectory(
        name='extraction',
        side=side,
        base_y=base_y,
        base_z=base_z,
    )
    node.get_logger().info(f'Loaded {len(extraction_waypoints)} extraction waypoints')

    if extraction_waypoints:
        first_wp = extraction_waypoints[0]
        last_wp = extraction_waypoints[-1]
        node.get_logger().info(
            f'  Joint Y: {first_wp.y:.3f} -> {last_wp.y:.3f}'
        )

    # Publish markers
    node.get_logger().info(f'Publishing extraction markers in {address_frame} frame')

    def publish_extraction_markers():
        node.generator.publish_trajectory_markers(
            waypoints=extraction_waypoints,
            marker_publisher=node.marker_pub,
            node=node,
            side=side,
            address_frame=address_frame,
        )

    marker_timer = node.create_timer(0.5, publish_extraction_markers)
    publish_extraction_markers()

    if no_execute:
        node.get_logger().info('EXTRACTION markers visible. Press Enter to show insertion markers...')
        input()
    else:
        node.get_logger().info('Extraction markers visible. Executing in 2 seconds...')
        time.sleep(2.0)

        node.get_logger().info('Executing extraction trajectory...')
        success = execute_trajectory_sync(node, extraction_waypoints, timeout_sec=30.0)

        if success:
            node.get_logger().info('Extraction trajectory completed!')
        else:
            node.get_logger().error('Extraction trajectory failed')
            marker_timer.cancel()
            return False

        if pause_between_phases:
            node.get_logger().info('Press Enter to continue to Phase 4 (INSERT)...')
            input()

    marker_timer.cancel()
    node.generator.clear_trajectory_markers(node.marker_pub, node)
    time.sleep(0.5)

    # ========================================
    # PHASE 4: INSERT - Curved trajectory to push box back IN
    # ========================================
    node.get_logger().info('')
    node.get_logger().info('PHASE 4: INSERT - Push box back into cabinet')
    node.get_logger().info('-' * 40)

    # Get current Z for insertion trajectory
    base_z = node.get_current_joint_position('main_frame_selector_frame_joint')

    # The insertion trajectory in the YAML goes from Y=0 to Y=0.4
    # This represents: start at outside, end at depth
    node.get_logger().info(
        f'Loading insertion trajectory: base_y={base_y:.3f}, base_z={base_z:.3f}'
    )

    insertion_waypoints = node.generator.load_trajectory(
        name='insertion',
        side=side,
        base_y=base_y,
        base_z=base_z,
    )
    node.get_logger().info(f'Loaded {len(insertion_waypoints)} insertion waypoints')

    if insertion_waypoints:
        first_wp = insertion_waypoints[0]
        last_wp = insertion_waypoints[-1]
        node.get_logger().info(
            f'  Joint Y: {first_wp.y:.3f} -> {last_wp.y:.3f}'
        )

    # Publish markers
    node.get_logger().info(f'Publishing insertion markers in {address_frame} frame')

    def publish_insertion_markers():
        node.generator.publish_trajectory_markers(
            waypoints=insertion_waypoints,
            marker_publisher=node.marker_pub,
            node=node,
            side=side,
            address_frame=address_frame,
        )

    marker_timer = node.create_timer(0.5, publish_insertion_markers)
    publish_insertion_markers()

    if no_execute:
        node.get_logger().info('INSERTION markers visible. Press Enter to continue to RETRACT...')
        input()
    else:
        node.get_logger().info('Insertion markers visible. Executing in 2 seconds...')
        time.sleep(2.0)

        node.get_logger().info('Executing insertion trajectory...')
        success = execute_trajectory_sync(node, insertion_waypoints, timeout_sec=30.0)

        if success:
            node.get_logger().info('Insertion trajectory completed!')
        else:
            node.get_logger().error('Insertion trajectory failed')
            marker_timer.cancel()
            return False

        if pause_between_phases:
            node.get_logger().info('Press Enter to continue to Phase 5 (RETRACT)...')
            input()

    marker_timer.cancel()
    node.generator.clear_trajectory_markers(node.marker_pub, node)
    time.sleep(0.3)

    # ========================================
    # PHASE 5: RETRACT - Straight line Y back to center (joint Y=0)
    # ========================================
    node.get_logger().info('')
    node.get_logger().info('PHASE 5: RETRACT - Return gripper to center')
    node.get_logger().info('-' * 40)

    if not no_execute:
        if not retract_to_center(node):
            node.get_logger().error('Failed to retract gripper to center')
            return False

    node.get_logger().info('')
    node.get_logger().info('=' * 60)
    node.get_logger().info('BOX OPERATION SEQUENCE COMPLETE')
    node.get_logger().info('=' * 60)

    return True


def main():
    parser = argparse.ArgumentParser(
        description='Test full box operation sequence with trajectory visualization'
    )
    parser.add_argument('--side', choices=['left', 'right'], default='left',
                        help='Cabinet side (default: left)')
    parser.add_argument('--cabinet', type=int, default=1,
                        help='Cabinet number 1-4 (default: 1)')
    parser.add_argument('--row', type=int, default=5,
                        help='Row number (default: 5)')
    parser.add_argument('--col', type=int, default=2,
                        help='Column number (default: 2)')
    parser.add_argument('--no-execute', action='store_true',
                        help='Only show markers, do not execute trajectories')
    parser.add_argument('--pause', action='store_true',
                        help='Pause between phases for inspection')
    args = parser.parse_args()

    rclpy.init()
    node = BoxOperationTestNode(side=args.side)

    # Start executor in background
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for joint states
    node.get_logger().info('Waiting for joint states...')
    time.sleep(1.0)

    # Wait for action servers
    if not wait_for_action_servers(node):
        node.get_logger().error('Action servers not available. Is simulation running?')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return

    # Run sequence
    success = run_box_operation_sequence(
        node=node,
        side=args.side,
        cabinet_num=args.cabinet,
        row=args.row,
        column=args.col,
        no_execute=args.no_execute,
        pause_between_phases=args.pause,
    )

    # Cleanup
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
