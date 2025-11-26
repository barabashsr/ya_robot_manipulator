#!/usr/bin/env python3
"""
Controller Interface Utility

Dual-mode utility for commanding joints via:
- JointTrajectoryController (7 motion joints) - action-based with smooth interpolation
- ForwardCommandController (2 container jaws) - topic-based instant commands

NOT a ROS2 node - requires parent node reference for publishers/subscribers/action clients.

Single Source of Truth: Joint soft limits loaded from manipulator_params.yaml
"""

from typing import Dict, List, Optional, Tuple, Callable, Any
import os
import yaml

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Joint classification for dual-mode control
TRAJECTORY_JOINTS = frozenset([
    'base_main_frame_joint',
    'main_frame_selector_frame_joint',
    'selector_frame_gripper_joint',
    'selector_frame_picker_frame_joint',
    'picker_frame_picker_rail_joint',
    'picker_rail_picker_base_joint',
    'picker_base_picker_jaw_joint'
])

FORWARD_COMMAND_JOINTS = frozenset([
    'selector_left_container_jaw_joint',
    'selector_right_container_jaw_joint'
])


class ControllerInterface:
    """
    Dual-mode utility for commanding joints via trajectory or forward command controllers.

    This class is NOT a ROS2 node - it requires a parent node to be passed
    during initialization. This allows action servers to share this utility
    without creating separate node instances.

    Hybrid Architecture:
    - 7 motion joints: JointTrajectoryController (action-based, smooth interpolation)
    - 2 container jaws: ForwardCommandController (topic-based, instant response)

    Usage:
        class MyActionServer(Node):
            def __init__(self):
                super().__init__('my_action_server')
                self.controller = ControllerInterface(self)

                # Wait for action servers on startup
                self.controller.wait_for_action_servers(timeout_sec=30.0)

                # Command any joint (routing is automatic)
                success = self.controller.command_joint('base_main_frame_joint', 1.5)
    """

    def __init__(self, node: Node):
        """
        Initialize ControllerInterface with reference to parent ROS2 node.

        Args:
            node: ROS2 node instance for creating publishers/subscribers/action clients
        """
        self.node = node
        self.logger = node.get_logger()

        # Callback group for action clients (allows concurrent callbacks)
        self._callback_group = ReentrantCallbackGroup()

        # Load joint limits from manipulator_params.yaml (single source of truth)
        self.joint_limits = self._load_joint_limits_from_params()

        if not self.joint_limits:
            self.logger.error('No joint limits loaded - ControllerInterface not functional')
            return

        self.logger.info(f'Loaded limits for {len(self.joint_limits)} joints')

        # Action clients for trajectory joints (7 motion joints)
        self.action_clients: Dict[str, ActionClient] = {}
        for joint_name in TRAJECTORY_JOINTS:
            if joint_name in self.joint_limits:
                action_name = f'/{joint_name}_controller/follow_joint_trajectory'
                self.action_clients[joint_name] = ActionClient(
                    node,
                    FollowJointTrajectory,
                    action_name,
                    callback_group=self._callback_group
                )
                self.logger.debug(f'Created action client for {action_name}')

        # Publishers for forward command joints (2 container jaws)
        self.publishers: Dict[str, Any] = {}
        for joint_name in FORWARD_COMMAND_JOINTS:
            if joint_name in self.joint_limits:
                topic = f'/{joint_name}_controller/commands'
                self.publishers[joint_name] = node.create_publisher(Float64MultiArray, topic, 10)
                self.logger.debug(f'Created publisher for {topic}')

        # Track active goals for cancellation
        self._active_goals: Dict[str, Any] = {}

        # Subscribe to joint states for position feedback
        self.joint_positions: Dict[str, float] = {}
        self._joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        self.logger.info(
            f'ControllerInterface initialized: '
            f'{len(self.action_clients)} trajectory joints, '
            f'{len(self.publishers)} forward command joints'
        )

    def _load_joint_limits_from_params(self) -> Dict[str, Dict[str, float]]:
        """
        Load soft limits from manipulator_params.yaml (single source of truth).

        Returns:
            Dict mapping joint_name -> {'min': float, 'max': float, 'velocity': float}
        """
        try:
            pkg_path = get_package_share_directory('manipulator_description')
            params_file = os.path.join(pkg_path, 'config', 'manipulator_params.yaml')

            with open(params_file, 'r') as f:
                params = yaml.safe_load(f)

            limits = {}
            # Search through all assemblies for joint definitions
            for assembly_name, assembly in params.items():
                if not isinstance(assembly, dict):
                    continue
                for key, value in assembly.items():
                    # Joint entries have 'safety_controller' with soft limits
                    if isinstance(value, dict) and 'safety_controller' in value:
                        joint_name = key
                        sc = value['safety_controller']
                        limits[joint_name] = {
                            'min': sc['soft_lower'],
                            'max': sc['soft_upper'],
                            'velocity': value.get('limits', {}).get('velocity', 1.0),
                        }

            self.logger.info(f'Loaded joint limits from {params_file}')
            return limits

        except FileNotFoundError as e:
            self.logger.error(f'manipulator_params.yaml not found: {e}')
            return {}
        except Exception as e:
            self.logger.error(f'Error loading joint limits: {e}')
            return {}

    def _joint_state_cb(self, msg: JointState) -> None:
        """
        Update position cache from /joint_states topic.

        Args:
            msg: JointState message containing joint positions
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def _calculate_duration(self, joint_name: str, current: float, target: float) -> float:
        """
        Calculate trajectory duration based on distance and max velocity.

        Args:
            joint_name: Joint to calculate duration for
            current: Current position
            target: Target position

        Returns:
            Duration in seconds, clamped to [1.0, 30.0] for physics stability
        """
        distance = abs(target - current)
        max_velocity = self.joint_limits[joint_name]['velocity']

        if max_velocity > 0:
            duration = distance / max_velocity
        else:
            duration = 2.0

        # Clamp to reasonable range - minimum 1.0s for physics simulation stability
        # Too fast trajectories cause physics lag and tolerance violations
        return max(1.0, min(30.0, duration))

    def _build_trajectory_goal(
        self,
        joint_name: str,
        target: float,
        duration: float,
        current_position: Optional[float] = None
    ) -> FollowJointTrajectory.Goal:
        """
        Build FollowJointTrajectory goal message with proper two-point trajectory.

        Creates a trajectory with:
        - Point 1 (t=0): Current position with zero velocity (starting point)
        - Point 2 (t=duration): Target position with zero velocity (ending point)

        This ensures smooth interpolation from current state to target.

        Args:
            joint_name: Joint to move
            target: Target position
            duration: Time to reach target
            current_position: Current joint position (required for proper interpolation)

        Returns:
            FollowJointTrajectory.Goal message
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        # IMPORTANT: Leave header.stamp at default (0,0) for immediate execution.
        # A zero timestamp tells the trajectory controller to start executing NOW.
        # Reference: https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html

        # Point 1: Starting point at t=0 with current position
        # This establishes the initial state for smooth interpolation
        if current_position is not None:
            start_point = JointTrajectoryPoint()
            start_point.positions = [current_position]
            start_point.velocities = [0.0]  # Starting from rest
            start_point.accelerations = [0.0]
            start_point.time_from_start = Duration(sec=0, nanosec=0)
            trajectory.points.append(start_point)

        # Point 2: Target point at t=duration
        end_point = JointTrajectoryPoint()
        end_point.positions = [target]
        end_point.velocities = [0.0]  # Stop at target
        end_point.accelerations = [0.0]  # Explicit zero acceleration
        end_point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        trajectory.points.append(end_point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        return goal

    def wait_for_action_servers(self, timeout_sec: float = 30.0) -> bool:
        """
        Wait for all trajectory action servers to become available.

        Args:
            timeout_sec: Maximum time to wait for each server

        Returns:
            True if all servers available, False if any timed out
        """
        all_ready = True
        for joint_name, client in self.action_clients.items():
            action_name = f'/{joint_name}_controller/follow_joint_trajectory'
            self.logger.info(f'Waiting for action server: {action_name}')

            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.logger.error(f'Action server not available: {action_name}')
                all_ready = False
            else:
                self.logger.debug(f'Action server ready: {action_name}')

        if all_ready:
            self.logger.info('All trajectory action servers are ready')

        return all_ready

    def cancel_trajectory(self, joint_name: str) -> bool:
        """
        Cancel active trajectory goal for a joint.

        Args:
            joint_name: Joint to cancel trajectory for

        Returns:
            True if cancellation sent, False if no active goal or not trajectory joint
        """
        if joint_name not in TRAJECTORY_JOINTS:
            self.logger.warning(f'{joint_name} is not a trajectory joint')
            return False

        if joint_name not in self._active_goals:
            self.logger.debug(f'No active goal to cancel for {joint_name}')
            return False

        goal_handle = self._active_goals[joint_name]
        try:
            cancel_future = goal_handle.cancel_goal_async()
            self.logger.info(f'Cancellation requested for {joint_name}')
            del self._active_goals[joint_name]
            return True
        except Exception as e:
            self.logger.error(f'Failed to cancel goal for {joint_name}: {e}')
            return False

    def command_joint(
        self,
        joint_name: str,
        position: float,
        duration_sec: Optional[float] = None
    ) -> bool:
        """
        Command a single joint to target position.

        Routes automatically to action client (trajectory joints) or
        publisher (forward command joints).

        Args:
            joint_name: Name of joint to command
            position: Target position (validated against soft limits)
            duration_sec: Optional trajectory duration (trajectory joints only)
                         If None, calculated from distance and max velocity

        Returns:
            True if command sent successfully, False if validation failed
        """
        # Validate joint exists
        if joint_name not in self.joint_limits:
            self.logger.warning(f'Unknown joint: {joint_name}')
            return False

        # Validate position against soft limits
        limits = self.joint_limits[joint_name]
        if not (limits['min'] <= position <= limits['max']):
            self.logger.warning(
                f"Position {position} outside soft limits "
                f"[{limits['min']}, {limits['max']}] for {joint_name}"
            )
            return False

        # Route to appropriate interface
        if joint_name in TRAJECTORY_JOINTS:
            return self._command_trajectory_joint(joint_name, position, duration_sec)
        elif joint_name in FORWARD_COMMAND_JOINTS:
            return self._command_forward_joint(joint_name, position)
        else:
            self.logger.error(f'Joint {joint_name} not in TRAJECTORY_JOINTS or FORWARD_COMMAND_JOINTS')
            return False

    def _command_trajectory_joint(
        self,
        joint_name: str,
        position: float,
        duration_sec: Optional[float] = None
    ) -> bool:
        """
        Command trajectory joint via action client.

        Sends trajectory goal asynchronously and returns immediately.
        The trajectory controller will interpolate to the target position.

        Args:
            joint_name: Joint name
            position: Target position
            duration_sec: Optional duration override

        Returns:
            True if goal sent successfully, False if validation failed
        """
        if joint_name not in self.action_clients:
            self.logger.error(f'No action client for {joint_name}')
            return False

        # Calculate duration if not provided
        current_pos = self.get_joint_position(joint_name)
        if current_pos is None:
            current_pos = 0.0  # Fallback if no joint state received yet

        if duration_sec is None:
            duration_sec = self._calculate_duration(joint_name, current_pos, position)

        # Build and send goal with current position for proper trajectory interpolation
        goal = self._build_trajectory_goal(joint_name, position, duration_sec, current_pos)

        client = self.action_clients[joint_name]

        # Wait for action server if not ready (with timeout)
        if not client.server_is_ready():
            self.logger.info(f'Waiting for action server: {joint_name}_controller')
            if not client.wait_for_server(timeout_sec=10.0):
                self.logger.error(f'Action server not available: {joint_name}_controller')
                return False
            self.logger.info(f'Action server ready: {joint_name}_controller')

        # Send goal asynchronously with callback for tracking
        send_goal_future = client.send_goal_async(goal)

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self._active_goals[joint_name] = goal_handle
                    self.logger.info(f'Trajectory goal accepted for {joint_name}')
                else:
                    self.logger.warning(f'Trajectory goal rejected for {joint_name}')
            except Exception as e:
                self.logger.error(f'Goal response error for {joint_name}: {e}')

        send_goal_future.add_done_callback(goal_response_callback)

        self.logger.info(
            f'Trajectory goal sent: {joint_name} -> {position} '
            f'(duration: {duration_sec:.2f}s)'
        )
        return True

    def _command_forward_joint(self, joint_name: str, position: float) -> bool:
        """
        Command forward command joint via publisher.

        Args:
            joint_name: Joint name
            position: Target position

        Returns:
            True if command published
        """
        if joint_name not in self.publishers:
            self.logger.error(f'No publisher for {joint_name}')
            return False

        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.publishers[joint_name].publish(msg)

        self.logger.debug(f'Forward command sent: {joint_name} -> {position}')
        return True

    def command_trajectory_with_callback(
        self,
        joint_name: str,
        position: float,
        duration_sec: Optional[float] = None,
        feedback_callback: Optional[Callable] = None,
        result_callback: Optional[Callable] = None
    ) -> bool:
        """
        Command trajectory joint with optional callbacks.

        Args:
            joint_name: Joint name (must be in TRAJECTORY_JOINTS)
            position: Target position
            duration_sec: Optional duration override
            feedback_callback: Called with feedback during motion
            result_callback: Called when trajectory completes

        Returns:
            True if goal sent successfully
        """
        if joint_name not in TRAJECTORY_JOINTS:
            self.logger.warning(f'{joint_name} is not a trajectory joint')
            return False

        if joint_name not in self.action_clients:
            self.logger.error(f'No action client for {joint_name}')
            return False

        # Validate position
        limits = self.joint_limits[joint_name]
        if not (limits['min'] <= position <= limits['max']):
            self.logger.warning(
                f"Position {position} outside soft limits "
                f"[{limits['min']}, {limits['max']}] for {joint_name}"
            )
            return False

        # Calculate duration if not provided
        current_pos = self.get_joint_position(joint_name)
        if current_pos is None:
            current_pos = 0.0

        if duration_sec is None:
            duration_sec = self._calculate_duration(joint_name, current_pos, position)

        # Build goal with current position for proper trajectory interpolation
        goal = self._build_trajectory_goal(joint_name, position, duration_sec, current_pos)

        client = self.action_clients[joint_name]
        if not client.server_is_ready():
            self.logger.warning(f'Action server not ready for {joint_name}')
            return False

        # Send goal with callbacks
        send_goal_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback
        )

        def goal_response_callback(future):
            goal_handle = future.result()
            if goal_handle.accepted:
                self._active_goals[joint_name] = goal_handle
                self.logger.debug(f'Goal accepted for {joint_name}')

                # Get result async
                if result_callback:
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(
                        lambda f: result_callback(f.result())
                    )
            else:
                self.logger.warning(f'Goal rejected for {joint_name}')
                if result_callback:
                    result_callback(None)

        send_goal_future.add_done_callback(goal_response_callback)

        self.logger.debug(
            f'Trajectory goal with callbacks sent: {joint_name} -> {position} '
            f'(duration: {duration_sec:.2f}s)'
        )
        return True

    def command_joint_group(self, joint_names: List[str], positions: List[float]) -> bool:
        """
        Command multiple joints simultaneously.

        Routes each joint to appropriate interface.
        Returns False if ANY joint fails validation (no partial commands sent).

        Args:
            joint_names: List of joint names to command
            positions: List of target positions (same order as joint_names)

        Returns:
            True if all commands sent, False if any validation failed
        """
        if len(joint_names) != len(positions):
            self.logger.warning(
                f'Mismatched lengths: {len(joint_names)} joints, {len(positions)} positions'
            )
            return False

        # Validate all joints first (fail fast, no partial commands)
        for joint_name, position in zip(joint_names, positions):
            if joint_name not in self.joint_limits:
                self.logger.warning(f'Unknown joint in group: {joint_name}')
                return False

            limits = self.joint_limits[joint_name]
            if not (limits['min'] <= position <= limits['max']):
                self.logger.warning(
                    f"Position {position} outside soft limits "
                    f"[{limits['min']}, {limits['max']}] for {joint_name}"
                )
                return False

        # All validated - send commands
        for joint_name, position in zip(joint_names, positions):
            self.command_joint(joint_name, position)

        return True

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """
        Get current joint position from /joint_states cache.

        Args:
            joint_name: Name of joint to query

        Returns:
            Current position, or None if joint not found or no data received
        """
        return self.joint_positions.get(joint_name)

    def get_joint_limits(self, joint_name: str) -> Optional[Tuple[float, float]]:
        """
        Get soft limits for a joint.

        Args:
            joint_name: Name of joint to query

        Returns:
            Tuple (min_limit, max_limit), or None if joint unknown
        """
        if joint_name not in self.joint_limits:
            return None
        limits = self.joint_limits[joint_name]
        return (limits['min'], limits['max'])

    def get_all_joint_names(self) -> List[str]:
        """
        Get list of all known joint names.

        Returns:
            List of joint names with configured limits
        """
        return list(self.joint_limits.keys())

    def is_trajectory_joint(self, joint_name: str) -> bool:
        """
        Check if joint uses trajectory controller.

        Args:
            joint_name: Joint name to check

        Returns:
            True if trajectory joint, False otherwise
        """
        return joint_name in TRAJECTORY_JOINTS

    def is_forward_command_joint(self, joint_name: str) -> bool:
        """
        Check if joint uses forward command controller.

        Args:
            joint_name: Joint name to check

        Returns:
            True if forward command joint, False otherwise
        """
        return joint_name in FORWARD_COMMAND_JOINTS
