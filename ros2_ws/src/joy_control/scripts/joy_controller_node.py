#!/usr/bin/env python3
"""
Configurable Joystick Controller Node with Trajectory Controller Support

Supports hybrid controller architecture:
- 7 motion joints: JointTrajectoryController (smooth interpolation via FollowJointTrajectory action)
- 2 container jaws: ForwardCommandController (instant response via topic publishers)

Joint limits loaded from manipulator_params.yaml (single source of truth).

Author: Generated for ya_robot_manipulator
License: MIT
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory


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


class ConfigurableJoyController(Node):
    """Maps joystick inputs to robot joint controllers with smooth trajectory control"""

    def __init__(self):
        super().__init__('joy_controller_node')

        # Callback group for async action calls
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('trajectory_update_rate', 10.0)
        self.declare_parameter('trajectory_duration_min', 0.05)
        self.declare_parameter('trajectory_duration_max', 2.0)
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('enable_turbo_button', 5)

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.trajectory_update_rate = self.get_parameter('trajectory_update_rate').value
        self.trajectory_duration_min = self.get_parameter('trajectory_duration_min').value
        self.trajectory_duration_max = self.get_parameter('trajectory_duration_max').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_button = self.get_parameter('enable_button').value
        self.enable_turbo_button = self.get_parameter('enable_turbo_button').value

        # Load joint limits from manipulator_params.yaml (SINGLE SOURCE OF TRUTH)
        self.joint_limits = self._load_joint_limits_from_params()
        self.get_logger().info(f'Loaded limits for {len(self.joint_limits)} joints from manipulator_params.yaml')

        # Control state
        self.enabled = False
        self.turbo_enabled = False
        self.ever_enabled = False
        self.joint_states_received = False
        self.last_joy = None
        self.last_status_time = self.get_clock().now()

        # Joint positions and targets
        self.joint_positions = {}  # Current positions from /joint_states
        self.pending_targets = {}  # Accumulated targets from joystick input
        self.last_sent_targets = {}  # Last targets sent to controllers
        self.current_goal_handles = {}  # Active trajectory goal handles for preemption

        # Axis and button mappings
        self.axis_mappings = {}
        self.button_actions = {}
        self._load_axis_mappings()
        self._load_button_actions()

        # Create action clients for trajectory joints
        self.trajectory_clients = {}
        for joint in TRAJECTORY_JOINTS:
            if joint in self.joint_limits:
                action_name = f'/{joint}_controller/follow_joint_trajectory'
                self.trajectory_clients[joint] = ActionClient(
                    self, FollowJointTrajectory, action_name,
                    callback_group=self.callback_group
                )
                self.get_logger().info(f'Created trajectory action client: {action_name}')

        # Create publishers for forward command joints (container jaws)
        self.forward_publishers = {}
        for joint in FORWARD_COMMAND_JOINTS:
            if joint in self.joint_limits:
                topic = f'/{joint}_controller/commands'
                self.forward_publishers[joint] = self.create_publisher(
                    Float64MultiArray, topic, 10
                )
                self.get_logger().info(f'Created forward command publisher: {topic}')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self._joy_callback, 10
        )

        # Trajectory timer (10Hz) for sending accumulated goals
        self.trajectory_timer = self.create_timer(
            1.0 / self.trajectory_update_rate,
            self._send_trajectory_goals
        )

        self.get_logger().info('Joy Controller started with trajectory controller support')
        self.get_logger().info(f'  Trajectory joints: {len(self.trajectory_clients)}')
        self.get_logger().info(f'  Forward command joints: {len(self.forward_publishers)}')
        self.get_logger().info(f'  Hold button {self.enable_button} (L1) to enable control')

    def _load_joint_limits_from_params(self) -> dict:
        """Load limits and velocities from manipulator_params.yaml (SINGLE SOURCE OF TRUTH)"""
        try:
            pkg_path = get_package_share_directory('manipulator_description')
            params_file = os.path.join(pkg_path, 'config', 'manipulator_params.yaml')

            with open(params_file, 'r') as f:
                params = yaml.safe_load(f)

            limits = {}
            for assembly_name, assembly in params.items():
                if not isinstance(assembly, dict):
                    continue
                for key, value in assembly.items():
                    if isinstance(value, dict) and 'safety_controller' in value:
                        sc = value['safety_controller']
                        limits[key] = {
                            'min': sc['soft_lower'],
                            'max': sc['soft_upper'],
                            'velocity': value.get('limits', {}).get('velocity', 1.0),
                        }

            self.get_logger().info(f'Loaded joint limits from {params_file}')
            return limits

        except Exception as e:
            self.get_logger().error(f'Error loading joint limits: {e}')
            return {}

    def _load_axis_mappings(self):
        """Load axis mappings from parameters"""
        mapping_names = [
            'base_main_frame',
            'main_frame_selector_frame',
            'selector_frame_gripper',
            'selector_left_container_jaw',
            'selector_right_container_jaw',
            'selector_frame_picker_frame',
            'picker_frame_picker_rail',
            'picker_rail_picker_base',
            'picker_base_picker_jaw'
        ]

        for name in mapping_names:
            try:
                # Declare parameters for this mapping
                self.declare_parameter(f'axis_mappings.{name}.joint_name', '')
                self.declare_parameter(f'axis_mappings.{name}.axis_index', -1)
                self.declare_parameter(f'axis_mappings.{name}.axis_scale', 1.0)
                self.declare_parameter(f'axis_mappings.{name}.velocity_scale', 0.5)

                joint_name = self.get_parameter(f'axis_mappings.{name}.joint_name').value
                axis_idx = self.get_parameter(f'axis_mappings.{name}.axis_index').value

                if joint_name:
                    self.axis_mappings[name] = {
                        'joint_name': joint_name,
                        'axis_index': axis_idx,
                        'axis_scale': self.get_parameter(f'axis_mappings.{name}.axis_scale').value,
                        'velocity_scale': self.get_parameter(f'axis_mappings.{name}.velocity_scale').value,
                    }
                    self.get_logger().info(f'Loaded axis mapping: {name} -> {joint_name} (axis {axis_idx})')

            except Exception as e:
                self.get_logger().debug(f'Could not load mapping for {name}: {e}')
                continue

    def _load_button_actions(self):
        """Load button action mappings from parameters"""
        action_names = [
            'picker_jaw_extend',
            'picker_jaw_retract',
            'container_jaws_open_left',
            'container_jaws_open_right',
            'container_jaws_close_left',
            'container_jaws_close_right',
            'home_all',
            'emergency_stop',
            'store_position'
        ]

        for name in action_names:
            try:
                self.declare_parameter(f'button_actions.{name}.button_index', -1)
                self.declare_parameter(f'button_actions.{name}.action_type', 'press')
                self.declare_parameter(f'button_actions.{name}.joint', '')
                self.declare_parameter(f'button_actions.{name}.target_position', 0.0)
                self.declare_parameter(f'button_actions.{name}.enabled', True)

                button_idx = self.get_parameter(f'button_actions.{name}.button_index').value
                enabled = self.get_parameter(f'button_actions.{name}.enabled').value

                if button_idx >= 0 and enabled:
                    self.button_actions[name] = {
                        'button_index': button_idx,
                        'action_type': self.get_parameter(f'button_actions.{name}.action_type').value,
                        'joint': self.get_parameter(f'button_actions.{name}.joint').value,
                        'target_position': self.get_parameter(f'button_actions.{name}.target_position').value,
                        'last_state': False
                    }
                    self.get_logger().info(f'Loaded button action: {name} on button {button_idx}')
            except Exception as e:
                self.get_logger().debug(f'Could not load button action {name}: {e}')
                continue

    def _joint_state_cb(self, msg: JointState):
        """Update joint positions from /joint_states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('Initial joint positions received')

    def _apply_deadzone(self, value: float, axis_index: int) -> float:
        """Apply deadzone to axis value"""
        # D-Pad axes (6, 7) are discrete
        if axis_index in [6, 7]:
            return value if abs(value) >= 0.5 else 0.0

        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _joy_callback(self, msg: Joy):
        """Process joystick input - accumulate target positions"""
        self.last_joy = msg

        # Check enable/turbo buttons
        if len(msg.buttons) > self.enable_button:
            self.enabled = msg.buttons[self.enable_button] == 1
            if self.enabled and not self.ever_enabled:
                self.ever_enabled = True
                self.get_logger().info('Control enabled')

        if len(msg.buttons) > self.enable_turbo_button:
            self.turbo_enabled = msg.buttons[self.enable_turbo_button] == 1

        if not self.enabled:
            return

        dt = 1.0 / self.update_rate
        turbo_mult = 2.0 if self.turbo_enabled else 1.0

        # Process axis mappings
        for mapping_name, mapping in self.axis_mappings.items():
            axis_idx = mapping.get('axis_index', -1)
            if axis_idx < 0 or axis_idx >= len(msg.axes):
                continue

            joint_name = mapping.get('joint_name')
            if not joint_name or joint_name not in self.joint_limits:
                continue

            # Read and filter axis value
            raw_value = msg.axes[axis_idx]
            axis_value = self._apply_deadzone(raw_value, axis_idx)

            if abs(axis_value) < 0.01:
                continue

            # Calculate velocity and target
            axis_value *= mapping.get('axis_scale', 1.0)
            velocity = axis_value * mapping.get('velocity_scale', 0.5) * self.scale_linear * turbo_mult

            current = self.joint_positions.get(joint_name, 0.0)
            target = current + velocity * dt

            # Clamp to limits from manipulator_params.yaml
            limits = self.joint_limits[joint_name]
            target = max(limits['min'], min(limits['max'], target))

            # Accumulate target
            self.pending_targets[joint_name] = target

        # Process button actions
        self._process_button_actions(msg, dt, turbo_mult)

    def _process_button_actions(self, msg: Joy, dt: float, turbo_mult: float):
        """Process button actions for container jaws and picker jaw"""
        for action_name, action in self.button_actions.items():
            button_idx = action['button_index']

            if button_idx < 0 or button_idx >= len(msg.buttons):
                continue

            button_pressed = msg.buttons[button_idx] == 1
            action_type = action['action_type']

            if action_type == 'hold' and button_pressed:
                joint = action.get('joint', '')
                target = action.get('target_position', 0.0)

                if not joint:
                    continue

                # Map joint key to full joint name
                joint_name = f'{joint}_joint' if not joint.endswith('_joint') else joint

                if joint_name not in self.joint_limits:
                    continue

                # Move towards target
                current = self.joint_positions.get(joint_name, 0.0)
                direction = 1.0 if target > current else -1.0

                # Get velocity from limits
                max_velocity = self.joint_limits[joint_name].get('velocity', 1.0)
                velocity = direction * max_velocity * 0.15 * turbo_mult

                new_target = current + velocity * dt

                # Clamp to limits
                limits = self.joint_limits[joint_name]
                new_target = max(limits['min'], min(limits['max'], new_target))

                # Also clamp to not overshoot target
                if direction > 0:
                    new_target = min(new_target, target)
                else:
                    new_target = max(new_target, target)

                self.pending_targets[joint_name] = new_target

            elif action_type == 'press':
                if button_pressed and not action['last_state']:
                    self.get_logger().info(f'Button action: {action_name}')

            action['last_state'] = button_pressed

    def _send_trajectory_goals(self):
        """Send accumulated targets as trajectory goals (10Hz timer callback)"""
        if not self.enabled or not self.ever_enabled or not self.joint_states_received:
            return

        for joint_name, target in list(self.pending_targets.items()):
            # Skip if target hasn't changed significantly
            last_sent = self.last_sent_targets.get(joint_name)
            if last_sent is not None and abs(target - last_sent) < 0.001:
                continue

            if joint_name in TRAJECTORY_JOINTS:
                self._send_trajectory_goal(joint_name, target)
            elif joint_name in FORWARD_COMMAND_JOINTS:
                self._send_forward_command(joint_name, target)

            self.last_sent_targets[joint_name] = target

        # Status update periodically
        now = self.get_clock().now()
        if (now - self.last_status_time).nanoseconds / 1e9 > 5.0:
            status = "TURBO" if self.turbo_enabled else "NORMAL"
            active = len(self.pending_targets)
            self.get_logger().info(f'Status: {status} | Active joints: {active}')
            self.last_status_time = now

    def _send_trajectory_goal(self, joint_name: str, target: float):
        """Send trajectory goal with calculated duration"""
        client = self.trajectory_clients.get(joint_name)
        if not client:
            return

        if not client.server_is_ready():
            return

        # Cancel previous goal (preemption for responsiveness)
        if joint_name in self.current_goal_handles:
            old_handle = self.current_goal_handles.get(joint_name)
            if old_handle is not None:
                try:
                    old_handle.cancel_goal_async()
                except Exception:
                    pass

        # Calculate duration from distance and max velocity
        current = self.joint_positions.get(joint_name, target)
        distance = abs(target - current)
        max_velocity = self.joint_limits[joint_name].get('velocity', 1.0)

        duration = distance / max_velocity if max_velocity > 0 else 0.5
        duration = max(self.trajectory_duration_min, min(self.trajectory_duration_max, duration))

        # Build and send trajectory goal
        goal = self._build_trajectory_goal(joint_name, target, duration)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, jn=joint_name: self._on_goal_response(f, jn))

    def _build_trajectory_goal(self, joint_name: str, target: float, duration: float):
        """Build FollowJointTrajectory goal message"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        # Start point at current position
        current = self.joint_positions.get(joint_name, target)
        start_point = JointTrajectoryPoint()
        start_point.positions = [current]
        start_point.velocities = [0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=0)

        # End point at target
        end_point = JointTrajectoryPoint()
        end_point.positions = [target]
        end_point.velocities = [0.0]
        end_point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )

        trajectory.points = [start_point, end_point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def _on_goal_response(self, future, joint_name: str):
        """Store goal handle for potential preemption"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.current_goal_handles[joint_name] = goal_handle
        except Exception as e:
            self.get_logger().debug(f'Goal response error for {joint_name}: {e}')

    def _send_forward_command(self, joint_name: str, target: float):
        """Send forward command for container jaws (instant response)"""
        publisher = self.forward_publishers.get(joint_name)
        if publisher:
            msg = Float64MultiArray()
            msg.data = [target]
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableJoyController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
