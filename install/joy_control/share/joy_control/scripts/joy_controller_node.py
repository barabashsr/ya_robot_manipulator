#!/usr/bin/env python3
"""
Configurable Joystick Controller Node

Reads configuration from YAML and maps joystick inputs to joint controllers.
Supports axis mappings for movement and button mappings for actions/services.

Author: Generated for ya_robot_manipulator
License: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
import yaml


class ConfigurableJoyController(Node):
    """Maps joystick inputs to robot joint controllers based on YAML configuration"""

    def __init__(self):
        super().__init__('joy_controller_node')

        # Declare parameters
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 0.3)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('enable_turbo_button', 5)

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_button = self.get_parameter('enable_button').value
        self.enable_turbo_button = self.get_parameter('enable_turbo_button').value

        # Control state
        self.enabled = False
        self.turbo_enabled = False
        self.last_joy = None

        # Joint positions (current targets)
        self.positions = {}
        self.axis_mappings = {}
        self.button_actions = {}
        self.joint_publishers_dict = {}

        # Load configuration
        self._load_axis_mappings()
        self._load_button_actions()

        # Create publishers for each mapped controller
        self._create_publishers()

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Timer for publishing commands
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.publish_commands)

        # Status tracking
        self.last_status_time = self.get_clock().now()
        self.get_logger().info('Configurable Joy Controller started')
        self.get_logger().info(f'Loaded {len(self.axis_mappings)} axis mappings')
        self.get_logger().info(f'Loaded {len(self.button_actions)} button actions')
        self.get_logger().info(f'Hold button {self.enable_button} (L1) to enable control')

    def _load_axis_mappings(self):
        """Load axis mappings from parameters"""
        # Get all parameters under 'axis_mappings'
        try:
            # In ROS2, we need to declare parameters first
            # For now, we'll use a simpler approach - load from node parameters

            # Define known axis mappings (from YAML)
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
                # Try to get parameters for this mapping
                try:
                    self.declare_parameter(f'axis_mappings.{name}.controller_topic', '')
                    self.declare_parameter(f'axis_mappings.{name}.axis_index', -1)
                    self.declare_parameter(f'axis_mappings.{name}.axis_scale', 1.0)
                    self.declare_parameter(f'axis_mappings.{name}.velocity_scale', 0.5)
                    self.declare_parameter(f'axis_mappings.{name}.limits', [0.0, 1.0])
                    self.declare_parameter(f'axis_mappings.{name}.control_mode', 'velocity')

                    topic = self.get_parameter(f'axis_mappings.{name}.controller_topic').value
                    axis_idx = self.get_parameter(f'axis_mappings.{name}.axis_index').value

                    if topic and axis_idx >= 0:
                        self.axis_mappings[name] = {
                            'topic': topic,
                            'axis_index': axis_idx,
                            'axis_scale': self.get_parameter(f'axis_mappings.{name}.axis_scale').value,
                            'velocity_scale': self.get_parameter(f'axis_mappings.{name}.velocity_scale').value,
                            'limits': self.get_parameter(f'axis_mappings.{name}.limits').value,
                            'control_mode': self.get_parameter(f'axis_mappings.{name}.control_mode').value,
                        }

                        # Initialize position at midpoint or min
                        limits = self.axis_mappings[name]['limits']
                        self.positions[name] = limits[0] if limits[0] >= 0 else (limits[0] + limits[1]) / 2.0

                        self.get_logger().info(f'Loaded axis mapping: {name} -> {topic} (axis {axis_idx})')

                except Exception as e:
                    self.get_logger().debug(f'Could not load mapping for {name}: {e}')
                    continue

        except Exception as e:
            self.get_logger().error(f'Error loading axis mappings: {e}')

    def _load_button_actions(self):
        """Load button action mappings from parameters"""
        try:
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
                    self.get_logger().warn(f'Could not load button action {name}: {e}')
                    continue

        except Exception as e:
            self.get_logger().error(f'Error loading button actions: {e}')

    def _create_publishers(self):
        """Create publishers for each mapped controller"""
        for name, mapping in self.axis_mappings.items():
            topic = mapping['topic']
            self.joint_publishers_dict[name] = self.create_publisher(
                Float64MultiArray,
                topic,
                10
            )

        # Add publishers for button-controlled joints (not in axis_mappings)
        # Check button actions for joints that need publishers
        for action_name, action in self.button_actions.items():
            joint = action.get('joint', '')
            if joint and joint not in self.joint_publishers_dict:
                # Need to create publisher and position tracking for this joint
                topic = f'/{joint}_joint_controller/commands'
                self.joint_publishers_dict[joint] = self.create_publisher(
                    Float64MultiArray,
                    topic,
                    10
                )
                # Initialize position based on joint type
                if joint not in self.positions:
                    # Container jaws start closed (-0.2), picker jaw starts at 0
                    if 'container_jaw' in joint:
                        self.positions[joint] = -0.2
                        limits = [-0.2, 0.2]
                    else:
                        self.positions[joint] = 0.0
                        limits = [0.0, 0.2]

                    # Add to axis_mappings with limits for clamping
                    self.axis_mappings[joint] = {
                        'topic': topic,
                        'axis_index': -1,
                        'axis_scale': 1.0,
                        'velocity_scale': 0.2,
                        'limits': limits,
                    }
                self.get_logger().info(f'Created publisher for button-controlled joint: {joint}')

    def apply_deadzone(self, value, axis_index=-1):
        """Apply deadzone to axis value"""
        # D-Pad axes (6, 7) are discrete - don't apply scaling
        if axis_index in [6, 7]:
            # D-Pad: just filter small noise
            if abs(value) < 0.5:
                return 0.0
            return value

        # Normal analog sticks
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the remaining range
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))

    def joy_callback(self, msg):
        """Process joystick input"""
        self.last_joy = msg

        # Check enable button
        if len(msg.buttons) > self.enable_button:
            self.enabled = msg.buttons[self.enable_button] == 1

        # Check turbo button
        if len(msg.buttons) > self.enable_turbo_button:
            self.turbo_enabled = msg.buttons[self.enable_turbo_button] == 1

        if not self.enabled:
            return  # Don't process if not enabled

        # Calculate time step
        dt = 1.0 / self.update_rate

        # Apply turbo multiplier
        turbo_mult = 2.0 if self.turbo_enabled else 1.0

        # Process axis mappings
        for name, mapping in self.axis_mappings.items():
            axis_idx = mapping['axis_index']

            if axis_idx >= 0 and len(msg.axes) > axis_idx:
                # Read axis value
                raw_value = msg.axes[axis_idx]
                axis_value = self.apply_deadzone(raw_value, axis_idx)

                # Debug D-Pad and problem axes
                if axis_idx in [6, 7] and abs(raw_value) > 0.1:
                    self.get_logger().info(f'{name}: axis[{axis_idx}] raw={raw_value:.2f} filtered={axis_value:.2f}')

                # Check control mode
                control_mode = mapping.get('control_mode', 'velocity')
                limits = mapping['limits']

                if control_mode == 'position':
                    # Position mode: axis value maps directly to position
                    # For triggers (axis 2, 5): remap from [+1.0 rest, -1.0 pressed] to joint limits
                    if axis_idx in [2, 5]:  # L2, R2 triggers
                        # Trigger: +1.0 (released/closed) to -1.0 (pressed/open)
                        # Apply scale first to handle direction
                        scaled_value = axis_value * mapping['axis_scale']
                        # Map [-scale, +scale] to [limits[0], limits[1]]
                        # When axis*scale = +scale: position = limits[1]
                        # When axis*scale = -scale: position = limits[0]
                        normalized = (scaled_value + abs(mapping['axis_scale'])) / (2.0 * abs(mapping['axis_scale']))
                        position_range = limits[1] - limits[0]
                        self.positions[name] = limits[0] + normalized * position_range
                    else:
                        # Normal axis: map [-1, 1] to limits and apply scale
                        self.positions[name] = axis_value * mapping['axis_scale']
                    self.positions[name] = self.clamp(self.positions[name], limits[0], limits[1])
                else:
                    # Velocity mode: axis value controls velocity
                    axis_value *= mapping['axis_scale']
                    # Velocity mode: axis value controls velocity
                    velocity = axis_value * mapping['velocity_scale'] * self.scale_linear * turbo_mult
                    self.positions[name] += velocity * dt
                    self.positions[name] = self.clamp(self.positions[name], limits[0], limits[1])

        # Process button actions
        for action_name, action in self.button_actions.items():
            button_idx = action['button_index']

            if button_idx >= 0 and len(msg.buttons) > button_idx:
                button_pressed = msg.buttons[button_idx] == 1
                action_type = action['action_type']

                # Debug button presses
                if button_pressed and button_idx in [0, 2]:
                    self.get_logger().info(f'Button action {action_name}: pressed={button_pressed}, type={action_type}, joint={action.get("joint", "none")}')

                # Handle different action types
                if action_type == 'hold':
                    # Continuous action while button held
                    if button_pressed and action['joint']:
                        joint_name = action['joint']
                        target = action['target_position']

                        # Move towards target
                        if joint_name in self.positions:
                            current = self.positions[joint_name]
                            direction = 1.0 if target > current else -1.0
                            velocity = direction * 0.3 * dt
                            self.positions[joint_name] += velocity

                            # Clamp
                            if joint_name in self.axis_mappings:
                                limits = self.axis_mappings[joint_name]['limits']
                                self.positions[joint_name] = self.clamp(
                                    self.positions[joint_name], limits[0], limits[1]
                                )

                elif action_type == 'press':
                    # Single action on button press (not release)
                    if button_pressed and not action['last_state']:
                        # Button just pressed
                        self.get_logger().info(f'Button action: {action_name}')
                        # Future: call service or action here

                # Update last state
                action['last_state'] = button_pressed

    def publish_commands(self):
        """Publish current target positions to all controllers"""
        if self.last_joy is None or not self.enabled:
            return  # Wait for joystick and enable

        for name, position in self.positions.items():
            if name in self.joint_publishers_dict:
                msg = Float64MultiArray()
                msg.data = [position]
                self.joint_publishers_dict[name].publish(msg)

                # Debug picker vertical publishing
                if name == 'selector_frame_picker_frame':
                    self.get_logger().info(f'Publishing {name}: {position:.3f}')

        # Print status periodically
        now = self.get_clock().now()
        if (now - self.last_status_time).nanoseconds / 1e9 > 2.0:  # Every 2 seconds
            status = "TURBO" if self.turbo_enabled else "NORMAL"
            self.get_logger().info(f'Status: {status} | Joints active: {len(self.positions)}')
            self.last_status_time = now


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
