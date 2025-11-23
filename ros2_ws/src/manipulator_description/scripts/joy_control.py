#!/usr/bin/env python3
"""
Joystick control node for manipulator

PlayStation Controller Mapping:
  Left Stick Y-axis  : Base railway (X-axis)
  Right Stick Y-axis : Selector vertical (Z-axis)
  Right Stick X-axis : Gripper Y-axis
  L2/R2 Triggers     : Container jaws (open/close)
  D-Pad Up/Down      : Picker vertical (Z-axis)
  D-Pad Left/Right   : Picker Y-axis
  L1/R1              : Picker X-axis slider
  Triangle/Cross     : Picker jaw extension

Usage:
  ros2 run manipulator_description joy_control.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class ManipulatorJoyControl(Node):
    """Converts joystick inputs to manipulator joint commands"""

    def __init__(self):
        super().__init__('manipulator_joy_control')

        # Parameters
        self.declare_parameter('scale_linear', 0.5)  # Max velocity scale
        self.declare_parameter('scale_angular', 0.3)
        self.declare_parameter('deadzone', 0.1)  # Joystick deadzone

        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.deadzone = self.get_parameter('deadzone').value

        # Current joint positions (initialize at zero)
        self.positions = {
            'base_main_frame': 0.0,              # 0.0 to 4.0 m
            'main_frame_selector_frame': 0.0,    # 0.0 to 1.5 m
            'selector_left_container_jaw': 0.0,  # -0.2 to 0.2 m
            'selector_frame_gripper': 0.0,       # -0.4 to 0.4 m
            'selector_frame_picker_frame': 0.0,  # 0.0 to 0.3 m
            'picker_frame_picker_rail': 0.0,     # -0.3 to 0.3 m
            'picker_rail_picker_base': 0.0,      # 0.0 to 0.12 m
            'picker_base_picker_jaw': 0.0,       # 0.0 to 0.2 m
        }

        # Joint limits
        self.limits = {
            'base_main_frame': (0.0, 4.0),
            'main_frame_selector_frame': (0.0, 1.5),
            'selector_left_container_jaw': (-0.2, 0.2),
            'selector_frame_gripper': (-0.4, 0.4),
            'selector_frame_picker_frame': (0.0, 0.3),
            'picker_frame_picker_rail': (-0.3, 0.3),
            'picker_rail_picker_base': (0.0, 0.12),
            'picker_base_picker_jaw': (0.0, 0.2),
        }

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Publishers for each controller (use different name to avoid conflict with node property)
        self.joint_publishers = {
            'base_main_frame': self.create_publisher(
                Float64MultiArray,
                '/base_main_frame_joint_controller/commands',
                10
            ),
            'main_frame_selector_frame': self.create_publisher(
                Float64MultiArray,
                '/main_frame_selector_frame_joint_controller/commands',
                10
            ),
            'selector_left_container_jaw': self.create_publisher(
                Float64MultiArray,
                '/selector_left_container_jaw_joint_controller/commands',
                10
            ),
            'selector_frame_gripper': self.create_publisher(
                Float64MultiArray,
                '/selector_frame_gripper_joint_controller/commands',
                10
            ),
            'selector_frame_picker_frame': self.create_publisher(
                Float64MultiArray,
                '/selector_frame_picker_frame_joint_controller/commands',
                10
            ),
            'picker_frame_picker_rail': self.create_publisher(
                Float64MultiArray,
                '/picker_frame_picker_rail_joint_controller/commands',
                10
            ),
            'picker_rail_picker_base': self.create_publisher(
                Float64MultiArray,
                '/picker_rail_picker_base_joint_controller/commands',
                10
            ),
            'picker_base_picker_jaw': self.create_publisher(
                Float64MultiArray,
                '/picker_base_picker_jaw_joint_controller/commands',
                10
            ),
        }

        # Timer for publishing at fixed rate
        self.timer = self.create_timer(0.05, self.publish_commands)  # 20 Hz

        self.get_logger().info('Manipulator joystick control started')
        self.get_logger().info('Connect PlayStation controller and press buttons to control')

        # Track last joy message
        self.last_joy = None

    def apply_deadzone(self, value):
        """Apply deadzone to joystick axis"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))

    def joy_callback(self, msg):
        """Process joystick inputs"""
        self.last_joy = msg

        dt = 0.05  # Time step (matches timer rate)

        # PlayStation controller axis mapping (may vary by controller)
        # axes[0] = Left stick X
        # axes[1] = Left stick Y
        # axes[2] = L2 trigger
        # axes[3] = Right stick X
        # axes[4] = Right stick Y
        # axes[5] = R2 trigger
        # buttons[0] = Cross (X)
        # buttons[1] = Circle (O)
        # buttons[2] = Triangle
        # buttons[3] = Square
        # buttons[4] = L1
        # buttons[5] = R1
        # buttons[13] = D-pad Up
        # buttons[14] = D-pad Down
        # buttons[15] = D-pad Left
        # buttons[16] = D-pad Right

        # Base railway (X-axis) - Left stick Y
        if len(msg.axes) > 1:
            base_vel = -self.apply_deadzone(msg.axes[1]) * self.scale_linear
            self.positions['base_main_frame'] += base_vel * dt
            self.positions['base_main_frame'] = self.clamp(
                self.positions['base_main_frame'],
                *self.limits['base_main_frame']
            )

        # Selector vertical (Z-axis) - Right stick Y
        if len(msg.axes) > 4:
            selector_vel = -self.apply_deadzone(msg.axes[4]) * self.scale_linear * 0.5
            self.positions['main_frame_selector_frame'] += selector_vel * dt
            self.positions['main_frame_selector_frame'] = self.clamp(
                self.positions['main_frame_selector_frame'],
                *self.limits['main_frame_selector_frame']
            )

        # Gripper Y-axis - Right stick X
        if len(msg.axes) > 3:
            gripper_vel = self.apply_deadzone(msg.axes[3]) * self.scale_angular
            self.positions['selector_frame_gripper'] += gripper_vel * dt
            self.positions['selector_frame_gripper'] = self.clamp(
                self.positions['selector_frame_gripper'],
                *self.limits['selector_frame_gripper']
            )

        # Container jaws - L2/R2 triggers
        if len(msg.axes) > 5:
            # L2 closes (negative), R2 opens (positive)
            jaw_vel = (msg.axes[5] - msg.axes[2]) * self.scale_angular * 0.3
            self.positions['selector_left_container_jaw'] += jaw_vel * dt
            self.positions['selector_left_container_jaw'] = self.clamp(
                self.positions['selector_left_container_jaw'],
                *self.limits['selector_left_container_jaw']
            )

        # Picker vertical - D-pad Up/Down
        if len(msg.buttons) > 14:
            picker_z_vel = 0.0
            if msg.buttons[13]:  # D-pad Up
                picker_z_vel = self.scale_linear * 0.3
            if msg.buttons[14]:  # D-pad Down
                picker_z_vel = -self.scale_linear * 0.3
            self.positions['selector_frame_picker_frame'] += picker_z_vel * dt
            self.positions['selector_frame_picker_frame'] = self.clamp(
                self.positions['selector_frame_picker_frame'],
                *self.limits['selector_frame_picker_frame']
            )

        # Picker Y-axis - D-pad Left/Right
        if len(msg.buttons) > 16:
            picker_y_vel = 0.0
            if msg.buttons[16]:  # D-pad Right
                picker_y_vel = self.scale_angular * 0.5
            if msg.buttons[15]:  # D-pad Left
                picker_y_vel = -self.scale_angular * 0.5
            self.positions['picker_frame_picker_rail'] += picker_y_vel * dt
            self.positions['picker_frame_picker_rail'] = self.clamp(
                self.positions['picker_frame_picker_rail'],
                *self.limits['picker_frame_picker_rail']
            )

        # Picker X-axis slider - L1/R1
        if len(msg.buttons) > 5:
            picker_x_vel = 0.0
            if msg.buttons[5]:  # R1
                picker_x_vel = self.scale_angular * 0.2
            if msg.buttons[4]:  # L1
                picker_x_vel = -self.scale_angular * 0.2
            self.positions['picker_rail_picker_base'] += picker_x_vel * dt
            self.positions['picker_rail_picker_base'] = self.clamp(
                self.positions['picker_rail_picker_base'],
                *self.limits['picker_rail_picker_base']
            )

        # Picker jaw extension - Triangle/Cross
        if len(msg.buttons) > 2:
            picker_jaw_vel = 0.0
            if msg.buttons[2]:  # Triangle
                picker_jaw_vel = self.scale_angular * 0.3
            if msg.buttons[0]:  # Cross
                picker_jaw_vel = -self.scale_angular * 0.3
            self.positions['picker_base_picker_jaw'] += picker_jaw_vel * dt
            self.positions['picker_base_picker_jaw'] = self.clamp(
                self.positions['picker_base_picker_jaw'],
                *self.limits['picker_base_picker_jaw']
            )

    def publish_commands(self):
        """Publish current positions to all controllers"""
        if self.last_joy is None:
            return  # Wait for first joystick message

        for joint_name, position in self.positions.items():
            msg = Float64MultiArray()
            msg.data = [position]
            self.joint_publishers[joint_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorJoyControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
