#!/usr/bin/env python3
"""
Debug Joy Control - Shows what's being published to controllers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class JoyDebugger(Node):
    def __init__(self):
        super().__init__('joy_debugger')

        # Subscribe to joy
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Subscribe to all controller commands
        self.controllers = {
            'base_main_frame': self.create_subscription(
                Float64MultiArray, '/base_main_frame_joint_controller/commands',
                lambda msg: self.cmd_callback('base_main_frame', msg), 10),
            'selector_vertical': self.create_subscription(
                Float64MultiArray, '/main_frame_selector_frame_joint_controller/commands',
                lambda msg: self.cmd_callback('selector_vertical', msg), 10),
            'gripper': self.create_subscription(
                Float64MultiArray, '/selector_frame_gripper_joint_controller/commands',
                lambda msg: self.cmd_callback('gripper', msg), 10),
            'container_jaw_L': self.create_subscription(
                Float64MultiArray, '/selector_left_container_jaw_joint_controller/commands',
                lambda msg: self.cmd_callback('container_jaw_L', msg), 10),
            'container_jaw_R': self.create_subscription(
                Float64MultiArray, '/selector_right_container_jaw_joint_controller/commands',
                lambda msg: self.cmd_callback('container_jaw_R', msg), 10),
            'picker_vertical': self.create_subscription(
                Float64MultiArray, '/selector_frame_picker_frame_joint_controller/commands',
                lambda msg: self.cmd_callback('picker_vertical', msg), 10),
            'picker_horizontal': self.create_subscription(
                Float64MultiArray, '/picker_frame_picker_rail_joint_controller/commands',
                lambda msg: self.cmd_callback('picker_horizontal', msg), 10),
            'picker_slider': self.create_subscription(
                Float64MultiArray, '/picker_rail_picker_base_joint_controller/commands',
                lambda msg: self.cmd_callback('picker_slider', msg), 10),
            'picker_jaw': self.create_subscription(
                Float64MultiArray, '/picker_base_picker_jaw_joint_controller/commands',
                lambda msg: self.cmd_callback('picker_jaw', msg), 10),
        }

        self.last_joy = None
        self.get_logger().info('Joy debugger started. Move controller and watch output.')

    def joy_callback(self, msg):
        self.last_joy = msg
        # Only print when D-Pad or buttons change
        if len(msg.axes) > 7:
            dpad_x = msg.axes[6]
            dpad_y = msg.axes[7]
            if abs(dpad_x) > 0.1 or abs(dpad_y) > 0.1:
                self.get_logger().info(f'D-Pad: X={dpad_x:+.2f}, Y={dpad_y:+.2f}')

        # Print Right Stick Y for selector vertical debug
        if len(msg.axes) > 4:
            right_y = msg.axes[4]
            if abs(right_y) > 0.1:
                self.get_logger().info(f'Right Stick Y (axis 4): {right_y:+.2f}')

        if len(msg.buttons) > 4 and msg.buttons[4]:
            self.get_logger().info('L1 PRESSED (enable)')

        # Print button presses for picker jaw debug
        if len(msg.buttons) > 2:
            if msg.buttons[2]:  # Triangle
                self.get_logger().info('TRIANGLE (button 2) - Picker jaw extend')
            if msg.buttons[0]:  # Cross
                self.get_logger().info('CROSS (button 0) - Picker jaw retract')

    def cmd_callback(self, name, msg):
        if len(msg.data) > 0:
            self.get_logger().info(f'{name:20s} -> {msg.data[0]:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
