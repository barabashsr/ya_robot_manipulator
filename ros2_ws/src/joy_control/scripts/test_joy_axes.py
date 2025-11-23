#!/usr/bin/env python3
"""
Joystick Axis Tester

Displays which axes and buttons are being pressed in real-time.
Use this to find the correct axis/button indices for your controller.

Usage:
  ros2 run joy_control test_joy_axes.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys


class JoyTester(Node):
    def __init__(self):
        super().__init__('joy_tester')

        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('=== Joystick Axis/Button Tester ===')
        self.get_logger().info('Move sticks and press buttons to see their indices')
        self.get_logger().info('Press Ctrl+C to exit\n')

        # Track previous state to only show changes
        self.prev_axes = []
        self.prev_buttons = []

    def joy_callback(self, msg):
        """Display axis and button states"""

        # Show axes that have changed
        for i, value in enumerate(msg.axes):
            # Show if axis moved significantly from center or previous value
            if abs(value) > 0.1:  # Threshold to filter noise
                if i >= len(self.prev_axes) or abs(value - self.prev_axes[i]) > 0.05:
                    self.get_logger().info(f'Axis[{i}] = {value:+.2f}')

        # Show buttons that are pressed
        for i, value in enumerate(msg.buttons):
            if value == 1:
                if i >= len(self.prev_buttons) or self.prev_buttons[i] != value:
                    self.get_logger().info(f'Button[{i}] PRESSED')

        # Update previous state
        self.prev_axes = list(msg.axes)
        self.prev_buttons = list(msg.buttons)

    def print_full_state(self, msg):
        """Print complete joystick state (for debugging)"""
        print("\n" + "="*50)
        print("AXES:")
        for i, value in enumerate(msg.axes):
            bar = self._make_bar(value)
            print(f"  [{i:2d}] {bar} {value:+.2f}")

        print("\nBUTTONS:")
        button_row = ""
        for i, value in enumerate(msg.buttons):
            status = "[X]" if value else "[ ]"
            button_row += f"{i:2d}:{status} "
            if (i + 1) % 8 == 0:
                print(f"  {button_row}")
                button_row = ""
        if button_row:
            print(f"  {button_row}")
        print("="*50)

    def _make_bar(self, value):
        """Create a visual bar for axis value"""
        # Value ranges from -1.0 to +1.0
        # Create a 20-character bar
        center = 10
        pos = int(center + value * 10)
        pos = max(0, min(20, pos))

        bar = ['-'] * 21
        bar[center] = '|'
        bar[pos] = '#'
        return ''.join(bar)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTester()

    print("\n" + "="*60)
    print("JOYSTICK TESTER - Move sticks/press buttons to see indices")
    print("="*60)
    print("\nCommon PlayStation Controller Layout:")
    print("  Axes:")
    print("    0 = Left Stick X-axis")
    print("    1 = Left Stick Y-axis")
    print("    2 = L2 Trigger")
    print("    3 = Right Stick X-axis")
    print("    4 = Right Stick Y-axis")
    print("    5 = R2 Trigger")
    print("    6 = D-Pad X-axis")
    print("    7 = D-Pad Y-axis")
    print("\n  Buttons:")
    print("    0 = Cross (X)")
    print("    1 = Circle")
    print("    2 = Triangle")
    print("    3 = Square")
    print("    4 = L1")
    print("    5 = R1")
    print("    6 = L2 (button)")
    print("    7 = R2 (button)")
    print("    8 = Share")
    print("    9 = Options")
    print("   10 = PS Button")
    print("   11 = Left Stick Click")
    print("   12 = Right Stick Click")
    print("\nWatching for input...\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nJoystick tester stopped.")
        print("Update your YAML config with the correct axis/button indices!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
