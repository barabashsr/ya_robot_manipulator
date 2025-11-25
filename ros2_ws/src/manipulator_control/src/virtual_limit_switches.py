#!/usr/bin/env python3
"""
Virtual Limit Switch Simulation Node

Simulates 18 limit switches (2 per joint) for the manipulator by monitoring
joint positions and publishing boolean states when joints reach configured
trigger positions. Used for picker state machine and safety monitoring in Gazebo.

Publishes to: /manipulator/end_switches/* (18 topics, std_msgs/Bool)
Subscribes to: /joint_states (sensor_msgs/JointState)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml
import os


class VirtualLimitSwitchNode(Node):
    """
    Virtual limit switch simulator for manipulator joints.

    Monitors joint positions from /joint_states and publishes boolean switch states
    to /manipulator/end_switches/* topics at configured rate (10 Hz).
    Switch triggers when: abs(joint_pos - trigger_pos) <= tolerance
    """

    def __init__(self):
        super().__init__('virtual_limit_switches')

        # Declare and get configuration file parameter
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value

        if not config_path or not os.path.exists(config_path):
            self.get_logger().error(f'Configuration file not found: {config_path}')
            raise FileNotFoundError(f'Config file required: {config_path}')

        # Load configuration
        self.load_config(config_path)

        # Initialize joint position storage
        self.joint_positions = {}

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create publishers for each switch (18 total)
        self.switch_publishers = {}
        for switch_name in self.switches.keys():
            topic = f'/manipulator/end_switches/{switch_name}'
            self.switch_publishers[switch_name] = self.create_publisher(
                Bool,
                topic,
                10
            )

        # Timer for publishing switch states at configured rate
        publish_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(publish_period, self.publish_switch_states)

        self.get_logger().info(f'Virtual Limit Switch Node started')
        self.get_logger().info(f'Publishing {len(self.switches)} switch states at {self.publish_rate} Hz')
        self.get_logger().info(f'Loaded config from: {config_path}')

    def load_config(self, config_path):
        """Load limit switch configuration from YAML file."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.publish_rate = config.get('publish_rate', 10.0)
        self.switches = config.get('switches', {})

        if not self.switches:
            raise ValueError('No switches defined in configuration file')

        self.get_logger().info(f'Loaded {len(self.switches)} switch configurations')

    def joint_state_callback(self, msg):
        """
        Update current joint positions from /joint_states topic.

        Args:
            msg: sensor_msgs/JointState message containing joint positions
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def publish_switch_states(self):
        """
        Compute and publish all switch states based on current joint positions.

        For each switch:
        - Get current position of associated joint
        - Check if position is within trigger_tolerance of trigger_position
        - Publish Bool(True) if triggered, Bool(False) otherwise
        """
        for switch_name, config in self.switches.items():
            joint_name = config['joint']
            trigger_position = config['trigger_position']
            trigger_tolerance = config['trigger_tolerance']

            # Get current joint position (default to 0.0 if not yet received)
            current_position = self.joint_positions.get(joint_name, 0.0)

            # Check if switch is triggered (within tolerance of trigger position)
            position_error = abs(current_position - trigger_position)
            triggered = position_error <= trigger_tolerance

            # Publish switch state
            msg = Bool()
            msg.data = triggered
            self.switch_publishers[switch_name].publish(msg)


def main(args=None):
    """Main entry point for virtual limit switch node."""
    rclpy.init(args=args)

    try:
        node = VirtualLimitSwitchNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in VirtualLimitSwitchNode: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
