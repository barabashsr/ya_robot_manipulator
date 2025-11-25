#!/usr/bin/env python3
"""
Controller Interface Utility

Utility class for commanding individual ForwardCommandControllers.
NOT a ROS2 node - requires parent node reference for publishers/subscribers.

Single Source of Truth: Joint soft limits loaded from manipulator_params.yaml
Topic Pattern: /{joint_name}_controller/command (std_msgs/Float64)
"""

from typing import Dict, List, Optional, Tuple
import os
import yaml

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


class ControllerInterface:
    """
    Utility for commanding individual ForwardCommandControllers.

    This class is NOT a ROS2 node - it requires a parent node to be passed
    during initialization. This allows action servers to share this utility
    without creating separate node instances.

    Usage:
        class MyActionServer(Node):
            def __init__(self):
                super().__init__('my_action_server')
                self.controller = ControllerInterface(self)

                # Command a joint
                success = self.controller.command_joint('base_main_frame_joint', 1.5)
    """

    def __init__(self, node: Node):
        """
        Initialize ControllerInterface with reference to parent ROS2 node.

        Args:
            node: ROS2 node instance for creating publishers/subscribers
        """
        self.node = node
        self.logger = node.get_logger()

        # Load joint limits from manipulator_params.yaml (single source of truth)
        self.joint_limits = self._load_joint_limits_from_params()

        if not self.joint_limits:
            self.logger.error('No joint limits loaded - ControllerInterface not functional')
            return

        self.logger.info(f'Loaded limits for {len(self.joint_limits)} joints')

        # Create publishers for each controller (topic pattern: /{joint_name}_controller/commands)
        # ForwardCommandController uses Float64MultiArray on /commands topic
        self.publishers: Dict[str, any] = {}
        for joint_name in self.joint_limits.keys():
            topic = f'/{joint_name}_controller/commands'
            self.publishers[joint_name] = node.create_publisher(Float64MultiArray, topic, 10)
            self.logger.debug(f'Created publisher for {topic}')

        # Subscribe to joint states for position feedback
        self.joint_positions: Dict[str, float] = {}
        self._joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
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

    def command_joint(self, joint_name: str, position: float) -> bool:
        """
        Command a single joint to target position.

        Args:
            joint_name: Name of joint to command
            position: Target position (validated against soft limits)

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

        # Publish command (Float64MultiArray with single element)
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.publishers[joint_name].publish(msg)
        return True

    def command_joint_group(self, joint_names: List[str], positions: List[float]) -> bool:
        """
        Command multiple joints simultaneously.

        All commands sent in rapid succession. Returns False if ANY joint
        fails validation (no partial commands sent).

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

        # All validated - send commands (Float64MultiArray with single element each)
        for joint_name, position in zip(joint_names, positions):
            msg = Float64MultiArray()
            msg.data = [float(position)]
            self.publishers[joint_name].publish(msg)

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
