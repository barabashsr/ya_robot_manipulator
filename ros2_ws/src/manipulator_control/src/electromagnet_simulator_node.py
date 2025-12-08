#!/usr/bin/env python3
"""
Electromagnet Simulator Node

Simulates electromagnet attachment/detachment in Gazebo using DetachableJoint plugin.
Provides ROS2 service interface for box extraction actions to control magnet state.

Story 4A.2: Implement Electromagnet Simulator and Service

Service:
    /manipulator/electromagnet/toggle (ToggleElectromagnet)

Publishes:
    /manipulator/electromagnet/engaged (std_msgs/Bool) @ 10 Hz

DetachableJoint Topics (per box):
    /model/{box_id}/detachable_joint/attach (std_msgs/Empty)
    /model/{box_id}/detachable_joint/detach (std_msgs/Empty)
"""

import math
import os
import re
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import yaml

from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

from manipulator_control.srv import ToggleElectromagnet


class ElectromagnetSimulatorNode(Node):
    """
    ROS2 node simulating electromagnet attachment/detachment via Gazebo DetachableJoint.

    Provides service-based control for magnet engagement with proximity check.
    Publishes magnet state at configured rate for visualization and upstream coordination.
    """

    def __init__(self):
        super().__init__('electromagnet_simulator_node')

        # Load configuration
        self._load_config()

        # State tracking
        self.engaged = False
        self.attached_box_id: Optional[str] = None
        self.active_gripper_frame: Optional[str] = None

        # TF2 for proximity checks
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Attach/detach publishers cache: box_id -> (attach_pub, detach_pub)
        self._detachable_joint_pubs: dict = {}

        # Service server
        self.srv = self.create_service(
            ToggleElectromagnet,
            self.config['service_name'],
            self._toggle_callback
        )

        # State publisher
        self.state_pub = self.create_publisher(
            Bool,
            self.config['state_topic'],
            10
        )

        # State publication timer
        publish_period = 1.0 / self.config['state_publish_rate']
        self.state_timer = self.create_timer(publish_period, self._publish_state)

        self.get_logger().info('Electromagnet Simulator Node started')
        self.get_logger().info(f"Service: {self.config['service_name']}")
        self.get_logger().info(f"State topic: {self.config['state_topic']} @ {self.config['state_publish_rate']} Hz")
        self.get_logger().info(f"Proximity distance: {self.config['proximity_distance']}m")

    def _load_config(self):
        """Load configuration from electromagnet.yaml."""
        self.config = {
            'proximity_distance': 0.05,
            'engagement_wait_sec': 0.5,
            'disengagement_wait_sec': 0.3,
            'state_topic': '/manipulator/electromagnet/engaged',
            'state_publish_rate': 10.0,
            'left_gripper_frame': 'left_gripper_magnet',
            'right_gripper_frame': 'right_gripper_magnet',
            'box_frame_pattern': 'box_*_base_link',
            'service_name': '/manipulator/electromagnet/toggle'
        }

        try:
            pkg_share = get_package_share_directory('manipulator_control')
            config_path = os.path.join(pkg_share, 'config', 'electromagnet.yaml')

            with open(config_path, 'r') as f:
                loaded = yaml.safe_load(f)

            if loaded:
                self.config.update(loaded)

            self.get_logger().info(f'Loaded config from: {config_path}')

        except Exception as e:
            self.get_logger().warn(f'Failed to load config, using defaults: {e}')

    def _get_all_tf_frames(self) -> list:
        """Get all available TF frames."""
        try:
            return self.tf_buffer.all_frames_as_string()
        except Exception:
            return ""

    def _find_box_frames(self) -> list:
        """
        Find all TF frames matching box pattern.

        Returns list of box frame IDs (e.g., ['box_l_1_2_3_base_link', ...])
        """
        frames_str = self._get_all_tf_frames()
        # Pattern: box_{side}_{cabinet}_{row}_{col}_base_link
        pattern = r'box_[lr]_\d+_\d+_\d+_base_link'
        matches = re.findall(pattern, frames_str)
        return list(set(matches))

    def _box_frame_to_model_id(self, box_frame: str) -> str:
        """
        Convert box TF frame to Gazebo model ID.

        box_l_1_2_3_base_link -> box_l_1_2_3
        """
        return box_frame.replace('_base_link', '')

    def _check_box_proximity(self, gripper_frame: str) -> Optional[Tuple[str, float]]:
        """
        Find closest box within proximity distance.

        Args:
            gripper_frame: TF frame of gripper magnet (left_gripper_magnet or right_gripper_magnet)

        Returns:
            Tuple of (box_id, distance) if box found within proximity_distance, else None.
            box_id is the Gazebo model name (e.g., 'box_l_1_2_3')
        """
        box_frames = self._find_box_frames()
        if not box_frames:
            self.get_logger().debug('No box frames found in TF tree')
            return None

        closest_box = None
        closest_distance = float('inf')

        for box_frame in box_frames:
            try:
                # Lookup transform from gripper to box
                transform = self.tf_buffer.lookup_transform(
                    gripper_frame,
                    box_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)
                )

                # Calculate distance
                t = transform.transform.translation
                distance = math.sqrt(t.x**2 + t.y**2 + t.z**2)

                self.get_logger().debug(f'{box_frame} distance from {gripper_frame}: {distance:.3f}m')

                if distance < closest_distance:
                    closest_distance = distance
                    closest_box = box_frame

            except TransformException as e:
                self.get_logger().debug(f'TF lookup failed for {box_frame}: {e}')
                continue

        if closest_box and closest_distance <= self.config['proximity_distance']:
            box_id = self._box_frame_to_model_id(closest_box)
            self.get_logger().info(f'Box {box_id} in proximity: {closest_distance:.3f}m')
            return (box_id, closest_distance)

        if closest_box:
            self.get_logger().debug(
                f'Closest box {closest_box} at {closest_distance:.3f}m '
                f'exceeds proximity threshold {self.config["proximity_distance"]}m'
            )

        return None

    def _get_detachable_joint_publishers(self, box_id: str) -> Tuple:
        """
        Get or create attach/detach publishers for a box.

        Args:
            box_id: Gazebo model ID (e.g., 'box_l_1_2_3')

        Returns:
            Tuple of (attach_publisher, detach_publisher)
        """
        if box_id not in self._detachable_joint_pubs:
            attach_topic = f'/model/{box_id}/detachable_joint/attach'
            detach_topic = f'/model/{box_id}/detachable_joint/detach'

            attach_pub = self.create_publisher(Empty, attach_topic, 10)
            detach_pub = self.create_publisher(Empty, detach_topic, 10)

            self._detachable_joint_pubs[box_id] = (attach_pub, detach_pub)
            self.get_logger().debug(f'Created publishers for {box_id}')

        return self._detachable_joint_pubs[box_id]

    def _toggle_callback(self, request, response):
        """
        Handle ToggleElectromagnet service requests.

        AC2: activate=true with box nearby -> attach, engaged=true
        AC3: activate=true without box -> fail, engaged=false
        AC4: activate=false with box attached -> detach, engaged=false
        """
        if request.activate:
            return self._handle_activate(response)
        else:
            return self._handle_deactivate(response)

    def _handle_activate(self, response):
        """Handle magnet activation request."""
        if self.engaged:
            response.success = True
            response.magnet_engaged = True
            response.message = f'Already engaged with {self.attached_box_id}'
            return response

        # Check proximity for both grippers, prefer left
        left_frame = self.config['left_gripper_frame']
        right_frame = self.config['right_gripper_frame']

        result = self._check_box_proximity(left_frame)
        gripper_frame = left_frame

        if result is None:
            result = self._check_box_proximity(right_frame)
            gripper_frame = right_frame

        if result is None:
            # AC3: No box in proximity
            response.success = False
            response.magnet_engaged = False
            response.message = f'No box within {self.config["proximity_distance"]}m of gripper'
            self.get_logger().warn(response.message)
            return response

        box_id, distance = result

        # Publish attach message
        attach_pub, _ = self._get_detachable_joint_publishers(box_id)
        attach_pub.publish(Empty())

        self.get_logger().info(f'Published attach to {box_id} (distance: {distance:.3f}m)')

        # Update state
        self.engaged = True
        self.attached_box_id = box_id
        self.active_gripper_frame = gripper_frame

        response.success = True
        response.magnet_engaged = True
        response.message = f'Engaged with {box_id} at {distance:.3f}m'

        return response

    def _handle_deactivate(self, response):
        """Handle magnet deactivation request."""
        if not self.engaged:
            response.success = True
            response.magnet_engaged = False
            response.message = 'Already disengaged'
            return response

        if self.attached_box_id is None:
            # Shouldn't happen, but handle gracefully
            self.engaged = False
            response.success = True
            response.magnet_engaged = False
            response.message = 'State reset (no box was attached)'
            return response

        # Publish detach message
        _, detach_pub = self._get_detachable_joint_publishers(self.attached_box_id)
        detach_pub.publish(Empty())

        self.get_logger().info(f'Published detach for {self.attached_box_id}')

        # Update state
        released_box = self.attached_box_id
        self.engaged = False
        self.attached_box_id = None
        self.active_gripper_frame = None

        response.success = True
        response.magnet_engaged = False
        response.message = f'Released {released_box}'

        return response

    def _publish_state(self):
        """Publish current magnet engaged state (AC5: 10 Hz)."""
        msg = Bool()
        msg.data = self.engaged
        self.state_pub.publish(msg)


def main(args=None):
    """Main entry point for electromagnet simulator node."""
    rclpy.init(args=args)

    try:
        node = ElectromagnetSimulatorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in ElectromagnetSimulatorNode: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
