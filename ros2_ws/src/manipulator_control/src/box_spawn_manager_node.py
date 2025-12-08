#!/usr/bin/env python3
"""
Box Spawn Manager Node

Manages dynamic box spawning with department child links in the TF tree.
Provides SpawnBox and DespawnBox services for box extraction operations.

Story 4A.3: Implement Dynamic Box Spawner with Department Frame Generation

Services:
    /manipulator/box_spawn/spawn (SpawnBox) - Spawn box with TF frames
    /manipulator/box_spawn/despawn (DespawnBox) - Remove box and cleanup

AC3: When SpawnBox completes, robot_state_publisher is running and TF frames queryable
AC4: Box base_link attached to gripper_magnet frame via static transform
AC5: In simulation, box model spawned in Gazebo at gripper position
AC6: In simulation, DetachableJoint plugin configured and initially attached
AC8: DespawnBox terminates RSP, stops static transform, deletes Gazebo model
"""

import os
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import yaml

from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from ament_index_python.packages import get_package_share_directory

from manipulator_control.srv import SpawnBox, DespawnBox

# Import URDF generator
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.box_urdf_generator import (
    generate_box_urdf_from_address,
    load_storage_params
)


@dataclass
class ActiveBox:
    """Tracks a spawned box's resources."""
    box_id: str
    rsp_process: subprocess.Popen
    urdf: str
    urdf_file_path: str  # Path to temp URDF file
    num_departments: int
    source_address: dict = field(default_factory=dict)
    gripper_frame: str = ""


class BoxSpawnManagerNode(Node):
    """
    ROS2 node managing dynamic box spawning with TF frames and Gazebo integration.

    Spawns boxes by:
    1. Generating URDF with base_link and department child links
    2. Launching robot_state_publisher subprocess for TF broadcasting
    3. Publishing static transform gripper_magnet -> box_base_link
    4. (Simulation) Spawning model in Gazebo with DetachableJoint
    """

    def __init__(self):
        super().__init__('box_spawn_manager_node')

        # Load configuration
        self._load_config()
        self._load_storage_params()

        # State tracking
        self.active_boxes: Dict[str, ActiveBox] = {}

        # TF2 for position lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Static transform broadcaster for gripper->box attachment
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Detect simulation mode by checking for Gazebo services
        self.simulation_mode = self._check_gazebo_available()

        # Service servers
        self.spawn_srv = self.create_service(
            SpawnBox,
            self.config.get('spawn_service_name', '/manipulator/box_spawn/spawn'),
            self._spawn_callback
        )
        self.despawn_srv = self.create_service(
            DespawnBox,
            self.config.get('despawn_service_name', '/manipulator/box_spawn/despawn'),
            self._despawn_callback
        )

        # DetachableJoint publishers cache
        self._detachable_joint_pubs: Dict[str, tuple] = {}

        self.get_logger().info('Box Spawn Manager Node started')
        self.get_logger().info(f"Simulation mode: {self.simulation_mode}")
        self.get_logger().info(f"Spawn service: {self.config.get('spawn_service_name')}")
        self.get_logger().info(f"Despawn service: {self.config.get('despawn_service_name')}")

    def _load_config(self):
        """Load configuration from box_spawner.yaml."""
        self.config = {
            'tf_publish_rate': 10.0,
            'gazebo_world_name': 'empty',
            'box_mass': 0.5,
            'department_marker_radius': 0.01,
            'left_gripper_frame': 'left_gripper_magnet',
            'right_gripper_frame': 'right_gripper_magnet',
            'spawn_service_name': '/manipulator/box_spawn/spawn',
            'despawn_service_name': '/manipulator/box_spawn/despawn',
            'spawn_timeout_sec': 5.0,
            'despawn_timeout_sec': 5.0,
            'rsp_startup_wait_sec': 0.5,
            'box_color': {'r': 0.3, 'g': 0.5, 'b': 0.8, 'a': 1.0},
            'department_marker_color': {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.8}
        }

        try:
            pkg_share = get_package_share_directory('manipulator_control')
            config_path = os.path.join(pkg_share, 'config', 'box_spawner.yaml')

            with open(config_path, 'r') as f:
                loaded = yaml.safe_load(f)

            if loaded:
                self.config.update(loaded)

            self.get_logger().info(f'Loaded config from: {config_path}')

        except Exception as e:
            self.get_logger().warn(f'Failed to load config, using defaults: {e}')

    def _load_storage_params(self):
        """Load storage_params.yaml for box dimensions."""
        try:
            self.storage_params = load_storage_params()
            self.get_logger().info('Loaded storage_params.yaml')
        except Exception as e:
            self.get_logger().error(f'Failed to load storage_params: {e}')
            self.storage_params = {}

    def _check_gazebo_available(self) -> bool:
        """Check if Gazebo simulation is available using gz service CLI."""
        world_name = self.config.get('gazebo_world_name', 'empty')

        # Check if gz service for spawn exists
        try:
            result = subprocess.run(
                ['gz', 'service', '--list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )

            spawn_service = f'/world/{world_name}/create'
            if spawn_service in result.stdout:
                self.get_logger().info(f'Gazebo service {spawn_service} available - simulation mode')
                return True
            else:
                self.get_logger().info(f'Gazebo service {spawn_service} not found - hardware mode')
                return False

        except Exception as e:
            self.get_logger().debug(f'Gazebo check failed: {e}')
            return False

    def _get_gripper_frame(self, side: str) -> str:
        """Get gripper frame name for given side."""
        if side.lower() in ['left', 'l']:
            return self.config.get('left_gripper_frame', 'left_gripper_magnet')
        else:
            return self.config.get('right_gripper_frame', 'right_gripper_magnet')

    def _launch_rsp(self, box_id: str, urdf: str) -> tuple:
        """
        Launch robot_state_publisher subprocess for a box.

        Args:
            box_id: Box identifier
            urdf: URDF XML string

        Returns:
            Tuple of (subprocess handle, temp file path)
        """
        # Write URDF to a temporary file
        urdf_dir = tempfile.mkdtemp(prefix=f'box_urdf_{box_id}_')
        urdf_file_path = os.path.join(urdf_dir, f'{box_id}.urdf')

        with open(urdf_file_path, 'w') as f:
            f.write(urdf)

        self.get_logger().info(f'Wrote URDF to: {urdf_file_path}')

        # Create params YAML file for robot_state_publisher
        # This avoids issues with XML special characters on command line
        use_sim_time = self.simulation_mode
        params_file_path = os.path.join(urdf_dir, f'{box_id}_params.yaml')

        params_content = {
            f'rsp_{box_id}': {
                'ros__parameters': {
                    'robot_description': urdf,
                    'use_sim_time': use_sim_time
                }
            }
        }

        with open(params_file_path, 'w') as f:
            yaml.dump(params_content, f, default_flow_style=False)

        self.get_logger().info(f'Wrote params to: {params_file_path}')

        # Launch RSP with params file
        # CRITICAL: Remap robot_description topic to avoid overwriting main robot URDF
        # Without this, RViz RobotModel display gets the box URDF instead of the robot
        cmd = [
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args',
            '--params-file', params_file_path,
            '-r', f'__node:=rsp_{box_id}',
            '-r', f'robot_description:={box_id}/robot_description'
        ]

        self.get_logger().debug(f'Launching RSP for {box_id}: {" ".join(cmd)}')
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        return process, urdf_file_path

    def _publish_gripper_to_box_transform(self, box_id: str, gripper_frame: str):
        """
        Publish static transform from gripper to box base_link.

        AC4: Box attached to gripper_magnet frame via static transform.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = gripper_frame
        t.child_frame_id = f'{box_id}_base_link'

        # Zero offset - box origin at gripper origin
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published static TF: {gripper_frame} -> {box_id}_base_link')

    def _get_gripper_pose_in_world(self, gripper_frame: str) -> Optional[Pose]:
        """
        Get gripper pose in world frame for Gazebo spawning.

        Returns:
            Pose message or None if lookup fails.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                gripper_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose

        except TransformException as e:
            self.get_logger().warn(f'Failed to lookup {gripper_frame} pose: {e}')
            return None

    def _spawn_in_gazebo(self, box_id: str, urdf_file_path: str, gripper_frame: str) -> bool:
        """
        Spawn box model in Gazebo simulation using gz service CLI.

        AC5: Box model spawned in Gazebo at gripper position.

        Uses gz service CLI because Gazebo Harmonic spawn services are not
        bridged to ROS2 by default.

        Returns:
            True if spawn succeeded.
        """
        # Get gripper pose
        pose = self._get_gripper_pose_in_world(gripper_frame)
        if pose is None:
            self.get_logger().error('Cannot get gripper pose for Gazebo spawn')
            return False

        # Get world name from config
        world_name = self.config.get('gazebo_world_name', 'empty')

        # Build gz service request
        # EntityFactory message: sdf_filename, name, pose
        pose_str = (
            f'position: {{x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}}}, '
            f'orientation: {{x: {pose.orientation.x}, y: {pose.orientation.y}, '
            f'z: {pose.orientation.z}, w: {pose.orientation.w}}}'
        )

        req = f'sdf_filename: "{urdf_file_path}", name: "{box_id}", pose: {{{pose_str}}}'

        cmd = [
            'gz', 'service',
            '-s', f'/world/{world_name}/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req
        ]

        self.get_logger().debug(f'Gazebo spawn cmd: {" ".join(cmd)}')

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10.0
            )

            if result.returncode == 0 and 'data: true' in result.stdout:
                self.get_logger().info(f'Spawned {box_id} in Gazebo')
                return True
            else:
                self.get_logger().error(f'Gazebo spawn failed: {result.stderr or result.stdout}')
                return False

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Gazebo spawn timeout for {box_id}')
            return False
        except Exception as e:
            self.get_logger().error(f'Gazebo spawn error: {e}')
            return False

    def _attach_in_gazebo(self, box_id: str):
        """
        Publish attach message to DetachableJoint topic.

        AC6: Box initially attached to gripper via DetachableJoint.
        """
        attach_topic = f'/model/{box_id}/detachable_joint/attach'

        if box_id not in self._detachable_joint_pubs:
            detach_topic = f'/model/{box_id}/detachable_joint/detach'
            attach_pub = self.create_publisher(Empty, attach_topic, 10)
            detach_pub = self.create_publisher(Empty, detach_topic, 10)
            self._detachable_joint_pubs[box_id] = (attach_pub, detach_pub)

        attach_pub, _ = self._detachable_joint_pubs[box_id]

        # Small delay for publisher to register
        time.sleep(0.1)
        attach_pub.publish(Empty())
        self.get_logger().info(f'Published attach for {box_id}')

    def _delete_from_gazebo(self, box_id: str) -> bool:
        """
        Delete box model from Gazebo using gz service CLI.

        Returns:
            True if delete succeeded.
        """
        # Get world name from config
        world_name = self.config.get('gazebo_world_name', 'empty')

        # Entity message: name and type (MODEL=2)
        req = f'name: "{box_id}", type: MODEL'

        cmd = [
            'gz', 'service',
            '-s', f'/world/{world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req
        ]

        self.get_logger().debug(f'Gazebo delete cmd: {" ".join(cmd)}')

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10.0
            )

            if result.returncode == 0 and 'data: true' in result.stdout:
                self.get_logger().info(f'Deleted {box_id} from Gazebo')
                return True
            else:
                self.get_logger().warn(f'Gazebo delete may have failed: {result.stderr or result.stdout}')
                return False

        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'Gazebo delete timeout for {box_id}')
            return False
        except Exception as e:
            self.get_logger().warn(f'Gazebo delete error: {e}')
            return False

    def _detach_in_gazebo(self, box_id: str):
        """Publish detach message to DetachableJoint topic."""
        if box_id in self._detachable_joint_pubs:
            _, detach_pub = self._detachable_joint_pubs[box_id]
            detach_pub.publish(Empty())
            self.get_logger().info(f'Published detach for {box_id}')
            time.sleep(0.2)  # Wait for physics detach

    def _spawn_callback(self, request, response):
        """
        Handle SpawnBox service request.

        Phases:
        1. Generate URDF with base_link and department links
        2. Launch robot_state_publisher subprocess
        3. Publish static transform gripper->box
        4. (Simulation) Spawn in Gazebo and attach
        """
        box_id = request.box_id
        side = request.side
        cabinet_num = request.cabinet_num
        row = request.row
        column = request.column

        self.get_logger().info(f'SpawnBox request: {box_id} at {side}/{cabinet_num}/{row}/{column}')

        # Validate box_id is not empty
        if not box_id or not box_id.strip():
            response.success = False
            response.box_id = box_id
            response.department_count = 0
            response.message = 'box_id cannot be empty'
            self.get_logger().error('SpawnBox rejected: empty box_id')
            return response

        # Check if already spawned
        if box_id in self.active_boxes:
            response.success = False
            response.box_id = box_id
            response.department_count = 0
            response.message = f'Box {box_id} already spawned'
            return response

        gripper_frame = self._get_gripper_frame(side)

        try:
            # Phase 1: Generate URDF
            urdf, num_departments = generate_box_urdf_from_address(
                box_id=box_id,
                side=side,
                cabinet_num=cabinet_num,
                row=row,
                column=column,
                gripper_frame=gripper_frame,
                spawner_config=self.config,
                storage_params=self.storage_params,
                include_gazebo_plugin=self.simulation_mode
            )
            self.get_logger().info(f'Generated URDF for {box_id}: {num_departments} departments')

            # Phase 2: Launch robot_state_publisher
            rsp_process, urdf_file_path = self._launch_rsp(box_id, urdf)

            # Wait for RSP to start
            wait_time = self.config.get('rsp_startup_wait_sec', 1.0)  # Increased wait time
            time.sleep(wait_time)

            # Check RSP is running
            if rsp_process.poll() is not None:
                # Try to get error output
                _, stderr = rsp_process.communicate(timeout=1)
                err_msg = stderr.decode() if stderr else 'Unknown error'
                self.get_logger().error(f'RSP stderr: {err_msg}')
                response.success = False
                response.box_id = box_id
                response.department_count = 0
                response.message = f'robot_state_publisher failed to start for {box_id}: {err_msg[:100]}'
                # Cleanup temp directory
                try:
                    temp_dir = os.path.dirname(urdf_file_path)
                    for f in os.listdir(temp_dir):
                        os.remove(os.path.join(temp_dir, f))
                    os.rmdir(temp_dir)
                except:
                    pass
                return response

            # Publish static transform gripper->box
            self._publish_gripper_to_box_transform(box_id, gripper_frame)

            # Phase 3 (Simulation): Spawn in Gazebo
            if self.simulation_mode:
                if not self._spawn_in_gazebo(box_id, urdf_file_path, gripper_frame):
                    self.get_logger().warn(f'Gazebo spawn failed, TF frames still available')

                # Attach via DetachableJoint
                self._attach_in_gazebo(box_id)

            # Track active box
            self.active_boxes[box_id] = ActiveBox(
                box_id=box_id,
                rsp_process=rsp_process,
                urdf=urdf,
                urdf_file_path=urdf_file_path,
                num_departments=num_departments,
                source_address={
                    'side': side,
                    'cabinet_num': cabinet_num,
                    'row': row,
                    'column': column
                },
                gripper_frame=gripper_frame
            )

            response.success = True
            response.box_id = box_id
            response.department_count = num_departments
            response.message = f'Spawned {box_id} with {num_departments} departments'

            self.get_logger().info(response.message)
            return response

        except Exception as e:
            self.get_logger().error(f'SpawnBox failed: {e}')
            response.success = False
            response.box_id = box_id
            response.department_count = 0
            response.message = f'SpawnBox error: {str(e)}'
            return response

    def _despawn_callback(self, request, response):
        """
        Handle DespawnBox service request.

        AC8: Terminate RSP, stop static transform, delete Gazebo model.
        """
        box_id = request.box_id

        self.get_logger().info(f'DespawnBox request: {box_id}')

        if box_id not in self.active_boxes:
            response.success = False
            response.message = f'Box {box_id} not found in active boxes'
            return response

        active_box = self.active_boxes[box_id]

        try:
            # Phase 1 (Simulation): Detach and delete from Gazebo
            if self.simulation_mode:
                self._detach_in_gazebo(box_id)
                self._delete_from_gazebo(box_id)

            # Phase 2: Kill robot_state_publisher
            if active_box.rsp_process and active_box.rsp_process.poll() is None:
                active_box.rsp_process.terminate()
                try:
                    active_box.rsp_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    active_box.rsp_process.kill()
                self.get_logger().info(f'Terminated RSP for {box_id}')

            # Cleanup temp directory (URDF and params files)
            if active_box.urdf_file_path:
                try:
                    temp_dir = os.path.dirname(active_box.urdf_file_path)
                    # Remove all files in temp directory
                    for f in os.listdir(temp_dir):
                        os.remove(os.path.join(temp_dir, f))
                    os.rmdir(temp_dir)
                    self.get_logger().debug(f'Removed temp dir: {temp_dir}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to cleanup temp files: {e}')

            # Phase 3: Cleanup
            # Note: Static transform cannot be "removed" - it will just stop being republished
            # The TF tree will eventually drop the stale transform

            # Clean up publishers
            if box_id in self._detachable_joint_pubs:
                attach_pub, detach_pub = self._detachable_joint_pubs.pop(box_id)
                self.destroy_publisher(attach_pub)
                self.destroy_publisher(detach_pub)

            # Remove from tracking
            del self.active_boxes[box_id]

            response.success = True
            response.message = f'Despawned {box_id}'

            self.get_logger().info(response.message)
            return response

        except Exception as e:
            self.get_logger().error(f'DespawnBox failed: {e}')
            response.success = False
            response.message = f'DespawnBox error: {str(e)}'
            return response


def main(args=None):
    """Main entry point for box spawn manager node."""
    rclpy.init(args=args)

    try:
        node = BoxSpawnManagerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in BoxSpawnManagerNode: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
