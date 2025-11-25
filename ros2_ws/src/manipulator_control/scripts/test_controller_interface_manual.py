#!/usr/bin/env python3
"""
Manual Gazebo Integration Test for ControllerInterface

Tests all ControllerInterface methods with a running Gazebo simulation.
Requires: ros2 launch manipulator_control manipulator_simulation.launch.py

Run: python3 scripts/test_controller_interface_manual.py
"""

import sys
import os
import time

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import rclpy
from rclpy.node import Node
from controller_interface import ControllerInterface


class ControllerInterfaceTestNode(Node):
    """Test node for manual validation of ControllerInterface."""

    def __init__(self):
        super().__init__('controller_interface_test')
        self.ctrl = ControllerInterface(self)
        self.get_logger().info('ControllerInterface initialized')

    def run_tests(self):
        """Execute all manual tests."""
        print('\n' + '=' * 60)
        print('ControllerInterface Manual Test Suite')
        print('=' * 60)

        # Wait for joint_states to populate
        print('\n[Setup] Waiting for joint_states (2 seconds)...')
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.5)

        passed = 0
        failed = 0

        # Test 1: Get all joint names
        print('\n--- Test 1: Get All Joint Names ---')
        joints = self.ctrl.get_all_joint_names()
        print(f'Found {len(joints)} joints:')
        for j in joints:
            print(f'  - {j}')
        if len(joints) == 9:
            print('✓ PASS: All 9 joints discovered')
            passed += 1
        else:
            print(f'✗ FAIL: Expected 9 joints, got {len(joints)}')
            failed += 1

        # Test 2: Get joint limits
        print('\n--- Test 2: Get Joint Limits ---')
        test_joints = [
            ('base_main_frame_joint', 0.1, 3.9),
            ('main_frame_selector_frame_joint', 0.05, 1.45),
            ('selector_frame_gripper_joint', -0.39, 0.39),
        ]
        limits_ok = True
        for joint_name, expected_min, expected_max in test_joints:
            limits = self.ctrl.get_joint_limits(joint_name)
            if limits and abs(limits[0] - expected_min) < 0.001 and abs(limits[1] - expected_max) < 0.001:
                print(f'  {joint_name}: [{limits[0]}, {limits[1]}] ✓')
            else:
                print(f'  {joint_name}: {limits} ✗ (expected [{expected_min}, {expected_max}])')
                limits_ok = False
        if limits_ok:
            print('✓ PASS: Soft limits match manipulator_params.yaml')
            passed += 1
        else:
            print('✗ FAIL: Limits mismatch')
            failed += 1

        # Test 3: Get joint position
        print('\n--- Test 3: Get Joint Position ---')
        rclpy.spin_once(self, timeout_sec=0.5)
        pos = self.ctrl.get_joint_position('base_main_frame_joint')
        if pos is not None:
            print(f'  base_main_frame_joint position: {pos:.4f}')
            print('✓ PASS: Position retrieved from /joint_states')
            passed += 1
        else:
            print('✗ FAIL: Could not get joint position (no /joint_states?)')
            failed += 1

        # Test 4: Command single joint (valid)
        print('\n--- Test 4: Command Single Joint (valid) ---')
        print('  Commanding base_main_frame_joint to 1.5m...')
        result = self.ctrl.command_joint('base_main_frame_joint', 1.5)
        if result:
            print('✓ PASS: command_joint returned True')
            passed += 1
        else:
            print('✗ FAIL: command_joint returned False')
            failed += 1

        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        new_pos = self.ctrl.get_joint_position('base_main_frame_joint')
        print(f'  New position: {new_pos:.4f}' if new_pos else '  New position: N/A')

        # Test 5: Command single joint (invalid - outside limits)
        print('\n--- Test 5: Command Single Joint (invalid position) ---')
        print('  Commanding base_main_frame_joint to 10.0m (outside soft limit 3.9)...')
        result = self.ctrl.command_joint('base_main_frame_joint', 10.0)
        if not result:
            print('✓ PASS: command_joint correctly returned False')
            passed += 1
        else:
            print('✗ FAIL: command_joint should have returned False')
            failed += 1

        # Test 6: Command unknown joint
        print('\n--- Test 6: Command Unknown Joint ---')
        print('  Commanding fake_joint to 1.0m...')
        result = self.ctrl.command_joint('fake_joint', 1.0)
        if not result:
            print('✓ PASS: command_joint correctly returned False for unknown joint')
            passed += 1
        else:
            print('✗ FAIL: command_joint should have returned False')
            failed += 1

        # Test 7: Command joint group (valid)
        print('\n--- Test 7: Command Joint Group (valid) ---')
        print('  Commanding base + selector frame...')
        result = self.ctrl.command_joint_group(
            ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            [2.0, 0.5]
        )
        if result:
            print('✓ PASS: command_joint_group returned True')
            passed += 1
        else:
            print('✗ FAIL: command_joint_group returned False')
            failed += 1

        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.5)

        # Test 8: Command joint group (invalid - one position out of range)
        print('\n--- Test 8: Command Joint Group (invalid position) ---')
        print('  Commanding with one invalid position (10.0 outside limits)...')
        result = self.ctrl.command_joint_group(
            ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
            [2.0, 10.0]  # 10.0 is outside selector frame limits
        )
        if not result:
            print('✓ PASS: command_joint_group correctly returned False')
            passed += 1
        else:
            print('✗ FAIL: command_joint_group should have returned False')
            failed += 1

        # Test 9: Command all 9 joints
        print('\n--- Test 9: Command All 9 Joints ---')
        all_joints = self.ctrl.get_all_joint_names()
        all_ok = True
        for joint in all_joints:
            limits = self.ctrl.get_joint_limits(joint)
            if limits:
                mid_pos = (limits[0] + limits[1]) / 2
                result = self.ctrl.command_joint(joint, mid_pos)
                status = '✓' if result else '✗'
                print(f'  {joint}: cmd={mid_pos:.3f} -> {status}')
                if not result:
                    all_ok = False
        if all_ok:
            print('✓ PASS: All 9 joints commanded successfully')
            passed += 1
        else:
            print('✗ FAIL: Some joints failed')
            failed += 1

        # Summary
        print('\n' + '=' * 60)
        print(f'TEST SUMMARY: {passed} passed, {failed} failed')
        print('=' * 60)

        return failed == 0


def main():
    rclpy.init()

    try:
        node = ControllerInterfaceTestNode()
        success = node.run_tests()
        node.destroy_node()
    except Exception as e:
        print(f'Error: {e}')
        success = False
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
