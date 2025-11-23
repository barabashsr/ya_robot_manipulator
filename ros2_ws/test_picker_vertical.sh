#!/bin/bash
# Test script for picker vertical (D-Pad Y / Axis 7)

echo "=== Testing Picker Vertical Control ==="
echo ""
echo "1. Checking if commands are being published..."
timeout 2 ros2 topic hz /selector_frame_picker_frame_joint_controller/commands 2>&1 | head -5
echo ""

echo "2. Monitoring commands for 5 seconds (hold L1 and press D-Pad UP/DOWN)..."
timeout 5 ros2 topic echo /selector_frame_picker_frame_joint_controller/commands

echo ""
echo "3. Current joint position:"
ros2 topic echo /joint_states --once 2>/dev/null | grep -A 8 "^name:" | grep -E "selector_frame_picker|position" | head -15
