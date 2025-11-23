#!/bin/bash
# Comprehensive controller test script

echo "=========================================="
echo "CONTROLLER DIAGNOSTICS"
echo "=========================================="
echo ""

echo "1. Controller Status:"
echo "-------------------"
ros2 control list_controllers
echo ""

echo "2. Hardware Interfaces:"
echo "----------------------"
ros2 control list_hardware_interfaces | grep "command interfaces" -A 20
echo ""

echo "3. Command Topics (checking if they receive messages):"
echo "------------------------------------------------------"

declare -a controllers=(
    "base_main_frame_joint_controller"
    "main_frame_selector_frame_joint_controller"
    "selector_frame_gripper_joint_controller"
    "selector_left_container_jaw_joint_controller"
    "selector_right_container_jaw_joint_controller"
    "selector_frame_picker_frame_joint_controller"
    "picker_frame_picker_rail_joint_controller"
    "picker_rail_picker_base_joint_controller"
    "picker_base_picker_jaw_joint_controller"
)

for controller in "${controllers[@]}"; do
    echo -n "$controller: "
    timeout 1 ros2 topic echo /${controller}/commands --once 2>/dev/null >/dev/null
    if [ $? -eq 0 ]; then
        echo "✓ RECEIVING COMMANDS"
    else
        echo "✗ NO COMMANDS"
    fi
done

echo ""
echo "4. Current Joint Positions:"
echo "--------------------------"
ros2 topic echo /joint_states --once 2>/dev/null | grep -E "^name:|^position:" | head -20

echo ""
echo "=========================================="
echo "To monitor a specific controller in real-time:"
echo "  ros2 topic echo /CONTROLLER_NAME/commands"
echo ""
echo "To manually send a test command:"
echo "  ros2 topic pub --once /CONTROLLER_NAME/commands std_msgs/msg/Float64MultiArray \"{data: [0.5]}\""
echo "=========================================="
