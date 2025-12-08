cmake_minimum_required(VERSION 3.8)
project(manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install Python package for importable modules (AC10 - testability)
# Uses different package name to avoid conflict with rosidl_generator_py
ament_python_install_package(manipulator_utils)

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJoint.action"
  "action/MoveJointGroup.action"
  "action/NavigateToAddress.action"
  "action/ExtractBox.action"
  "action/ReturnBox.action"
  "action/PutBox.action"
  "action/MoveBoxToLoad.action"
  "action/ManipulateContainer.action"
  "action/GetContainer.action"
  "action/PlaceContainer.action"
  "action/PickItem.action"
  "action/PickItemFromStorage.action"
  # Services (Story 1.3)
  "srv/GetAddressCoordinates.srv"
  "srv/ToggleElectromagnet.srv"
  "srv/SpawnBox.srv"
  "srv/DespawnBox.srv"
  "srv/ValidateAddress.srv"
  # Messages (Story 1.3)
  "msg/Address.msg"
  "msg/JointCommand.msg"
  "msg/LimitSwitchState.msg"
  DEPENDENCIES geometry_msgs
)

# Install Python scripts
install(
  PROGRAMS
    src/virtual_limit_switches.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME virtual_limit_switches
)

# Install MoveJoint action server (Story 2.3)
install(
  PROGRAMS
    src/move_joint_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_joint_server
)

# Install State Marker Publisher (Story 2.4)
install(
  PROGRAMS
    src/state_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME state_marker_publisher
)

# Install MoveJointGroup action server (Story 2.5)
install(
  PROGRAMS
    src/move_joint_group_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_joint_group_server
)

# Install Address Service Node (Story 3.2)
install(
  PROGRAMS
    manipulator_control/address_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME address_service_node
)

# Install NavigateToAddress action server (Story 3.4)
install(
  PROGRAMS
    src/navigate_to_address_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigate_to_address_server
)

# Install Electromagnet Simulator Node (Story 4A.2)
install(
  PROGRAMS
    src/electromagnet_simulator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME electromagnet_simulator_node
)

# Install Python modules (utility classes)
install(
  PROGRAMS
    src/controller_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test scripts (Story 2.3)
install(
  PROGRAMS
    scripts/test_move_joint_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_move_joint_action
)

# Install directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
