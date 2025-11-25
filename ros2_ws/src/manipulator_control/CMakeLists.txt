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

# Interface generation
# Placeholder for future interface files (Story 1.2 and 1.3)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/MoveJoint.action"
#   "action/MoveJointGroup.action"
#   "action/NavigateToAddress.action"
#   "action/ExtractBox.action"
#   "action/ReturnBox.action"
#   "action/PutBox.action"
#   "action/MoveBoxToLoad.action"
#   "action/ManipulateContainer.action"
#   "action/GetContainer.action"
#   "action/PlaceContainer.action"
#   "action/PickItem.action"
#   "action/PickItemFromStorage.action"
#   "srv/GetAddressCoordinates.srv"
#   "srv/ToggleElectromagnet.srv"
#   "srv/SpawnBox.srv"
#   "srv/DespawnBox.srv"
#   "srv/ValidateAddress.srv"
#   "msg/Address.msg"
#   "msg/JointCommand.msg"
#   "msg/LimitSwitchState.msg"
#   DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
# )

# Install directories (for future use)
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
