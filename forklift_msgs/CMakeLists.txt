cmake_minimum_required(VERSION 3.8)
project(forklift_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DetectPalletOffset.action"
  "action/ExecuteRoute.action"
  "action/ExecutePalletPickup.action"
  "action/ForkMoveTo.action"
  "action/MoveRelative.action"
  "msg/ForkliftControlCommand.msg"
  "msg/ForkliftVehicleState.msg"
  "msg/ForkliftFaultState.msg"
  "msg/ForkliftIoState.msg"
  "msg/TaskStatus.msg"
  "srv/GoToStation.srv"
  "srv/SetControlMode.srv"
  "srv/SetEmergencyStop.srv"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
