cmake_minimum_required(VERSION 3.16)
project(atd_interfaces)
# interface package は rosidl の生成処理を明示する。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg"
  "msg/OperatorEvent.msg"
  "msg/PicoTelemetry.msg"
  "msg/JoyDriveStatus.msg"
  "msg/LocalizationStatus.msg"
  "msg/PlanningStatus.msg"
  "msg/ControlStatus.msg"
  "msg/SpeedControlStatus.msg"
  "msg/TunedControlStatus.msg"
  "msg/TunedSpeedControlStatus.msg"
  "srv/SetSpeedTable.srv"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
