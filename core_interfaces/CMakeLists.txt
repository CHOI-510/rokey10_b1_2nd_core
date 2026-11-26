cmake_minimum_required(VERSION 3.8)
project(core_interfaces)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  "msg/ControlStatus.msg"
  "msg/RobotStatus.msg"
  "msg/LogEvent.msg"
  "msg/TaskResult.msg"
  "msg/Announcement.msg"
  "srv/Inspect.srv"
  "srv/Locate.srv"
  "srv/JobCommand.srv"
  "srv/ParseCommand.srv"
  "srv/Summarize.srv"
  "srv/EmergencyStop.srv"
  "action/MoveTo.action"
  "action/Fetch.action"
  "action/Stow.action"
  "action/Fasten.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
