cmake_minimum_required(VERSION 3.8)
project(hamal_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

set(
  msg_files
  "msg/HardwareInformation.msg"
  "msg/HardwareInformationArray.msg"
  "msg/HomingInformation.msg"
  "msg/LifterCommand.msg"
  "msg/MotionProfileCommand.msg"
  "msg/MotionReference.msg"
  "msg/SpeedLimitRatio.msg"
  "msg/PathInfo.msg"
)

set(
  srv_files
  "srv/EmergencyStop.srv"
)

set(
  action_files
  "action/HomingOperation.action"
  "action/LifterOperation.action"
  "action/MotionProfileOperation.action"
  "action/GeneratePath.action"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  std_msgs
  builtin_interfaces
  geometry_msgs
  nav_msgs
)
ament_package()
