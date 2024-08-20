cmake_minimum_required(VERSION 3.8)
project(hamal_hardware)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
  realtime_tools
  hamal_custom_interfaces
  std_msgs
)

set(ethercat_interface_lib /usr/local/lib/libethercat_interface.so)
#set(ethercat_interface_lib_INCLUDE_DIRS /home/hamal22/ethercat_interface/include/)
set(ethercat_interface_lib_INCLUDE_DIRS /home/hamal22/ethercat_interface/include/)
set(etherlab /usr/local/lib/libethercat.so)

find_package(ament_cmake REQUIRED)
foreach(depends IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${depends} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hamal_hardware.cpp
  src/ethercat_handler.cpp
  src/homing_helper.cpp
  src/hardware_interface_node.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
  ${ethercat_interface_lib_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} 
  ${ethercat_interface_lib}
  ${etherlab}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface hamal_hardware.xml)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
