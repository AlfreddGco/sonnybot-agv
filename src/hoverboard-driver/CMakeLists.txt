cmake_minimum_required(VERSION 3.3)
project(hoverboard_driver)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)
find_package(control_toolbox REQUIRED)
#find_package(rosparam_shortcuts REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
 include
 include/hoverboard_driver
 ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config)

add_executable(hoverboard_driver 
  src/hoverboard.cpp 
  src/pid.cpp
  src/main.cpp
)
target_link_libraries(hoverboard_driver ${Boost_LIBRARIES})
ament_target_dependencies(hoverboard_driver
  rclcpp
  pluginlib
  control_toolbox
  controller_manager
  hardware_interface
)
pluginlib_export_plugin_description_file(hardware_interface config/hardware.yaml)

install(TARGETS hoverboard_driver DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ros2_controllers
)
ament_export_dependencies(
  rclcpp
  pluginlib
  control_toolbox
  controller_manager
  hardware_interface
)

ament_package()
