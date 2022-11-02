cmake_minimum_required(VERSION 3.3)
project(hoverboard_driver)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-std=c++11)

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

#generate_dynamic_reconfigure_options(
#  config/Hoverboard.cfg
#)

#catkin_package(
#  INCLUDE_DIRS include
#)

include_directories(
 include
 include/hoverboard_driver
 ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

add_executable(hoverboard_driver 
  src/hoverboard.cpp 
  src/pid.cpp
  src/main.cpp
)
target_link_libraries(hoverboard_driver ${Boost_LIBRARIES})
ament_target_dependencies(hoverboard_driver
  rclcpp
  control_toolbox
  controller_manager
  hardware_interface
)
install(TARGETS hoverboard_driver DESTINATION lib/${PROJECT_NAME})

#add_dependencies(hoverboard_driver ${PROJECT_NAME}_gencfg)

ament_package()
