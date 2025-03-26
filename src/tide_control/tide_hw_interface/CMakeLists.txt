cmake_minimum_required(VERSION 3.10)
project(tide_hw_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build the hardware interface library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tide_motor.cpp
  src/socket_can/socket_can_common.cpp
  src/socket_can/socket_can_id.cpp
  src/socket_can/socket_can_receiver.cpp
  src/socket_can/socket_can_sender.cpp
  src/tide_hw_interface.cpp
)

pluginlib_export_plugin_description_file(hardware_interface tide_hw_interface.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager
)

ament_auto_package()
