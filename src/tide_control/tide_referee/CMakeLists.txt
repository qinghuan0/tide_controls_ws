cmake_minimum_required(VERSION 3.8)
project(tide_referee)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(tide_msgs REQUIRED)
ament_auto_find_build_dependencies()


# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN tide_serial_driver::tide_referee::TideReferee
  EXECUTABLE tide_referee_node
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  tide_msgs
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)
