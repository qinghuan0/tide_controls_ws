cmake_minimum_required(VERSION 3.10)
project(tide_ctrl_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install directories
install(
  DIRECTORY
    config
    launch
    description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
