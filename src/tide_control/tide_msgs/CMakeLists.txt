cmake_minimum_required(VERSION 3.8)
project(tide_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalState.msg"
  "msg/GimbalCmd.msg"
  "msg/ShooterState.msg"
  "msg/ShooterCmd.msg"
  "msg/BigyawState.msg"
  "msg/BigyawCmd.msg"
  "msg/FdFrame.msg"
  "msg/vision/Armor.msg"
  "msg/vision/Armors.msg"
  "msg/vision/DebugArmor.msg"
  "msg/vision/DebugArmors.msg"
  "msg/vision/DebugLight.msg"
  "msg/vision/DebugLights.msg"
  "msg/vision/Target.msg"
  "msg/vision/TimeInfo.msg"
  "msg/vision/TrackerInfo.msg"
  "msg/referee/Buff.msg"
  "msg/referee/BulletRemaining.msg"
  "msg/referee/EventData.msg"
  "msg/referee/GameResult.msg"
  "msg/referee/GameRobotHP.msg"
  "msg/referee/GameRobotPos.msg"
  "msg/referee/GameRobotStatus.msg"
  "msg/referee/GameStatus.msg"
  "msg/referee/GroundRobot.msg"
  "msg/referee/InteractID.msg"
  "msg/referee/PowerHeatData.msg"
  "msg/referee/RefereeData.msg"
  "msg/referee/RefereeWarning.msg"
  "msg/referee/RFIDStatus.msg"
  "msg/referee/RobotHurt.msg"
  "msg/referee/SentryInfo.msg"
  "msg/referee/ShootData.msg"

  DEPENDENCIES
  std_msgs
  geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
