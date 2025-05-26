cmake_minimum_required(VERSION 3.8)
project(ib2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare message files
set(msg_files
  "msg/AliveStatus.msg"
  "msg/BatteryChargeInfo.msg"
  "msg/CtlProfile.msg"
  "msg/CtlStatus.msg"
  "msg/CtlStatusType.msg"
  "msg/DiskSpace.msg"
  "msg/DisplayManagerStatus.msg"
  "msg/FanStatus.msg"
  "msg/FileMonitoringStatus.msg"
  "msg/IMU.msg"
  "msg/LEDColors.msg"
  "msg/MainCameraResolutionType.msg"
  "msg/MainCameraStatus.msg"
  "msg/MainCameraWhiteBalanceMode.msg"
  "msg/Marker.msg"
  "msg/Mode.msg"
  "msg/MonitoredAliveStatuses.msg"
  "msg/Navigation.msg"
  "msg/NavigationDebug.msg"
  "msg/NavigationStatus.msg"
  "msg/PowerStatus.msg"
  "msg/RosParam.msg"
  "msg/RosParams.msg"
  "msg/Slam.msg"
  "msg/SystemStatus.msg"
)

# Declare service files
set(srv_files
  "srv/Dock.srv"
  "srv/DumpRosParams.srv"
  "srv/ExitDockingMode.srv"
  "srv/GetRosParam.srv"
  "srv/GetRosParams.srv"
  "srv/LoadRosParams.srv"
  "srv/MarkerCorrection.srv"
  "srv/Record.srv"
  "srv/SetMaintenanceMode.srv"
  "srv/SetRosParam.srv"
  "srv/SetRosParams.srv"
  "srv/SwitchPower.srv"
  "srv/UpdateParameter.srv"
)

# Declare action files
set(action_files
  "action/CtlCommand.action"
  "action/NavigationStartUp.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs action_msgs example_interfaces builtin_interfaces
  ADD_LINTER_TESTS
)

# Package dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
