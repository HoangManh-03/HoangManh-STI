cmake_minimum_required(VERSION 3.8)
project(message_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Add this line
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "msg/Cansend.msg"
  "msg/Canreceived.msg"
  "msg/StatusPort.msg"
  "msg/StatusLaunch.msg"
  "msg/HcInfo.msg"
  "msg/HcInfo2.msg"
  "msg/HcRequest.msg"
  "msg/FieldSafetyRequest.msg"
  "msg/PowerInfo.msg"
  "msg/PowerRequest.msg"
  "msg/LiftRequest.msg"
  "msg/LiftStatus.msg"
  "msg/ConveyorStatus.msg"
  "msg/ConveyorControl.msg"
  "msg/McInfo.msg"
  "msg/McRequest.msg"
  "msg/RFID.msg"
  "msg/MagneticLine.msg"
  "msg/NNcmdRequest.msg"
  "msg/NNinfoRequest.msg"
  "msg/NNinfoRespond.msg"
  "msg/MoveRequest.msg"
  "msg/MoveRespond.msg"
  "msg/StatusReconnect.msg"
  "msg/ReconnectSts.msg"
  "msg/AppButton.msg"
  "msg/ZoneLidar2Head.msg"
  "msg/LiftControl.msg"
  "msg/DriverRespond.msg"
  "msg/AppButtonAgvmag.msg"
  "msg/McuInfo.msg"
  "msg/HcuInfo.msg"
  "msg/NucInfo.msg"
  "msg/PsuInfo.msg"
  "msg/LineControllerRequest.msg"
  "msg/LineControllerRespond.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}

  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
