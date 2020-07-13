cmake_minimum_required(VERSION 3.5)
project(fcat_msgs_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(fcat_msgs_ros2
  "msg/CommanderEnableCmd.msg"
  "msg/EgdCspCmd.msg"
  "msg/EgdCsvCmd.msg"
  "msg/EgdCstCmd.msg"
  "msg/EgdProfPosCmd.msg"
  "msg/EgdProfVelCmd.msg"
  "msg/EgdProfTorqueCmd.msg"
  "msg/EgdResetCmd.msg"
  "msg/EgdHaltCmd.msg"
  "msg/EgdSdoSetDrivePosCmd.msg"
  "msg/EgdSdoSetUnitModeCmd.msg"
  "msg/FtsTareCmd.msg"
  "msg/El2124WriteAllChannelsCmd.msg"
  "msg/El2124WriteChannelCmd.msg"
  "msg/CommanderState.msg"
  "msg/ActuatorCspCmd.msg"
  "msg/ActuatorCsvCmd.msg"
  "msg/ActuatorCstCmd.msg"
  "msg/ActuatorProfPosCmd.msg"
  "msg/ActuatorProfVelCmd.msg"
  "msg/ActuatorProfTorqueCmd.msg"
  "msg/ActuatorResetCmd.msg"
  "msg/ActuatorHaltCmd.msg"
  "msg/ActuatorSdoSetDrivePosCmd.msg"
  "msg/ActuatorSdoSetUnitModeCmd.msg"
  "msg/SignalGeneratorState.msg"
  "msg/FunctionState.msg"
  "msg/ConditionalState.msg"
  "msg/SchmittTriggerState.msg"
  "msg/FilterState.msg"
  "msg/FtsState.msg"
  "msg/EgdState.msg"
  "msg/El3208State.msg"
  "msg/El3602State.msg"
  "msg/El2124State.msg"
  "msg/ActuatorState.msg"
  "msg/VirtualFtsState.msg"
  "msg/GlobalState.msg"
  #"srv/TareForceTorqueSensor.srv"
  #"action/CommandActuatorPosition.action"
  DEPENDENCIES builtin_interfaces #action_msgs
  )

ament_package()
