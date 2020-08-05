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
  "srv/AllDeviceFaultCmd.srv"
  "srv/AllDeviceResetCmd.srv"
  "srv/CommanderEnableCmd.srv"
  "srv/EgdCspCmd.srv"
  "srv/EgdCsvCmd.srv"
  "srv/EgdCstCmd.srv"
  "srv/EgdProfPosCmd.srv"
  "srv/EgdProfVelCmd.srv"
  "srv/EgdProfTorqueCmd.srv"
  "srv/EgdResetCmd.srv"
  "srv/EgdHaltCmd.srv"
  "srv/EgdSdoSetDrivePosCmd.srv"
  "srv/EgdSdoSetUnitModeCmd.srv"
  "srv/FaulterEnableCmd.srv"
  "srv/FtsTareCmd.srv"
  "srv/El2124WriteAllChannelsCmd.srv"
  "srv/El2124WriteChannelCmd.srv"
  "srv/ActuatorCspCmd.srv"
  "srv/ActuatorCsvCmd.srv"
  "srv/ActuatorCstCmd.srv"
  "srv/ActuatorProfPosCmd.srv"
  "srv/ActuatorProfVelCmd.srv"
  "srv/ActuatorProfTorqueCmd.srv"
  "srv/ActuatorResetCmd.srv"
  "srv/ActuatorHaltCmd.srv"
  "srv/ActuatorSdoSetDrivePosCmd.srv"
  "srv/ActuatorSdoSetUnitModeCmd.srv"
  "srv/StateFormatCmd.srv"
  "msg/SignalGeneratorState.msg"
  "msg/FunctionState.msg"
  "msg/CommanderState.msg"
  "msg/ConditionalState.msg"
  "msg/SchmittTriggerState.msg"
  "msg/FaulterState.msg"
  "msg/FilterState.msg"
  "msg/FtsState.msg"
  "msg/EgdState.msg"
  "msg/El3208State.msg"
  "msg/El3602State.msg"
  "msg/El2124State.msg"
  "msg/ActuatorState.msg"
  "msg/VirtualFtsState.msg"
  "msg/ModuleState.msg"
  "msg/MegaState.msg"
  #"srv/TareForceTorqueSensor.srv"
  #"action/CommandActuatorPosition.action"
  DEPENDENCIES builtin_interfaces #action_msgs
  )

ament_package()
