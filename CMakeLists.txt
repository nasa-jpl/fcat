cmake_minimum_required(VERSION 3.5)
project(fcat_msgs)

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

set(msg_files
  "msg/ActuatorCalibrateCmd.msg"
  "msg/ActuatorCspCmd.msg"
  "msg/ActuatorCstCmd.msg"
  "msg/ActuatorCsvCmd.msg"
  "msg/ActuatorProfPosCmd.msg"
  "msg/ActuatorProfTorqueCmd.msg"
  "msg/ActuatorProfVelCmd.msg"
  "msg/ActuatorSetOutputPositionCmd.msg"
  "msg/ActuatorState.msg"
  "msg/CommanderEnableCmd.msg"
  "msg/CommanderDisableCmd.msg"
  "msg/CommanderState.msg"
  "msg/EgdCspCmd.msg"
  "msg/EgdCstCmd.msg"
  "msg/EgdCsvCmd.msg"
  "msg/EgdState.msg"
  "msg/El2124State.msg"
  "msg/El2124WriteAllChannelsCmd.msg"
  "msg/El2124WriteChannelCmd.msg"
  "msg/El3208State.msg"
  "msg/El3602State.msg"
  "msg/FaulterEnableCmd.msg"
  "msg/FaulterState.msg"
  "msg/FtsTareCmd.msg"
  "msg/JedSetCmdValueCmd.msg"
  "msg/JedState.msg"
  "msg/ModuleState.msg"
  "msg/PidActivateCmd.msg"
  "msg/PidState.msg"
  "msg/SignalGeneratorState.msg"
  "msg/FunctionState.msg"
  "msg/ConditionalState.msg"
  "msg/SchmittTriggerState.msg"
  "msg/SaturationState.msg"
  "msg/FilterState.msg"
  )


rosidl_generate_interfaces(fcat_msgs
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  )
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION include/${PROJECT_NAME}
#    )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
