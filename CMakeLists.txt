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
  "msg/ActuatorStates.msg"
  "msg/CommanderDisableCmd.msg"
  "msg/CommanderEnableCmd.msg"
  "msg/CommanderState.msg"
  "msg/CommanderStates.msg"
  "msg/ConditionalState.msg"
  "msg/ConditionalStates.msg"
  "msg/EgdCspCmd.msg"
  "msg/EgdCstCmd.msg"
  "msg/EgdCsvCmd.msg"
  "msg/EgdState.msg"
  "msg/EgdStates.msg"
  "msg/El2124State.msg"
  "msg/El2124States.msg"
  "msg/El2124WriteAllChannelsCmd.msg"
  "msg/El2124WriteChannelCmd.msg"
  "msg/El3208State.msg"
  "msg/El3208States.msg"
  "msg/El3602State.msg"
  "msg/El3602States.msg"
  "msg/FaulterEnableCmd.msg"
  "msg/FaulterState.msg"
  "msg/FaulterStates.msg"
  "msg/FilterState.msg"
  "msg/FilterStates.msg"
  "msg/FtsTareCmd.msg"
  "msg/FunctionState.msg"
  "msg/FunctionStates.msg"
  "msg/JedSetCmdValueCmd.msg"
  "msg/JedState.msg"
  "msg/JedStates.msg"
  "msg/ModuleState.msg"
  "msg/PidActivateCmd.msg"
  "msg/PidState.msg"
  "msg/PidStates.msg"
  "msg/SaturationState.msg"
  "msg/SaturationStates.msg"
  "msg/SchmittTriggerState.msg"
  "msg/SchmittTriggerStates.msg"
  "msg/SignalGeneratorState.msg"
  "msg/SignalGeneratorStates.msg"
  )

set(srv_files
  "srv/ActuatorCalibrateCmd.srv"
  "srv/ActuatorProfPosCmd.srv"
  "srv/ActuatorProfTorqueCmd.srv"
  "srv/ActuatorProfVelCmd.srv"
  "srv/PidActivateCmd.srv"
  )


rosidl_generate_interfaces(fcat_msgs
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
  )
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION include/${PROJECT_NAME}
#    )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
