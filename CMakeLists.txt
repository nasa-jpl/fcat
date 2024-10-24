cmake_minimum_required(VERSION 3.11)
project(fcat_msgs
  VERSION 0.9.14
  DESCRIPTION "ROS FCAT Messages"
  LANGUAGES C CXX
)

find_package(CfwCmake)
cfw_add_target_tag(
    ARTIFACTORY_ARTIFACTS
      "build/fcat_msgs*.tar.gz"
)
cfw_get_project_default_options()

# Define the files that contain all the messages and services
set(msg_files
  "msg/ActuatorCalibrateCmd.msg"
  "msg/ActuatorCspCmd.msg"
  "msg/ActuatorCspCmds.msg"
  "msg/ActuatorCstCmd.msg"
  "msg/ActuatorCsvCmd.msg"
  "msg/ActuatorHaltCmd.msg"
  "msg/ActuatorHaltCmds.msg"
  "msg/ActuatorProfPosCmd.msg"
  "msg/ActuatorProfPosCmds.msg"
  "msg/ActuatorProfTorqueCmd.msg"
  "msg/ActuatorProfVelCmd.msg"
  "msg/ActuatorSetOutputPositionCmd.msg"
  "msg/ActuatorSetDigitalOutputCmd.msg"
  "msg/ActuatorSetMaxCurrentCmd.msg"
  "msg/ActuatorSetUnitModeCmd.msg"
  "msg/ActuatorState.msg"
  "msg/ActuatorStates.msg"
  "msg/PlatinumActuatorState.msg"
  "msg/PlatinumActuatorStates.msg"
  "msg/ActuatorSetGainSchedulingModeCmd.msg"
  "msg/ActuatorSetGainSchedulingModeCmds.msg"
  "msg/ActuatorSetGainSchedulingIndexCmd.msg"
  "msg/ActuatorSetGainSchedulingIndexCmds.msg"
  "msg/ActuatorSetProfDisengagingTimeoutCmd.msg"
  "msg/AsyncSdoWriteCmd.msg"
  "msg/AsyncSdoReadCmd.msg"
  "msg/AsyncSdoResponse.msg"
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
  "msg/El1008State.msg"
  "msg/El1008States.msg"
  "msg/El2124State.msg"
  "msg/El2124States.msg"
  "msg/El2124WriteAllChannelsCmd.msg"
  "msg/El2124WriteChannelCmd.msg"
  "msg/El2809State.msg"
  "msg/El2809States.msg"
  "msg/El2809WriteAllChannelsCmd.msg"
  "msg/El2809WriteChannelCmd.msg"
  "msg/El2828State.msg"
  "msg/El2828States.msg"
  "msg/El2828WriteAllChannelsCmd.msg"
  "msg/El2828WriteChannelCmd.msg"
  "msg/El2798State.msg"
  "msg/El2798States.msg"
  "msg/El2798WriteAllChannelsCmd.msg"
  "msg/El2798WriteChannelCmd.msg"
  "msg/El4102State.msg"
  "msg/El4102States.msg"
  "msg/El4102WriteAllChannelsCmd.msg"
  "msg/El4102WriteChannelCmd.msg"
  "msg/El3208State.msg"
  "msg/El3208States.msg"
  "msg/El3602State.msg"
  "msg/El3602States.msg"
  "msg/El3162State.msg"
  "msg/El3162States.msg"
  "msg/El3104State.msg"
  "msg/El3104States.msg"
  "msg/El3202State.msg"
  "msg/El3202States.msg"
  "msg/El3318State.msg"
  "msg/El3318States.msg"
  "msg/FaulterEnableCmd.msg"
  "msg/FaulterState.msg"
  "msg/FaulterStates.msg"
  "msg/FbcState.msg"
  "msg/FbcStates.msg"
  "msg/FbcRequestDataCmd.msg"
  "msg/FbcTriggerEstopCmd.msg"
  "msg/FbcResetFaultCmd.msg"
  "msg/FilterState.msg"
  "msg/FilterStates.msg"
  "msg/FtsState.msg"
  "msg/FtsStates.msg"
  "msg/FtsTareCmd.msg"
  "msg/FunctionState.msg"
  "msg/FunctionStates.msg"
  "msg/Ild1900State.msg"
  "msg/Ild1900States.msg"
  "msg/JedSetCmdValueCmd.msg"
  "msg/JedState.msg"
  "msg/JedStates.msg"
  "msg/LinearInterpolationState.msg"
  "msg/LinearInterpolationStates.msg"
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
  "msg/JcbState.msg"
  "msg/JcbStates.msg"
  "msg/ThreeNodeThermalModelState.msg"
  "msg/ThreeNodeThermalModelStates.msg"
  )

set(srv_files
  "srv/ActuatorCalibrateService.srv"
  "srv/ActuatorProfPosService.srv"
  "srv/ActuatorProfTorqueService.srv"
  "srv/ActuatorProfVelService.srv"
  "srv/ActuatorSetGainSchedulingModeService.srv"
  "srv/ActuatorSetUnitModeService.srv"
  "srv/PidActivateService.srv"
  "srv/AsyncSdoWriteService.srv"
  "srv/AsyncSdoReadService.srv"
  "srv/TlcWriteService.srv"
  "srv/TlcReadService.srv"
  "srv/ActuatorHaltService.srv"
  "srv/ActuatorSetGainSchedulingIndexService.srv"
  "srv/ActuatorSetDigitalOutputService.srv"
  "srv/ActuatorSetMaxCurrentService.srv"
  "srv/ActuatorSetOutputPositionService.srv"
  "srv/ActuatorSetProfDisengagingTimeoutService.srv"
  "srv/CommanderEnableService.srv"
  "srv/CommanderDisableService.srv"
  "srv/El2124WriteAllChannelsService.srv"
  "srv/El2124WriteChannelService.srv"
  "srv/El2809WriteAllChannelsService.srv"
  "srv/El2809WriteChannelService.srv"
  "srv/El2828WriteAllChannelsService.srv"
  "srv/El2828WriteChannelService.srv"
  "srv/El2798WriteAllChannelsService.srv"
  "srv/El2798WriteChannelService.srv"
  "srv/El4102WriteAllChannelsService.srv"
  "srv/El4102WriteChannelService.srv"
  "srv/FaulterEnableService.srv"
  "srv/DeviceTriggerService.srv"
  )

# Check if ament_cmake is present, indicating ROS2
find_package(ament_cmake QUIET)

# Run build system for ROS2 if ament_cmake was found
if(ament_cmake_FOUND)
  find_package(builtin_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(fcat_msgs
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces std_msgs
    )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()

# Else, we must be using catkin and ROS1
else()

  unset(CMAKE_PREFIX_PATH)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
  )

  foreach(FILE ${msg_files})
    string(REPLACE "msg/" "" new_msg_file ${FILE})
    set(ros1_msg_files ${ros1_msg_files} ${new_msg_file})
  endforeach()

  add_message_files(
    FILES
    ${ros1_msg_files}
  )

  foreach(FILE ${srv_files})
    string(REPLACE "srv/" "" new_srv_file ${FILE})
    set(ros1_srv_files ${ros1_srv_files} ${new_srv_file})
  endforeach()

  add_service_files(
    FILES
    ${ros1_srv_files}
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
    )

  catkin_package()
endif()


