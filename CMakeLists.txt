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

rosidl_generate_interfaces(casah_msgs
  "msgs/CommanderEnableCmd.msg"
  "msgs/CommanderState.msg"
  "msgs/EgdCspCmd.msg"
  "msgs/EgdResetCmd.msg"
  "msgs/EgdState.msg"
  "msgs/El2124WriteAllChannelsCmd.msg"
  "msgs/El2124WriteChannelCmd.msg"
  "msgs/FunctionState.msg"
  #"srv/TareForceTorqueSensor.srv"
  #"action/CommandActuatorPosition.action"
  DEPENDENCIES builtin_interfaces #action_msgs
  )

ament_package()
