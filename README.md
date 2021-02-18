# fcat

ROS2 Fastcat node for running EtherCAT Devices

`fcat` is a ROS2 wrapper around the C++ `fastcat` library. This module will published variable-sized arrays of state information and subscribe to ROS Topics to delegate commands down to the EtherCAT devices. 

Some logic checking is performed on the EtherCAT bus topology to only create state topics and commands subscriptions for device types on the bus. 

Cursory checking for device names is performed, if a command for non-existent device is issued, `fcat` will ignore the command and log a message to STDOUT - No indication the device name is not found will be sent to the application.

Configuration of `fcat` is delegated almost completely to the input fastcat configuration file. 

An optional ROS2 node called `fcat_srvs` is provided in this package. The purpose of this node is to provide a light-weight blocking service interface to make it easier to script long-running behaviors with a sequencing tool like `commander`. 

## Quickstart

**fcat**

```bash
ros2 run fcat fcat --ros-args -p fastcat_config_path:=[FILE_PATH]/[CONFIG].yaml
```

Optionally, the default namespace `"fcat"` and default node name `"fcat"`  can be remapped like so:

``` bash
ros2 run fcat fcat --ros-args -p fastcat_config_path:=[FILE_PATH]/[CONFIG].yaml -r __ns:=/foo -r __node:=bar
```

**fcat_srvs**

``` bash
ros2 run fcat fcat_srvs
```



# The `fcat` Node

## Parameters

| Name                   | Description                                                  | Default |
| ---------------------- | ------------------------------------------------------------ | ------- |
| fastcat_config_path    | The path  of Fastcat input YAML bus configuration            | ""      |
| create_joint_state_pub | Enables publishing of `state/joint_states` topic useful for RVIZ2 or Moveit2! | true    |

## Topics

### Publisher

Device publishers are only created if at least one device is found on the user-specified Fastcat configuration file. `module_state` is the only state topic guaranteed to be published so this signal is suitable for liveliness checks.

| Type                            | Default Name                    | Description                                            |
| ------------------------------- | ------------------------------- | ------------------------------------------------------ |
| fcat_msgs/ModuleState           | `/fcat/state/module_state`      | Module status of Fastcat Bus                           |
| fcat_msgs/ActuatorStates        | `/fcat/state/actuators`         | Variable-sized Array of ActuatorState telemetry        |
| fcat_msgs/EgdStates             | `/fcat/state/egds`              | Variable-sized Array of EgdState Telemetry             |
| fcat_msgs/El2124States          | `/fcat/state/el2124s`           | Variable-sized Array of El2124State Telemetry          |
| fcat_msgs/El3208States          | `/fcat/state/el3208s`           | Variable-sized Array of El3208State Telemetry          |
| fcat_msgs/El3602States          | `/fcat/state/el3602s`           | Variable-sized Array of El3602State Telemetry          |
| fcat_msgs/JedStates             | `/fcat/state/jeds`              | Variable-sized Array of JedState Telemetry             |
| fcat_msgs/CommanderStates       | `/fcat/state/commanders`        | Variable-sized Array of CommanderState Telemetry       |
| fcat_msgs/ConditionalStates     | `/fcat/state/conditionals`      | Variable-sized Array of ConditionalState Telemetry     |
| fcat_msgs/FaulterStates         | `/fcat/state/faulters`          | Variable-sized Array of FaulterState Telemetry         |
| fcat_msgs/FilterStates          | `/fcat/state/filters`           | Variable-sized Array of FilterState Telemetry          |
| fcat_msgs/FunctionStates        | `/fcat/state/functions`         | Variable-sized Array of FunctionState Telemetry        |
| fcat_msgs/PidStates             | `/fcat/state/pids`              | Variable-sized Array of PidState Telemetry             |
| fcat_msgs/SaturationStates      | `/fcat/state/saturations`       | Variable-sized Array of SaturationState Telemetry      |
| fcat_msgs/SchmittTriggerStates  | `/fcat/state/schmitt_triggers`  | Variable-sized Array of SchmittTriggerState Telemetry  |
| fcat_msgs/SignalGeneratorStates | `/fcat/state/signal_generators` | Variable-sized Array of SignalGeneratorState Telemetry |

### Subscribers

Device subscribers are only created if at least one device is found on the user-specified Fastcat configuration file. Reset and Fault are the only guaranteed command topics.

| Type                                   | Default Name                            | Description                                                  |
| -------------------------------------- | --------------------------------------- | ------------------------------------------------------------ |
| std_msgs/Empty                         | `fcat/cmd/reset`                        | Global reset for all devices on bus. Always available.       |
| std_msgs/Empty                         | `fcat/cmd/fault`                        | Global fault for all devices on bus. Always available.       |
| fcat_msgs/ActuatorCspCmd               | `fcat/cmd/actuator_csp`                 | Cyclic Synchronous Position Command, should be sent every loop period |
| fcat_msgs/ActuatorCsvCmd               | `fcat/cmd/actuator_csv`                 | Cyclic Synchronous Velocity Command, should be sent every loop period |
| fcat_msgs/ActuatorCstCmd               | `fcat/cmd/actuator_cst`                 | Cyclic Synchronous Torque Command, should be sent every loop period |
| fcat_msgs/ActuatorProfPosCmd           | `fcat/cmd/actuator_prof_pos`            | Trap Profiled Position Command                               |
| fcat_msgs/ActuatorProfVelCmd           | `fcat/cmd/actuator_prof_vel`            | Trap Profiled Velocity Command                               |
| fcat_msgs/ActuatorProfTorqueCmd        | `fcat/cmd/actuator_prof_torque`         | Trap Profiled Torque Command                                 |
| fcat_msgs/ActuatorSetOutputPositionCmd | `fcat/cmd/actuator_set_output_position` | Sets the Current Actuator Output Position                    |
| fcat_msgs/ActuatorCalibrate            | `fcat/cmd/actuator_calibrate`           | Performs Hardstop calibration                                |
| fcat_msgs/CommanderEnableCmd           | `fcat/cmd/commander_enable`             | Enables a commander for Fastcat internal functions           |
| fcat_msgs/CommanderDisableCmd          | `fcat/cmd/commander_disable`            | Disables an active Commander                                 |
| fcat_msgs/El2124WriteAllChannelsCmd    | `fcat/cmd/el2124_write_all_channels`    | Writes All Digital Outputs                                   |
| fcat_msgs/El2124WriteChannelCmd        | `fcat/cmd/el2124_write_channel`         | Writes Single Digital output Channel                         |
| fcat_msgs/FaulterEnableCmd             | `fcat/cmd/faulter_enable`               | Enables or Disables Faulter device, typically used for monitoring testbed-specific fault conditions |
| fcat_msgs/FtsTareCmd                   | `fcat/cmd/fts_tare`                     | Tares a FTS                                                  |
| fcat_msgs/JedSetCmdValueCmd            | `fcat/cmd/jed_set_cmd_value`            | Sets the JED cmd value                                       |
| fcat_msgs/PidActivateCmd               | `fcat/cmd/pid_activate`                 | Activates a Fastcat internal PID Controller                  |



# The `fcat_srvs` Node

## Parameters

| Name               | Description                                                  | Default |
| ------------------ | ------------------------------------------------------------ | ------- |
| loop_rate_hz       | Loop rate of services | 100 |
| position_tolerance | Used to determine completion of `service/actuator_prof_pos` commands | 0.01    |
| velocity_tolerance | Used to determine completion of `service/actuator_prof_vel` commands | 0.01    |
| current_tolerance  | Used to determine completion of `service/actuator_prof_vel` commands | 0.01    |

### Services

Services are only created if at least one device is found on the user-specified Fastcat configuration file. Currently, if the bus topology lacks `actuator` or `pid` devices, the `fcat_srvs` will not offer any services and need not be started at all.

| Type                                | Default Name                        | Description                                                  |
| ----------------------------------- | ----------------------------------- | ------------------------------------------------------------ |
| fcat_msgs/srv/ActuatorCalibrateCmd  | `fcat/service/actuator_calibrate`   | Executes blocking service to perform Actuator hardstop calibration |
| fcat_msgs/srv/ActuatorProfPosCmd    | `fcat/service/actuator_prof_pos`    | Executes blocking service to execute a profile position command to target position |
| fcat_msgs/srv/ActuatorProfVelCmd    | `fcat/service/actuator_prof_vel`    | Executes blocking service to execute a profiled velocity command to target velocity. Returns once target velocity is attained. |
| fcat_msgs/srv/ActuatorProfTorqueCmd | `fcat/service/actuator_prof_torque` | Executes blocking service to execute a profiled torque command to target current. Returns once target velocity is attained. |
| fcat_msgs/srv/PidActivateCmd        | `fcat/service/pid_activate`         | Executed blocking service to execute a PID controller. The will return once a timeout is reached or if the input error signal is within the `deadband` argument for `persistence_duration` seconds |

