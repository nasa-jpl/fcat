# fcat_msgs

ROS 2 message and service definitions for the FCAT (Fieldbus Common Abstraction Tool) system.

## Overview

This package defines 95 messages and 31 services used by the `fcat` node to interface with EtherCAT devices and software modules. Messages follow a consistent pattern:

- **State/States** - Telemetry published by `fcat` (e.g., `ActuatorState`, `ActuatorStates`)
- **Cmd/Cmds** - Commands subscribed to by `fcat` (e.g., `ActuatorCspCmd`, `ActuatorCspCmds`)
- **Service** - Request/response services exposed by `fcat` (e.g., `ActuatorProfPosService`)

`States` (plural) messages are variable-sized arrays of the corresponding singular `State` message. Similarly, `Cmds` (plural) messages wrap arrays of `Cmd` for multi-device commands.

## Topic Namespacing

| Prefix    | Direction       | Description                                        |
|-----------|-----------------|----------------------------------------------------|
| `state/`  | fcat -> clients | Telemetry published every cycle                    |
| `impl/`   | clients -> fcat | Low-level command topics (non-blocking)             |
| `cmd/`    | clients -> fcat | Service interface on the main fcat node             |
| `srv/`    | clients -> fcat | Blocking wrapper services on the fcat_srvs node     |

## Messages

### Actuator

Control and telemetry for EtherCAT servo drives.

| Message | Topic | Description |
|---------|-------|-------------|
| ActuatorState | - | Single actuator telemetry |
| ActuatorStates | `state/actuators` | Array of actuator telemetry |
| ActuatorCspCmd / Cmds | `impl/actuator_csp`, `impl/actuator_csp_multi` | Cyclic Synchronous Position |
| ActuatorCsvCmd / Cmds | `impl/actuator_csv`, `impl/actuator_csv_multi` | Cyclic Synchronous Velocity |
| ActuatorCstCmd / Cmds | `impl/actuator_cst`, `impl/actuator_cst_multi` | Cyclic Synchronous Torque |
| ActuatorHaltCmd / Cmds | `impl/actuator_halt`, `impl/actuator_halt_multi` | Halt motion |
| ActuatorProfPosCmd / Cmds | `impl/actuator_prof_pos`, `impl/actuator_prof_pos_multi` | Profile Position mode |
| ActuatorProfTorqueCmd | `impl/actuator_prof_torque` | Profile Torque mode |
| ActuatorProfVelCmd | `impl/actuator_prof_vel` | Profile Velocity mode |
| ActuatorCalibrateCmd | `impl/actuator_calibrate` | Trigger calibration |
| ActuatorSetDigitalOutputCmd | `impl/actuator_set_digital_output` | Set digital output |
| ActuatorSetMaxCurrentCmd | `impl/actuator_set_max_current` | Set max current limit |
| ActuatorSetOutputPositionCmd | `impl/actuator_set_output_position` | Set output position |
| ActuatorSetProfDisengagingTimeoutCmd | `impl/actuator_set_prof_disengaging_timeout` | Set profile disengaging timeout |
| ActuatorSetUnitModeCmd | `impl/actuator_set_unit_mode` | Set unit mode |

### EGD (EtherCAT Gold Drive)

| Message | Topic | Description |
|---------|-------|-------------|
| EgdState | - | Single EGD telemetry |
| EgdStates | `state/egds` | Array of EGD telemetry |

### Commander

Enable/disable device commanding.

| Message | Topic | Description |
|---------|-------|-------------|
| CommanderState / States | `state/commanders` | Commander telemetry |
| CommanderEnableCmd | `impl/commander_enable` | Enable a commander |
| CommanderDisableCmd | `impl/commander_disable` | Disable a commander |

### Async SDO

Service Data Object read/write over EtherCAT.

| Message | Topic | Description |
|---------|-------|-------------|
| AsyncSdoReadCmd | `impl/async_sdo_read` | Request an SDO read |
| AsyncSdoWriteCmd | `impl/async_sdo_write` | Request an SDO write |
| AsyncSdoResponse | `state/async_sdo_response` | SDO read/write response |

### EtherCAT I/O Terminals

State and command messages for Beckhoff EtherCAT terminals. Digital/analog output terminals include write commands; input-only terminals have state only.

**Digital Input:**

| Message | Topic |
|---------|-------|
| El1008State / States | `state/el1008s` |

**Digital Output:**

| Terminal | State Topic | Commands |
|----------|-------------|----------|
| EL2124 | `state/el2124s` | El2124WriteChannelCmd, El2124WriteAllChannelsCmd |
| EL2798 | `state/el2798s` | El2798WriteChannelCmd, El2798WriteAllChannelsCmd |
| EL2809 | `state/el2809s` | El2809WriteChannelCmd, El2809WriteAllChannelsCmd |
| EL2828 | `state/el2828s` | El2828WriteChannelCmd, El2828WriteAllChannelsCmd |

**Analog Input:**

| Terminal | State Topic |
|----------|-------------|
| EL3104 | `state/el3104s` |
| EL3162 | `state/el3162s` |
| EL3202 | `state/el3202s` |
| EL3208 | `state/el3208s` |
| EL3314 | `state/el3314s` |
| EL3318 | `state/el3318s` |
| EL3602 | `state/el3602s` |

**Analog Output:**

| Terminal | State Topic | Commands |
|----------|-------------|----------|
| EL4102 | `state/el4102s` | El4102WriteChannelCmd, El4102WriteAllChannelsCmd |

**Encoder:**

| Terminal | State Topic |
|----------|-------------|
| EL5042 | `state/el5042s` |

### Faulter

| Message | Topic | Description |
|---------|-------|-------------|
| FaulterState / States | `state/faulters` | Faulter telemetry |
| FaulterEnableCmd | `impl/faulter_enable` | Enable a faulter |

### FTS (Force-Torque Sensor)

| Message | Topic | Description |
|---------|-------|-------------|
| FtsState / States | `state/fts` | FTS telemetry |
| FtsTareCmd | `impl/fts_tare` | Tare a sensor |

### ILD1900 (Laser Displacement Sensor)

| Message | Topic | Description |
|---------|-------|-------------|
| Ild1900State / States | `state/ild1900s` | Sensor telemetry |

### Signal Processing Modules

| Message | Topic | Description |
|---------|-------|-------------|
| PidState / States | `state/pids` | PID controller telemetry |
| PidActivateCmd | `impl/pid_activate` | Activate/deactivate a PID |
| FilterState / States | `state/filter` | Filter telemetry |
| FunctionState / States | `state/functions` | Function telemetry |
| SaturationState / States | `state/saturations` | Saturation telemetry |
| SchmittTriggerState / States | `state/schmitt_triggers` | Schmitt trigger telemetry |
| SignalGeneratorState / States | `state/signal_generators` | Signal generator telemetry |
| LinearInterpolationState / States | `state/linear_interpolators` | Linear interpolation telemetry |
| ConditionalState / States | `state/conditionals` | Conditional telemetry |
| ThreeNodeThermalModelState / States | `state/three_node_thermal_models` | Thermal model telemetry |

### Other

| Message | Topic | Description |
|---------|-------|-------------|
| ModuleState | `state/module_state` | Overall module state (always published) |

## Services

### Actuator Services

| Service | `cmd/` topic | `srv/` topic | Description |
|---------|-------------|-------------|-------------|
| ActuatorCalibrateService | `cmd/actuator_calibrate` | `srv/actuator_calibrate` | Calibrate actuator |
| ActuatorHaltService | `cmd/actuator_halt` | - | Halt actuator |
| ActuatorProfPosService | `cmd/actuator_prof_pos` | `srv/actuator_prof_pos` | Profile position move |
| ActuatorProfTorqueService | `cmd/actuator_prof_torque` | `srv/actuator_prof_torque` | Profile torque move |
| ActuatorProfVelService | `cmd/actuator_prof_vel` | `srv/actuator_prof_vel` | Profile velocity move |
| ActuatorSetDigitalOutputService | `cmd/actuator_set_digital_output` | - | Set digital output |
| ActuatorSetGainSchedulingIndexService | `cmd/actuator_set_gain_scheduling_index` | - | Set gain scheduling index |
| ActuatorSetGainSchedulingModeService | - | `srv/actuator_set_gain_scheduling_mode` | Set gain scheduling mode (via TLC) |
| ActuatorSetMaxCurrentService | `cmd/actuator_set_max_current` | - | Set max current |
| ActuatorSetOutputPositionService | `cmd/actuator_set_output_position` | - | Set output position |
| ActuatorSetProfDisengagingTimeoutService | `cmd/actuator_set_prof_disengaging_timeout` | - | Set profile disengaging timeout |
| ActuatorSetUnitModeService | - | `srv/actuator_set_unit_mode` | Set unit mode (via TLC) |

### Other Services

| Service | `cmd/` topic | `srv/` topic | Description |
|---------|-------------|-------------|-------------|
| CommanderEnableService | `cmd/commander_enable` | - | Enable commander |
| CommanderDisableService | `cmd/commander_disable` | - | Disable commander |
| FaulterEnableService | `cmd/faulter_enable` | - | Enable faulter |
| PidActivateService | `cmd/pid_activate` | `srv/pid_activate` | Activate PID |
| DeviceTriggerService | `cmd/fts_tare` | - | Trigger device action (used for FTS tare) |
| AsyncSdoReadService | - | `srv/async_sdo_read` | Blocking SDO read |
| AsyncSdoWriteService | - | `srv/async_sdo_write` | Blocking SDO write |
| TlcReadService | - | `srv/tlc_read` | Two-Letter Command read |
| TlcWriteService | - | `srv/tlc_write` | Two-Letter Command write |

### EtherCAT I/O Services

Each writable terminal exposes both single-channel and all-channel services under `cmd/`:

| Terminal | WriteChannelService | WriteAllChannelsService |
|----------|-------------------|------------------------|
| EL2124 | `cmd/el2124_write_channel` | `cmd/el2124_write_all_channels` |
| EL2798 | `cmd/el2798_write_channel` | `cmd/el2798_write_all_channels` |
| EL2809 | `cmd/el2809_write_channel` | `cmd/el2809_write_all_channels` |
| EL2828 | `cmd/el2828_write_channel` | `cmd/el2828_write_all_channels` |
| EL4102 | `cmd/el4102_write_channel` | `cmd/el4102_write_all_channels` |
