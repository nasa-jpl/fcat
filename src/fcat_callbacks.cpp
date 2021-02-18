#include "fcat/fcat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"

#include "fastcat/jsd/actuator.h"

fcat_msgs::msg::ActuatorState 
  ActuatorStateToMsg(std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::ActuatorState();

  msg.actual_position =              state->actuator_state.actual_position;
  msg.actual_velocity =              state->actuator_state.actual_velocity;
  msg.actual_current =               state->actuator_state.actual_current;
  msg.faulted =                      state->actuator_state.faulted;
  msg.cmd_position =                 state->actuator_state.cmd_position;
  msg.cmd_velocity =                 state->actuator_state.cmd_velocity;
  msg.cmd_current =                  state->actuator_state.cmd_current;
  msg.cmd_max_current =              state->actuator_state.cmd_max_current;
  msg.egd_state_machine_state =      state->actuator_state.egd_state_machine_state;
  msg.egd_mode_of_operation =        state->actuator_state.egd_mode_of_operation;
  msg.sto_engaged =                  state->actuator_state.sto_engaged;
  msg.hall_state =                   state->actuator_state.hall_state;
  msg.target_reached =               state->actuator_state.target_reached;
  msg.motor_on =                     state->actuator_state.motor_on;
  msg.fault_code =                   state->actuator_state.fault_code;
  msg.bus_voltage =                  state->actuator_state.bus_voltage;
  msg.drive_temperature =            state->actuator_state.drive_temperature;
  msg.egd_actual_position =          state->actuator_state.egd_actual_position;
  msg.egd_cmd_position =             state->actuator_state.egd_cmd_position;
  msg.actuator_state_machine_state = state->actuator_state.actuator_state_machine_state;

  return msg;
}

fcat_msgs::msg::EgdState EgdStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::EgdState();

  msg.actual_position =            state->egd_state.actual_position;
  msg.actual_velocity =            state->egd_state.actual_velocity;
  msg.actual_current =             state->egd_state.actual_current;
  msg.faulted =                    state->egd_state.faulted;
  msg.cmd_position =               state->egd_state.cmd_position;
  msg.cmd_velocity =               state->egd_state.cmd_velocity;
  msg.cmd_current =                state->egd_state.cmd_current;
  msg.cmd_max_current =            state->egd_state.cmd_max_current;
  msg.cmd_ff_position =            state->egd_state.cmd_ff_position;
  msg.cmd_ff_velocity =            state->egd_state.cmd_ff_velocity;
  msg.cmd_ff_current =             state->egd_state.cmd_ff_current;
  msg.actual_state_machine_state = state->egd_state.actual_state_machine_state;
  msg.actual_mode_of_operation =   state->egd_state.actual_mode_of_operation;
  msg.async_sdo_in_prog =          state->egd_state.async_sdo_in_prog;
  msg.sto_engaged =                state->egd_state.sto_engaged;
  msg.hall_state =                 state->egd_state.hall_state;
  msg.in_motion =                  state->egd_state.in_motion;
  msg.warning =                    state->egd_state.warning;
  msg.target_reached =             state->egd_state.target_reached;
  msg.motor_on =                   state->egd_state.motor_on;
  msg.fault_code =                 state->egd_state.fault_code;
  msg.bus_voltage =                state->egd_state.bus_voltage;
  msg.analog_input_voltage =       state->egd_state.analog_input_voltage;
  msg.digital_input_ch1 =          state->egd_state.digital_input_ch1;
  msg.digital_input_ch2 =          state->egd_state.digital_input_ch2;
  msg.digital_input_ch3 =          state->egd_state.digital_input_ch3;
  msg.digital_input_ch4 =          state->egd_state.digital_input_ch4;
  msg.digital_input_ch5 =          state->egd_state.digital_input_ch5;
  msg.digital_input_ch6 =          state->egd_state.digital_input_ch6;
  msg.digital_output_cmd_ch1 =     state->egd_state.digital_output_cmd_ch1;
  msg.digital_output_cmd_ch2 =     state->egd_state.digital_output_cmd_ch2;
  msg.digital_output_cmd_ch3 =     state->egd_state.digital_output_cmd_ch3;
  msg.digital_output_cmd_ch4 =     state->egd_state.digital_output_cmd_ch4;
  msg.digital_output_cmd_ch5 =     state->egd_state.digital_output_cmd_ch5;
  msg.digital_output_cmd_ch6 =     state->egd_state.digital_output_cmd_ch6;
  msg.drive_temperature =          state->egd_state.drive_temperature;

  return msg;
}

fcat_msgs::msg::El2124State El2124StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El2124State();

  msg.level_ch1 = state->el2124_state.level_ch1;
  msg.level_ch2 = state->el2124_state.level_ch2;
  msg.level_ch3 = state->el2124_state.level_ch3;
  msg.level_ch4 = state->el2124_state.level_ch4;

  return msg;
}

fcat_msgs::msg::El3208State El3208StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3208State();

  msg.output_ch1 =    state->el3208_state.output_ch1;
  msg.adc_value_ch1 = state->el3208_state.adc_value_ch1;
  msg.output_ch2 =    state->el3208_state.output_ch2;
  msg.adc_value_ch2 = state->el3208_state.adc_value_ch2;
  msg.output_ch3 =    state->el3208_state.output_ch3;
  msg.adc_value_ch3 = state->el3208_state.adc_value_ch3;
  msg.output_ch4 =    state->el3208_state.output_ch4;
  msg.adc_value_ch4 = state->el3208_state.adc_value_ch4;
  msg.output_ch5 =    state->el3208_state.output_ch5;
  msg.adc_value_ch5 = state->el3208_state.adc_value_ch5;
  msg.output_ch6 =    state->el3208_state.output_ch6;
  msg.adc_value_ch6 = state->el3208_state.adc_value_ch6;
  msg.output_ch7 =    state->el3208_state.output_ch7;
  msg.adc_value_ch7 = state->el3208_state.adc_value_ch7;
  msg.output_ch8 =    state->el3208_state.output_ch8;
  msg.adc_value_ch8 = state->el3208_state.adc_value_ch8;

  return msg;
}

fcat_msgs::msg::El3602State El3602StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3602State();


  msg.voltage_ch1 =   state->el3602_state.voltage_ch1;
  msg.adc_value_ch1 = state->el3602_state.adc_value_ch1;
  msg.voltage_ch2 =   state->el3602_state.voltage_ch2;
  msg.adc_value_ch2 = state->el3602_state.adc_value_ch2;

  return msg;
}

fcat_msgs::msg::JedState JedStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::JedState();

  msg.status = state->jed_state.status;
  msg.cmd = state->jed_state.cmd;
  msg.w_raw = state->jed_state.w_raw;
  msg.x_raw = state->jed_state.x_raw;
  msg.y_raw = state->jed_state.y_raw;
  msg.z_raw = state->jed_state.z_raw;
  msg.w = state->jed_state.w;
  msg.x = state->jed_state.x;
  msg.y = state->jed_state.y;
  msg.z = state->jed_state.z;

  return msg;
}

fcat_msgs::msg::CommanderState CommanderStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::CommanderState();

  msg.enable = state->commander_state.enable;

  return msg;
}

fcat_msgs::msg::ConditionalState ConditionalStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::ConditionalState();

  msg.output = state->conditional_state.output;

  return msg;
}

fcat_msgs::msg::FaulterState FaulterStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FaulterState();

  msg.enable = state->faulter_state.enable;
  msg.fault_active = state->faulter_state.fault_active;

  return msg;
}

fcat_msgs::msg::FilterState FilterStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FilterState();

  msg.output = state->filter_state.output;

  return msg;
}

/* TODO
auto msg = geometry_msgs::msg::Wrench();

msg.force.x  = state->fts_state.raw_fx;
msg.force.y  = state->fts_state.raw_fy;
msg.force.z  = state->fts_state.raw_fz;
msg.torque.x = state->fts_state.raw_tx;
msg.torque.y = state->fts_state.raw_ty;
msg.torque.z = state->fts_state.raw_tz;

fts_raw_pub_[state->name]->publish(msg);

msg.force.x  = state->fts_state.tared_fx;
msg.force.y  = state->fts_state.tared_fy;
msg.force.z  = state->fts_state.tared_fz;
msg.torque.x = state->fts_state.tared_tx;
msg.torque.y = state->fts_state.tared_ty;
msg.torque.z = state->fts_state.tared_tz;

fts_tared_pub_[state->name]->publish(msg);
*/

fcat_msgs::msg::FunctionState FunctionStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FunctionState();

  msg.output = state->function_state.output;

  return msg;
}

fcat_msgs::msg::PidState PidStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::PidState();

  msg.active = state->pid_state.active;
  msg.output = state->pid_state.output;
  msg.kp_term = state->pid_state.kp_term;
  msg.ki_term = state->pid_state.ki_term;
  msg.kd_term = state->pid_state.kd_term;

  return msg;
}

fcat_msgs::msg::SaturationState SaturationStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::SaturationState();

  msg.output = state->saturation_state.output;

  return msg;
}

fcat_msgs::msg::SchmittTriggerState SchmittTriggerStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::SchmittTriggerState();

  msg.output = state->schmitt_trigger_state.output;

  return msg;
}

fcat_msgs::msg::SignalGeneratorState SignalGeneratorStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::SignalGeneratorState();

  msg.output = state->signal_generator_state.output;

  return msg;
}

void Fcat::ResetCb( const std::shared_ptr<std_msgs::msg::Empty> msg)
{
  (void)msg;
  fcat_manager_.ExecuteAllDeviceResets();
}

void Fcat::FaultCb( const std::shared_ptr<std_msgs::msg::Empty> msg)
{
  (void)msg;
  fcat_manager_.ExecuteAllDeviceFaults();
}

void Fcat::ActuatorCSPCmdCb( const std::shared_ptr<fcat_msgs::msg::ActuatorCspCmd> msg){
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_CSP_CMD;
  cmd.actuator_csp_cmd.target_position = msg->target_position;
  cmd.actuator_csp_cmd.position_offset = msg->position_offset;
  cmd.actuator_csp_cmd.velocity_offset = msg->velocity_offset;
  cmd.actuator_csp_cmd.torque_offset_amps = msg->torque_offset_amps;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::ActuatorCSVCmdCb( const std::shared_ptr<fcat_msgs::msg::ActuatorCsvCmd> msg){
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_CSV_CMD;
  cmd.actuator_csv_cmd.target_velocity = msg->target_velocity;
  cmd.actuator_csv_cmd.velocity_offset = msg->velocity_offset;
  cmd.actuator_csv_cmd.torque_offset_amps = msg->torque_offset_amps;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::ActuatorCSTCmdCb( const std::shared_ptr<fcat_msgs::msg::ActuatorCstCmd> msg){
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_CST_CMD;
  cmd.actuator_cst_cmd.target_torque_amps = msg->target_torque_amps;
  cmd.actuator_cst_cmd.torque_offset_amps = msg->torque_offset_amps;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::ActuatorCalibrateCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::ActuatorCalibrateCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_CALIBRATE_CMD;
  cmd.actuator_calibrate_cmd.velocity    = msg->velocity;
  cmd.actuator_calibrate_cmd.accel       = msg->accel;
  cmd.actuator_calibrate_cmd.max_current = msg->max_current;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::ActuatorProfPosCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfPosCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_PROF_POS_CMD;
  cmd.actuator_prof_pos_cmd.target_position = msg->target_position;
  cmd.actuator_prof_pos_cmd.profile_velocity = msg->profile_velocity;
  cmd.actuator_prof_pos_cmd.profile_accel= msg->profile_accel;
  cmd.actuator_prof_pos_cmd.relative = msg->relative;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}
void Fcat::ActuatorProfTorqueCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfTorqueCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_PROF_TORQUE_CMD;
  cmd.actuator_prof_torque_cmd.target_torque_amps = msg->target_torque_amps;
  cmd.actuator_prof_torque_cmd.max_duration = msg->max_duration;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}
void Fcat::ActuatorProfVelCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfVelCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_PROF_VEL_CMD;
  cmd.actuator_prof_vel_cmd.target_velocity = msg->target_velocity;
  cmd.actuator_prof_vel_cmd.profile_accel = msg->profile_accel;
  cmd.actuator_prof_vel_cmd.max_duration = msg->max_duration;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetOutputPositionCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::ActuatorSetOutputPositionCmd> msg)
{

  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_SET_OUTPUT_POSITION_CMD;
  cmd.actuator_set_output_position_cmd.position = msg->position;

  if(DeviceExistsOnBus(cmd.name, fastcat::ACTUATOR_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::CommanderEnableCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::CommanderEnableCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::COMMANDER_ENABLE_CMD;
  cmd.commander_enable_cmd.duration = msg->duration;

  if(DeviceExistsOnBus(cmd.name, fastcat::COMMANDER_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::CommanderDisableCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::CommanderDisableCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::COMMANDER_DISABLE_CMD;

  if(DeviceExistsOnBus(cmd.name, fastcat::COMMANDER_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::El2124WriteAllChannelsCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::El2124WriteAllChannelsCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL2124_WRITE_ALL_CHANNELS_CMD;
  cmd.el2124_write_all_channels_cmd.channel_ch1 = msg->channel_ch1;
  cmd.el2124_write_all_channels_cmd.channel_ch2 = msg->channel_ch2;
  cmd.el2124_write_all_channels_cmd.channel_ch3 = msg->channel_ch3;
  cmd.el2124_write_all_channels_cmd.channel_ch4 = msg->channel_ch4;

  if(DeviceExistsOnBus(cmd.name, fastcat::EL2124_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}
void Fcat::El2124WriteChannelCmdCb( 
    const std::shared_ptr<fcat_msgs::msg::El2124WriteChannelCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL2124_WRITE_CHANNEL_CMD;
  cmd.el2124_write_channel_cmd.channel = msg->channel;
  cmd.el2124_write_channel_cmd.level = msg->level;

  if(DeviceExistsOnBus(cmd.name, fastcat::EL2124_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::FaulterEnableCmdCb( const std::shared_ptr<fcat_msgs::msg::FaulterEnableCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::FAULTER_ENABLE_CMD;
  cmd.faulter_enable_cmd.enable = msg->enable;

  if(DeviceExistsOnBus(cmd.name, fastcat::FAULTER_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::FtsTareCmdCb( const std::shared_ptr<fcat_msgs::msg::FtsTareCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::FTS_TARE_CMD;

  if(DeviceExistsOnBus(cmd.name, fastcat::FTS_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::JedSetCmdValueCmdCb( const std::shared_ptr<fcat_msgs::msg::JedSetCmdValueCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::JED_SET_CMD_VALUE_CMD;
  cmd.jed_set_cmd_value_cmd.cmd = msg->cmd;

  if(DeviceExistsOnBus(cmd.name, fastcat::JED_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}

void Fcat::PidActivateCmdCb( const std::shared_ptr<fcat_msgs::msg::PidActivateCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::PID_ACTIVATE_CMD;
  cmd.pid_activate_cmd.setpoint = msg->setpoint;
  cmd.pid_activate_cmd.deadband = msg->deadband;
  cmd.pid_activate_cmd.persistence_duration = msg->persistence_duration;
  cmd.pid_activate_cmd.max_duration = msg->max_duration;

  if(DeviceExistsOnBus(cmd.name, fastcat::PID_STATE)){
    fcat_manager_.QueueCommand(cmd);
  }
}
