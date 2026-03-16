#include "fcat/fcat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"

#include "fastcat/jsd/actuator.h"

#include <cstdio>

void Fcat::ResetCmdCb(const std::shared_ptr<std_msgs::msg::Empty> msg)
{
  reset_in_progress_ = true;
  fcat_manager_.ExecuteAllDeviceResets();
  FaultInterface::ResetCmdCb(msg);
  reset_in_progress_ = false;
}

void Fcat::FaultCmdCb(const std::shared_ptr<std_msgs::msg::Empty> msg)
{
  (void)msg;
  if (!reset_in_progress_) {
    fcat_manager_.ExecuteAllDeviceFaults();
    FaultInterface::FaultCmdCb(msg);
  }
}

void Fcat::AsyncSdoReadCmdCb(
  const std::shared_ptr<fcat_msgs::msg::AsyncSdoReadCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ASYNC_SDO_READ_CMD;
  cmd.async_sdo_read_cmd.sdo_index = msg->sdo_index;
  cmd.async_sdo_read_cmd.sdo_subindex = msg->sdo_subindex;
  cmd.async_sdo_read_cmd.data_type =
    jsd_sdo_data_type_from_string(msg->data_type);
  cmd.async_sdo_read_cmd.app_id = msg->app_id;

  if (device_name_state_map_.end() != device_name_state_map_.find(msg->name)) {
    QueueCommand(cmd);
  }
}
void Fcat::AsyncSdoWriteCmdCb(
  const std::shared_ptr<fcat_msgs::msg::AsyncSdoWriteCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ASYNC_SDO_WRITE_CMD;
  cmd.async_sdo_write_cmd.sdo_index = msg->sdo_index;
  cmd.async_sdo_write_cmd.sdo_subindex = msg->sdo_subindex;
  cmd.async_sdo_write_cmd.data_type =
    jsd_sdo_data_type_from_string(msg->data_type);
  cmd.async_sdo_write_cmd.app_id = msg->app_id;

  cmd.async_sdo_write_cmd.data =
    jsd_sdo_data_from_string(cmd.async_sdo_write_cmd.data_type, msg->data);

  if (device_name_state_map_.end() != device_name_state_map_.find(msg->name)) {
    QueueCommand(cmd);
  }
}

void Fcat::CallActuatorCSP(const fcat_msgs::msg::ActuatorCspCmd& msg, double t)
{
  if(!cs_subscription_cb_realtime_preempt_initialized_) {
    if(enable_realtime_preempt_) {
      RCLCPP_INFO(this->get_logger(),
        "Setting realtime priority for subscription callback thread to %d", 
        scheduler_priority_ - 1);
      SetRealtimePreempt(scheduler_priority_ - 1); 
    }
    cs_subscription_cb_realtime_preempt_initialized_ = true; 
  }

  if (!subscription_cpu_affinity_initialized_) {
    if (process_loop_cpu_id_ >= 0) {
      SetCpuAffinity();
    }
    subscription_cpu_affinity_initialized_ = true;
  }

  fastcat::DeviceCmd cmd;
  cmd.name = msg.name;
  cmd.type = fastcat::ACTUATOR_CSP_CMD;
  cmd.actuator_csp_cmd.request_time = msg.request_time;
  cmd.actuator_csp_cmd.target_position = msg.target_position;
  cmd.actuator_csp_cmd.position_offset = msg.position_offset;
  cmd.actuator_csp_cmd.velocity_offset = msg.velocity_offset;
  cmd.actuator_csp_cmd.torque_offset_amps = msg.torque_offset_amps;

  // these arguments are not part of the CSP spec, but are used to
  // inform how to update CSP setpoints if a new setpoint is not received
  // from the calling module
  cmd.actuator_csp_cmd.acceleration_offset = msg.acceleration_offset;
  cmd.actuator_csp_cmd.interpolation_mode = msg.interpolation_mode;
  cmd.actuator_csp_cmd.receipt_stamp_time = t;

  if (std::isnan(msg.request_time) ||
      std::isnan(msg.target_position) ||
      std::isnan(msg.position_offset) ||
      std::isnan(msg.velocity_offset) ||
      std::isnan(msg.torque_offset_amps)) {
    RCLCPP_WARN(this->get_logger(),
        "A NaN value was found in a CSP setpoint; "
        "The CSP setpoint was ignored"
    );
    return;
  }

  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorCSPCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCspCmd> msg)
{
  this->CallActuatorCSP(*msg, this->now().seconds());
}

void Fcat::ActuatorCSPCmdsCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCspCmds> msg)
{
  fprintf(stderr, "Processing ActuatorCSPCmdsCb\n");
  double t = this->now().seconds();
  for (auto csp_cmd : msg->commands) {
    CallActuatorCSP(csp_cmd, t);
  }
}

void Fcat::CallActuatorCSV(
  const fcat_msgs::msg::ActuatorCsvCmd& msg)
{
  if(!cs_subscription_cb_realtime_preempt_initialized_) {
    if(enable_realtime_preempt_) {
       RCLCPP_INFO(this->get_logger(),
        "Setting realtime priority for subscription callback thread to %d", 
        scheduler_priority_ - 1);
      SetRealtimePreempt(scheduler_priority_ - 1); 
    }
    cs_subscription_cb_realtime_preempt_initialized_ = true; 
  }
  if (!subscription_cpu_affinity_initialized_) {
    if (process_loop_cpu_id_ >= 0) {
      SetCpuAffinity();
    }
    subscription_cpu_affinity_initialized_ = true;
  }

  fastcat::DeviceCmd cmd;
  cmd.name = msg.name;
  cmd.type = fastcat::ACTUATOR_CSV_CMD;
  cmd.actuator_csv_cmd.target_velocity = msg.target_velocity;
  cmd.actuator_csv_cmd.velocity_offset = msg.velocity_offset;
  cmd.actuator_csv_cmd.torque_offset_amps = msg.torque_offset_amps;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorCSVCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCsvCmd> msg)
{
  this->CallActuatorCSV(*msg);
}

void Fcat::ActuatorCSVCmdsCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCsvCmds> msg)
{
  fprintf(stderr, "Processing ActuatorCSVCmdsCb\n");
  for (auto csv_cmd : msg->commands) {
    CallActuatorCSV(csv_cmd);
  }
}

void Fcat::CallActuatorCST(
  const fcat_msgs::msg::ActuatorCstCmd& msg)
{
  if(!cs_subscription_cb_realtime_preempt_initialized_) {
    if(enable_realtime_preempt_) {
      RCLCPP_INFO(this->get_logger(),
        "Setting realtime priority for subscription callback thread to %d", 
        scheduler_priority_ - 1);
      SetRealtimePreempt(scheduler_priority_ - 1); 
    }
    cs_subscription_cb_realtime_preempt_initialized_ = true; 
  }
  if (!subscription_cpu_affinity_initialized_) {
    if (process_loop_cpu_id_ >= 0) {
      SetCpuAffinity();
    }
    subscription_cpu_affinity_initialized_ = true;
  }

  fastcat::DeviceCmd cmd;
  cmd.name = msg.name;
  cmd.type = fastcat::ACTUATOR_CST_CMD;
  cmd.actuator_cst_cmd.target_torque_amps = msg.target_torque_amps;
  cmd.actuator_cst_cmd.torque_offset_amps = msg.torque_offset_amps;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorCSTCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCstCmd> msg)
{
  CallActuatorCST(*msg);
}


void Fcat::ActuatorCSTCmdsCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCstCmds> msg)
{
  fprintf(stderr, "Processing ActuatorCSTCmdsCb\n");
  for (auto cst_cmd : msg->commands) {
    CallActuatorCST(cst_cmd);
  }
}

void Fcat::ActuatorCalibrateCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorCalibrateCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_CALIBRATE_CMD;
  cmd.actuator_calibrate_cmd.velocity = msg->velocity;
  cmd.actuator_calibrate_cmd.accel = msg->accel;
  cmd.actuator_calibrate_cmd.max_current = msg->max_current;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::CallActuatorProfPos(const fcat_msgs::msg::ActuatorProfPosCmd& msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg.name;
  cmd.type = fastcat::ACTUATOR_PROF_POS_CMD;
  cmd.actuator_prof_pos_cmd.target_position = msg.target_position;
  cmd.actuator_prof_pos_cmd.profile_velocity = msg.profile_velocity;
  cmd.actuator_prof_pos_cmd.profile_accel = msg.profile_accel;
  cmd.actuator_prof_pos_cmd.relative = msg.relative;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorProfPosCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorProfPosCmd> msg)
{
  CallActuatorProfPos(*msg);
}
void Fcat::ActuatorProfTorqueCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorProfTorqueCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_PROF_TORQUE_CMD;
  cmd.actuator_prof_torque_cmd.target_torque_amps = msg->target_torque_amps;
  cmd.actuator_prof_torque_cmd.max_duration = msg->max_duration;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
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
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorProfPosCmdsCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorProfPosCmds> msg)
{
  for (auto prof_pos_cmd : msg->commands) {
    CallActuatorProfPos(prof_pos_cmd);
  }
}

void Fcat::ActuatorSetOutputPositionCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorSetOutputPositionCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_SET_OUTPUT_POSITION_CMD;
  cmd.actuator_set_output_position_cmd.position = msg->position;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetDigitalOutputCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorSetDigitalOutputCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_SET_DIGITAL_OUTPUT_CMD;
  cmd.actuator_set_digital_output_cmd.digital_output_index =
    msg->digital_output_index;
  cmd.actuator_set_digital_output_cmd.output_level = msg->output_level;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetMaxCurrentCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorSetMaxCurrentCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_SET_MAX_CURRENT_CMD;
  cmd.actuator_set_max_current_cmd.current = msg->max_current;

  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetUnitModeCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorSetUnitModeCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_SDO_SET_UNIT_MODE_CMD;
  cmd.actuator_sdo_set_unit_mode_cmd.mode = msg->mode;
  cmd.actuator_sdo_set_unit_mode_cmd.app_id = msg->app_id;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetProfDisengagingTimeoutCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorSetProfDisengagingTimeoutCmd>
    msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_SET_PROF_DISENGAGING_TIMEOUT_CMD;
  cmd.actuator_set_prof_disengaging_timeout_cmd.timeout = msg->timeout;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorHaltCmdCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorHaltCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::ACTUATOR_HALT_CMD;
  if (ActuatorExistsOnBus(cmd.name)) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorHaltCmdsCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorHaltCmds> msg)
{
  for (auto& name : msg->names) {
    fastcat::DeviceCmd cmd;
    cmd.name = name;
    cmd.type = fastcat::ACTUATOR_HALT_CMD;
    if (ActuatorExistsOnBus(cmd.name)) {
      QueueCommand(cmd);
    }
  }
}

void Fcat::CommanderEnableCmdCb(
  const std::shared_ptr<fcat_msgs::msg::CommanderEnableCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::COMMANDER_ENABLE_CMD;
  cmd.commander_enable_cmd.duration = msg->duration;

  if (DeviceExistsOnBus(cmd.name, fastcat::COMMANDER_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::CommanderDisableCmdCb(
  const std::shared_ptr<fcat_msgs::msg::CommanderDisableCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::COMMANDER_DISABLE_CMD;

  if (DeviceExistsOnBus(cmd.name, fastcat::COMMANDER_STATE)) {
    QueueCommand(cmd);
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

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2124_STATE)) {
    QueueCommand(cmd);
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

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2124_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::El2809WriteAllChannelsCmdCb(
  const std::shared_ptr<fcat_msgs::msg::El2809WriteAllChannelsCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL2809_WRITE_ALL_CHANNELS_CMD;
  cmd.el2809_write_all_channels_cmd.channel_ch1 = msg->channel_ch1;
  cmd.el2809_write_all_channels_cmd.channel_ch2 = msg->channel_ch2;
  cmd.el2809_write_all_channels_cmd.channel_ch3 = msg->channel_ch3;
  cmd.el2809_write_all_channels_cmd.channel_ch4 = msg->channel_ch4;
  cmd.el2809_write_all_channels_cmd.channel_ch5 = msg->channel_ch5;
  cmd.el2809_write_all_channels_cmd.channel_ch6 = msg->channel_ch6;
  cmd.el2809_write_all_channels_cmd.channel_ch7 = msg->channel_ch7;
  cmd.el2809_write_all_channels_cmd.channel_ch8 = msg->channel_ch8;
  cmd.el2809_write_all_channels_cmd.channel_ch9 = msg->channel_ch9;
  cmd.el2809_write_all_channels_cmd.channel_ch10 = msg->channel_ch10;
  cmd.el2809_write_all_channels_cmd.channel_ch11 = msg->channel_ch11;
  cmd.el2809_write_all_channels_cmd.channel_ch12 = msg->channel_ch12;
  cmd.el2809_write_all_channels_cmd.channel_ch13 = msg->channel_ch13;
  cmd.el2809_write_all_channels_cmd.channel_ch14 = msg->channel_ch14;
  cmd.el2809_write_all_channels_cmd.channel_ch15 = msg->channel_ch15;
  cmd.el2809_write_all_channels_cmd.channel_ch16 = msg->channel_ch16;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2809_STATE)) {
    QueueCommand(cmd);
  }
}
void Fcat::El2809WriteChannelCmdCb(
  const std::shared_ptr<fcat_msgs::msg::El2809WriteChannelCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL2809_WRITE_CHANNEL_CMD;
  cmd.el2809_write_channel_cmd.channel = msg->channel;
  cmd.el2809_write_channel_cmd.level = msg->level;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2809_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::El2798WriteAllChannelsCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2798WriteAllChannelsCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL2798_WRITE_ALL_CHANNELS_CMD;
  cmd.el2798_write_all_channels_cmd.channel_ch1  = msg->channel_ch1;
  cmd.el2798_write_all_channels_cmd.channel_ch2  = msg->channel_ch2;
  cmd.el2798_write_all_channels_cmd.channel_ch3  = msg->channel_ch3;
  cmd.el2798_write_all_channels_cmd.channel_ch4  = msg->channel_ch4;
  cmd.el2798_write_all_channels_cmd.channel_ch5  = msg->channel_ch5;
  cmd.el2798_write_all_channels_cmd.channel_ch6  = msg->channel_ch6;
  cmd.el2798_write_all_channels_cmd.channel_ch7  = msg->channel_ch7;
  cmd.el2798_write_all_channels_cmd.channel_ch8  = msg->channel_ch8;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2798_STATE)) {
    QueueCommand(cmd);
  }
}
void Fcat::El2798WriteChannelCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2798WriteChannelCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name                             = msg->name;
  cmd.type                             = fastcat::EL2798_WRITE_CHANNEL_CMD;
  cmd.el2798_write_channel_cmd.channel = msg->channel;
  cmd.el2798_write_channel_cmd.level   = msg->level;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2798_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::El2828WriteAllChannelsCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2828WriteAllChannelsCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL2828_WRITE_ALL_CHANNELS_CMD;
  cmd.el2828_write_all_channels_cmd.channel_ch1  = msg->channel_ch1;
  cmd.el2828_write_all_channels_cmd.channel_ch2  = msg->channel_ch2;
  cmd.el2828_write_all_channels_cmd.channel_ch3  = msg->channel_ch3;
  cmd.el2828_write_all_channels_cmd.channel_ch4  = msg->channel_ch4;
  cmd.el2828_write_all_channels_cmd.channel_ch5  = msg->channel_ch5;
  cmd.el2828_write_all_channels_cmd.channel_ch6  = msg->channel_ch6;
  cmd.el2828_write_all_channels_cmd.channel_ch7  = msg->channel_ch7;
  cmd.el2828_write_all_channels_cmd.channel_ch8  = msg->channel_ch8;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2828_STATE)) {
    QueueCommand(cmd);
  }
}
void Fcat::El2828WriteChannelCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2828WriteChannelCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name                             = msg->name;
  cmd.type                             = fastcat::EL2828_WRITE_CHANNEL_CMD;
  cmd.el2828_write_channel_cmd.channel = msg->channel;
  cmd.el2828_write_channel_cmd.level   = msg->level;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL2828_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::El4102WriteAllChannelsCmdCb(
  const std::shared_ptr<fcat_msgs::msg::El4102WriteAllChannelsCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL4102_WRITE_ALL_CHANNELS_CMD;
  cmd.el4102_write_all_channels_cmd.voltage_output_ch1 =
    msg->voltage_output_ch1;
  cmd.el4102_write_all_channels_cmd.voltage_output_ch2 =
    msg->voltage_output_ch2;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL4102_STATE)) {
    QueueCommand(cmd);
  }
}
void Fcat::El4102WriteChannelCmdCb(
  const std::shared_ptr<fcat_msgs::msg::El4102WriteChannelCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::EL4102_WRITE_CHANNEL_CMD;
  cmd.el4102_write_channel_cmd.channel = msg->channel;
  cmd.el4102_write_channel_cmd.voltage_output = msg->voltage_output;

  if (DeviceExistsOnBus(cmd.name, fastcat::EL4102_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::FaulterEnableCmdCb(
  const std::shared_ptr<fcat_msgs::msg::FaulterEnableCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::FAULTER_ENABLE_CMD;
  cmd.faulter_enable_cmd.enable = msg->enable;

  if (DeviceExistsOnBus(cmd.name, fastcat::FAULTER_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::FtsTareCmdCb(const std::shared_ptr<fcat_msgs::msg::FtsTareCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::FTS_TARE_CMD;

  if (DeviceExistsOnBus(cmd.name, fastcat::FTS_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::PidActivateCmdCb(
  const std::shared_ptr<fcat_msgs::msg::PidActivateCmd> msg)
{
  fastcat::DeviceCmd cmd;
  cmd.name = msg->name;
  cmd.type = fastcat::PID_ACTIVATE_CMD;
  cmd.pid_activate_cmd.setpoint = msg->setpoint;
  cmd.pid_activate_cmd.deadband = msg->deadband;
  cmd.pid_activate_cmd.persistence_duration = msg->persistence_duration;
  cmd.pid_activate_cmd.max_duration = msg->max_duration;

  if (DeviceExistsOnBus(cmd.name, fastcat::PID_STATE)) {
    QueueCommand(cmd);
  }
}

void Fcat::ResetSrvCb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Reset Command");
  fcat_manager_.ExecuteAllDeviceResets();
  FaultInterface::ResetSrvCb(request, response);
}

void Fcat::FaultSrvCb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Fault Command");
  fcat_manager_.ExecuteAllDeviceFaults();
  FaultInterface::FaultSrvCb(request, response);
}

void Fcat::ActuatorHaltSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorHaltService::Request> request,
  std::shared_ptr<fcat_msgs::srv::ActuatorHaltService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Halt Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_HALT_CMD;

  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetGainSchedulingIndexSrvCb(
  const std::shared_ptr<
    fcat_msgs::srv::ActuatorSetGainSchedulingIndexService::Request>
    request,
  std::shared_ptr<
    fcat_msgs::srv::ActuatorSetGainSchedulingIndexService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Gain Scheduling Index Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_SET_GAIN_SCHEDULING_INDEX_CMD;
  cmd.actuator_set_gain_scheduling_index_cmd.gain_scheduling_index =
    request->gain_scheduling_index;

  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetMaxCurrentSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorSetMaxCurrentService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorSetMaxCurrentService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Max Current Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_SET_MAX_CURRENT_CMD;
  cmd.actuator_set_max_current_cmd.current = request->max_current;

  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetDigitalOutputSrvCb(
  const std::shared_ptr<
    fcat_msgs::srv::ActuatorSetDigitalOutputService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorSetDigitalOutputService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Digital Output Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_SET_DIGITAL_OUTPUT_CMD;
  cmd.actuator_set_digital_output_cmd.digital_output_index =
    request->digital_output_index;
  cmd.actuator_set_digital_output_cmd.output_level = request->output_level;
  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetProfDisengagingTimeoutSrvCb(
  const std::shared_ptr<
    fcat_msgs::srv::ActuatorSetProfDisengagingTimeoutService::Request>
    request,
  std::shared_ptr<
    fcat_msgs::srv::ActuatorSetProfDisengagingTimeoutService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Profile Disengaging Timeout Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_SET_PROF_DISENGAGING_TIMEOUT_CMD;
  cmd.actuator_set_prof_disengaging_timeout_cmd.timeout = request->timeout;

  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorSetOutputPositionSrvCb(
  const std::shared_ptr<
    fcat_msgs::srv::ActuatorSetOutputPositionService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorSetOutputPositionService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Output Position Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_SET_OUTPUT_POSITION_CMD;
  cmd.actuator_set_output_position_cmd.position = request->position;
  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorCalibrateSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Calibrate Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_CALIBRATE_CMD;
  cmd.actuator_calibrate_cmd.velocity = request->velocity;
  cmd.actuator_calibrate_cmd.accel = request->accel;
  cmd.actuator_calibrate_cmd.max_current = request->max_current;
  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorProfPosSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Prof Pos Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_PROF_POS_CMD;
  cmd.actuator_prof_pos_cmd.target_position = request->target_position;
  cmd.actuator_prof_pos_cmd.profile_velocity = request->profile_velocity;
  cmd.actuator_prof_pos_cmd.profile_accel = request->profile_accel;
  cmd.actuator_prof_pos_cmd.relative = request->relative;
  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorProfVelSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Prof Vel Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_PROF_VEL_CMD;
  cmd.actuator_prof_vel_cmd.target_velocity = request->target_velocity;
  cmd.actuator_prof_vel_cmd.profile_accel = request->profile_accel;
  cmd.actuator_prof_vel_cmd.max_duration = request->max_duration;
  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::ActuatorProfTorqueSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Prof Torque Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::ACTUATOR_PROF_TORQUE_CMD;
  cmd.actuator_prof_torque_cmd.target_torque_amps = request->target_torque_amps;
  cmd.actuator_prof_torque_cmd.max_duration = request->max_duration;
  response->success = ActuatorExistsOnBus(cmd.name, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::CommanderEnableSrvCb(
  const std::shared_ptr<fcat_msgs::srv::CommanderEnableService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::CommanderEnableService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Commander Enable Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::COMMANDER_ENABLE_CMD;
  cmd.commander_enable_cmd.duration = request->duration;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::COMMANDER_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::CommanderDisableSrvCb(
  const std::shared_ptr<fcat_msgs::srv::CommanderDisableService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::CommanderDisableService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Commander Disable Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::COMMANDER_DISABLE_CMD;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::COMMANDER_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2124WriteAllChannelsSrvCb(
  const std::shared_ptr<fcat_msgs::srv::El2124WriteAllChannelsService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::El2124WriteAllChannelsService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2124 Write All Channels Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL2124_WRITE_ALL_CHANNELS_CMD;
  cmd.el2124_write_all_channels_cmd.channel_ch1 = request->channel_ch1;
  cmd.el2124_write_all_channels_cmd.channel_ch2 = request->channel_ch2;
  cmd.el2124_write_all_channels_cmd.channel_ch3 = request->channel_ch3;
  cmd.el2124_write_all_channels_cmd.channel_ch4 = request->channel_ch4;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::EL2124_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2124WriteChannelSrvCb(
  const std::shared_ptr<fcat_msgs::srv::El2124WriteChannelService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::El2124WriteChannelService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2124 Write Channel Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL2124_WRITE_CHANNEL_CMD;
  cmd.el2124_write_channel_cmd.channel = request->channel;
  cmd.el2124_write_channel_cmd.level = request->level;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::EL2124_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2809WriteAllChannelsSrvCb(
  const std::shared_ptr<fcat_msgs::srv::El2809WriteAllChannelsService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::El2809WriteAllChannelsService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2809 Write All Channels Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL2809_WRITE_ALL_CHANNELS_CMD;
  cmd.el2809_write_all_channels_cmd.channel_ch1 = request->channel_ch1;
  cmd.el2809_write_all_channels_cmd.channel_ch2 = request->channel_ch2;
  cmd.el2809_write_all_channels_cmd.channel_ch3 = request->channel_ch3;
  cmd.el2809_write_all_channels_cmd.channel_ch4 = request->channel_ch4;
  cmd.el2809_write_all_channels_cmd.channel_ch5 = request->channel_ch5;
  cmd.el2809_write_all_channels_cmd.channel_ch6 = request->channel_ch6;
  cmd.el2809_write_all_channels_cmd.channel_ch7 = request->channel_ch7;
  cmd.el2809_write_all_channels_cmd.channel_ch8 = request->channel_ch8;
  cmd.el2809_write_all_channels_cmd.channel_ch9 = request->channel_ch9;
  cmd.el2809_write_all_channels_cmd.channel_ch10 = request->channel_ch10;
  cmd.el2809_write_all_channels_cmd.channel_ch11 = request->channel_ch11;
  cmd.el2809_write_all_channels_cmd.channel_ch12 = request->channel_ch12;
  cmd.el2809_write_all_channels_cmd.channel_ch13 = request->channel_ch13;
  cmd.el2809_write_all_channels_cmd.channel_ch14 = request->channel_ch14;
  cmd.el2809_write_all_channels_cmd.channel_ch15 = request->channel_ch15;
  cmd.el2809_write_all_channels_cmd.channel_ch16 = request->channel_ch16;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::EL2809_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2809WriteChannelSrvCb(
  const std::shared_ptr<fcat_msgs::srv::El2809WriteChannelService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::El2809WriteChannelService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2809 Write Channel Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL2809_WRITE_CHANNEL_CMD;
  cmd.el2809_write_channel_cmd.channel = request->channel;
  cmd.el2809_write_channel_cmd.level = request->level;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::EL2809_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2798WriteAllChannelsSrvCb(
    const std::shared_ptr<
        fcat_msgs::srv::El2798WriteAllChannelsService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::El2798WriteAllChannelsService::Response>
        response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2798 Write All Channels Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL2798_WRITE_ALL_CHANNELS_CMD;
  cmd.el2798_write_all_channels_cmd.channel_ch1  = request->channel_ch1;
  cmd.el2798_write_all_channels_cmd.channel_ch2  = request->channel_ch2;
  cmd.el2798_write_all_channels_cmd.channel_ch3  = request->channel_ch3;
  cmd.el2798_write_all_channels_cmd.channel_ch4  = request->channel_ch4;
  cmd.el2798_write_all_channels_cmd.channel_ch5  = request->channel_ch5;
  cmd.el2798_write_all_channels_cmd.channel_ch6  = request->channel_ch6;
  cmd.el2798_write_all_channels_cmd.channel_ch7  = request->channel_ch7;
  cmd.el2798_write_all_channels_cmd.channel_ch8  = request->channel_ch8;

  response->success =
      DeviceExistsOnBus(cmd.name, fastcat::EL2798_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2828WriteAllChannelsSrvCb(
    const std::shared_ptr<
        fcat_msgs::srv::El2828WriteAllChannelsService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::El2828WriteAllChannelsService::Response>
        response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2828 Write All Channels Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL2828_WRITE_ALL_CHANNELS_CMD;
  cmd.el2828_write_all_channels_cmd.channel_ch1  = request->channel_ch1;
  cmd.el2828_write_all_channels_cmd.channel_ch2  = request->channel_ch2;
  cmd.el2828_write_all_channels_cmd.channel_ch3  = request->channel_ch3;
  cmd.el2828_write_all_channels_cmd.channel_ch4  = request->channel_ch4;
  cmd.el2828_write_all_channels_cmd.channel_ch5  = request->channel_ch5;
  cmd.el2828_write_all_channels_cmd.channel_ch6  = request->channel_ch6;
  cmd.el2828_write_all_channels_cmd.channel_ch7  = request->channel_ch7;
  cmd.el2828_write_all_channels_cmd.channel_ch8  = request->channel_ch8;

  response->success =
      DeviceExistsOnBus(cmd.name, fastcat::EL2828_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2798WriteChannelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::El2798WriteChannelService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::El2798WriteChannelService::Response>
        response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2798 Write Channel Command");
  fastcat::DeviceCmd cmd;

  cmd.name                             = request->name;
  cmd.type                             = fastcat::EL2798_WRITE_CHANNEL_CMD;
  cmd.el2798_write_channel_cmd.channel = request->channel;
  cmd.el2798_write_channel_cmd.level   = request->level;

  response->success =
      DeviceExistsOnBus(cmd.name, fastcat::EL2798_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El2828WriteChannelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::El2828WriteChannelService::Request>
        request,
    std::shared_ptr<fcat_msgs::srv::El2828WriteChannelService::Response>
        response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL2828 Write Channel Command");
  fastcat::DeviceCmd cmd;

  cmd.name                             = request->name;
  cmd.type                             = fastcat::EL2828_WRITE_CHANNEL_CMD;
  cmd.el2828_write_channel_cmd.channel = request->channel;
  cmd.el2828_write_channel_cmd.level   = request->level;

  response->success =
      DeviceExistsOnBus(cmd.name, fastcat::EL2828_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El4102WriteAllChannelsSrvCb(
  const std::shared_ptr<fcat_msgs::srv::El4102WriteAllChannelsService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::El4102WriteAllChannelsService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL4102 Write All Channels Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL4102_WRITE_ALL_CHANNELS_CMD;
  cmd.el4102_write_all_channels_cmd.voltage_output_ch1 =
    request->voltage_output_ch1;
  cmd.el4102_write_all_channels_cmd.voltage_output_ch2 =
    request->voltage_output_ch2;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::EL4102_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::El4102WriteChannelSrvCb(
  const std::shared_ptr<fcat_msgs::srv::El4102WriteChannelService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::El4102WriteChannelService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling EL4102 Write Channel Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::EL4102_WRITE_CHANNEL_CMD;
  cmd.el4102_write_channel_cmd.channel = request->channel;
  cmd.el4102_write_channel_cmd.voltage_output = request->voltage_output;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::EL4102_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::FaulterEnableSrvCb(
  const std::shared_ptr<fcat_msgs::srv::FaulterEnableService::Request> request,
  std::shared_ptr<fcat_msgs::srv::FaulterEnableService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Faulter Enable Command");
  fastcat::DeviceCmd cmd;

  cmd.name = request->name;
  cmd.type = fastcat::FAULTER_ENABLE_CMD;
  cmd.faulter_enable_cmd.enable = request->enable;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::FAULTER_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::FtsTareSrvCb(
  const std::shared_ptr<fcat_msgs::srv::DeviceTriggerService::Request> request,
  std::shared_ptr<fcat_msgs::srv::DeviceTriggerService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling FTS Tare Command");
  fastcat::DeviceCmd cmd;
  cmd.name = request->name;
  cmd.type = fastcat::FTS_TARE_CMD;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::FTS_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}

void Fcat::PidActivateSrvCb(
  const std::shared_ptr<fcat_msgs::srv::PidActivateService::Request> request,
  std::shared_ptr<fcat_msgs::srv::PidActivateService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling PID Activate Command");
  fastcat::DeviceCmd cmd;
  cmd.type = fastcat::PID_ACTIVATE_CMD;
  cmd.pid_activate_cmd.setpoint = request->setpoint;
  cmd.pid_activate_cmd.deadband = request->deadband;
  cmd.pid_activate_cmd.persistence_duration = request->persistence_duration;
  cmd.pid_activate_cmd.max_duration = request->max_duration;

  response->success =
    DeviceExistsOnBus(cmd.name, fastcat::PID_STATE, response->message);
  if (response->success) {
    QueueCommand(cmd);
  }
}
