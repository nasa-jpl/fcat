#include "fcat_utils.hpp"

#include "jsd/jsd_print.h"

#include <cstdio>

fcat_msgs::msg::ActuatorState ActuatorStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::ActuatorState();

  if (state->type == fastcat::GOLD_ACTUATOR_STATE) {
    msg.read_time = state->time;
    msg.actual_position = state->gold_actuator_state.actual_position;
    msg.actual_velocity = state->gold_actuator_state.actual_velocity;
    msg.actual_current = state->gold_actuator_state.actual_current;
    msg.faulted = state->gold_actuator_state.faulted;
    msg.cmd_position = state->gold_actuator_state.cmd_position;
    msg.cmd_velocity = state->gold_actuator_state.cmd_velocity;
    msg.cmd_current = state->gold_actuator_state.cmd_current;
    msg.cmd_max_current = state->gold_actuator_state.cmd_max_current;
    msg.egd_state_machine_state =
      state->gold_actuator_state.elmo_state_machine_state;
    msg.egd_mode_of_operation =
      state->gold_actuator_state.elmo_mode_of_operation;
    msg.sto_engaged = state->gold_actuator_state.sto_engaged;
    msg.hall_state = state->gold_actuator_state.hall_state;
    msg.target_reached = state->gold_actuator_state.target_reached;
    msg.motor_on = state->gold_actuator_state.motor_on;
    msg.servo_enabled = state->gold_actuator_state.servo_enabled;
    msg.jsd_fault_code = state->gold_actuator_state.jsd_fault_code;
    msg.fastcat_fault_code = state->gold_actuator_state.fastcat_fault_code;
    msg.emcy_error_code = state->gold_actuator_state.emcy_error_code;
    msg.bus_voltage = state->gold_actuator_state.bus_voltage;
    msg.drive_temperature = state->gold_actuator_state.drive_temperature;
    msg.egd_actual_position = state->gold_actuator_state.elmo_actual_position;
    msg.egd_cmd_position = state->gold_actuator_state.elmo_cmd_position;
    msg.actuator_state_machine_state =
      state->gold_actuator_state.actuator_state_machine_state;
  } else if (state->type == fastcat::PLATINUM_ACTUATOR_STATE) {
    msg.read_time = state->time;
    msg.actual_position = state->platinum_actuator_state.actual_position;
    msg.actual_velocity = state->platinum_actuator_state.actual_velocity;
    msg.actual_current = state->platinum_actuator_state.actual_current;
    msg.faulted = state->platinum_actuator_state.faulted;
    msg.cmd_position = state->platinum_actuator_state.cmd_position;
    msg.cmd_velocity = state->platinum_actuator_state.cmd_velocity;
    msg.cmd_current = state->platinum_actuator_state.cmd_current;
    msg.cmd_max_current = state->platinum_actuator_state.cmd_max_current;
    msg.egd_state_machine_state =
      state->platinum_actuator_state.elmo_state_machine_state;
    msg.egd_mode_of_operation =
      state->platinum_actuator_state.elmo_mode_of_operation;
    msg.sto_engaged = state->platinum_actuator_state.sto_engaged;
    msg.hall_state = state->platinum_actuator_state.hall_state;
    msg.target_reached = state->platinum_actuator_state.target_reached;
    msg.motor_on = state->platinum_actuator_state.motor_on;
    msg.servo_enabled = state->platinum_actuator_state.servo_enabled;
    msg.jsd_fault_code = state->platinum_actuator_state.jsd_fault_code;
    msg.fastcat_fault_code = state->platinum_actuator_state.fastcat_fault_code;
    msg.emcy_error_code = state->platinum_actuator_state.emcy_error_code;
    msg.bus_voltage = state->platinum_actuator_state.bus_voltage;
    msg.drive_temperature = state->platinum_actuator_state.drive_temperature;
    msg.egd_actual_position =
      state->platinum_actuator_state.elmo_actual_position;
    msg.egd_cmd_position = state->platinum_actuator_state.elmo_cmd_position;
    msg.actuator_state_machine_state =
      state->platinum_actuator_state.actuator_state_machine_state;
  }

  return msg;
}

fcat_msgs::msg::EgdState EgdStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::EgdState();

  msg.read_time = state->time;
  msg.actual_position = state->egd_state.actual_position;
  msg.actual_velocity = state->egd_state.actual_velocity;
  msg.actual_current = state->egd_state.actual_current;
  msg.faulted = state->egd_state.faulted;
  msg.cmd_position = state->egd_state.cmd_position;
  msg.cmd_velocity = state->egd_state.cmd_velocity;
  msg.cmd_current = state->egd_state.cmd_current;
  msg.cmd_max_current = state->egd_state.cmd_max_current;
  msg.cmd_ff_position = state->egd_state.cmd_ff_position;
  msg.cmd_ff_velocity = state->egd_state.cmd_ff_velocity;
  msg.cmd_ff_current = state->egd_state.cmd_ff_current;
  msg.actual_state_machine_state = state->egd_state.actual_state_machine_state;
  msg.actual_mode_of_operation = state->egd_state.actual_mode_of_operation;
  msg.sto_engaged = state->egd_state.sto_engaged;
  msg.hall_state = state->egd_state.hall_state;
  msg.in_motion = state->egd_state.in_motion;
  msg.warning = state->egd_state.warning;
  msg.target_reached = state->egd_state.target_reached;
  msg.motor_on = state->egd_state.motor_on;
  msg.fault_code = state->egd_state.fault_code;
  msg.emcy_error_code = state->egd_state.emcy_error_code;
  msg.bus_voltage = state->egd_state.bus_voltage;
  msg.analog_input_voltage = state->egd_state.analog_input_voltage;
  msg.digital_input_ch1 = state->egd_state.digital_input_ch1;
  msg.digital_input_ch2 = state->egd_state.digital_input_ch2;
  msg.digital_input_ch3 = state->egd_state.digital_input_ch3;
  msg.digital_input_ch4 = state->egd_state.digital_input_ch4;
  msg.digital_input_ch5 = state->egd_state.digital_input_ch5;
  msg.digital_input_ch6 = state->egd_state.digital_input_ch6;
  msg.digital_output_cmd_ch1 = state->egd_state.digital_output_cmd_ch1;
  msg.digital_output_cmd_ch2 = state->egd_state.digital_output_cmd_ch2;
  msg.digital_output_cmd_ch3 = state->egd_state.digital_output_cmd_ch3;
  msg.digital_output_cmd_ch4 = state->egd_state.digital_output_cmd_ch4;
  msg.digital_output_cmd_ch5 = state->egd_state.digital_output_cmd_ch5;
  msg.digital_output_cmd_ch6 = state->egd_state.digital_output_cmd_ch6;
  msg.drive_temperature = state->egd_state.drive_temperature;

  return msg;
}

fcat_msgs::msg::El1008State El1008StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El1008State();

  msg.read_time = state->time;
  msg.level_ch1 = state->el1008_state.level_ch1;
  msg.level_ch2 = state->el1008_state.level_ch2;
  msg.level_ch3 = state->el1008_state.level_ch3;
  msg.level_ch4 = state->el1008_state.level_ch4;
  msg.level_ch5 = state->el1008_state.level_ch5;
  msg.level_ch6 = state->el1008_state.level_ch6;
  msg.level_ch7 = state->el1008_state.level_ch7;
  msg.level_ch8 = state->el1008_state.level_ch8;

  return msg;
}

fcat_msgs::msg::El2124State El2124StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El2124State();

  msg.read_time = state->time;
  msg.level_ch1 = state->el2124_state.level_ch1;
  msg.level_ch2 = state->el2124_state.level_ch2;
  msg.level_ch3 = state->el2124_state.level_ch3;
  msg.level_ch4 = state->el2124_state.level_ch4;

  return msg;
}

fcat_msgs::msg::El2809State El2809StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El2809State();

  msg.read_time = state->time;
  msg.level_ch1 = state->el2809_state.level_ch1;
  msg.level_ch2 = state->el2809_state.level_ch2;
  msg.level_ch3 = state->el2809_state.level_ch3;
  msg.level_ch4 = state->el2809_state.level_ch4;
  msg.level_ch5 = state->el2809_state.level_ch5;
  msg.level_ch6 = state->el2809_state.level_ch6;
  msg.level_ch7 = state->el2809_state.level_ch7;
  msg.level_ch8 = state->el2809_state.level_ch8;
  msg.level_ch9 = state->el2809_state.level_ch9;
  msg.level_ch10 = state->el2809_state.level_ch10;
  msg.level_ch11 = state->el2809_state.level_ch11;
  msg.level_ch12 = state->el2809_state.level_ch12;
  msg.level_ch13 = state->el2809_state.level_ch13;
  msg.level_ch14 = state->el2809_state.level_ch14;
  msg.level_ch15 = state->el2809_state.level_ch15;
  msg.level_ch16 = state->el2809_state.level_ch16;

  return msg;
}

fcat_msgs::msg::El2798State El2798StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El2798State();

  msg.read_time  = state->time;
  msg.level_ch1  = state->el2798_state.level_ch1;
  msg.level_ch2  = state->el2798_state.level_ch2;
  msg.level_ch3  = state->el2798_state.level_ch3;
  msg.level_ch4  = state->el2798_state.level_ch4;
  msg.level_ch5  = state->el2798_state.level_ch5;
  msg.level_ch6  = state->el2798_state.level_ch6;
  msg.level_ch7  = state->el2798_state.level_ch7;
  msg.level_ch8  = state->el2798_state.level_ch8;

  return msg;
}

fcat_msgs::msg::El2828State El2828StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El2828State();

  msg.read_time  = state->time;
  msg.level_ch1  = state->el2828_state.level_ch1;
  msg.level_ch2  = state->el2828_state.level_ch2;
  msg.level_ch3  = state->el2828_state.level_ch3;
  msg.level_ch4  = state->el2828_state.level_ch4;
  msg.level_ch5  = state->el2828_state.level_ch5;
  msg.level_ch6  = state->el2828_state.level_ch6;
  msg.level_ch7  = state->el2828_state.level_ch7;
  msg.level_ch8  = state->el2828_state.level_ch8;

  return msg;
}

fcat_msgs::msg::El3104State El3104StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3104State();

  msg.read_time = state->time;
  msg.voltage_ch1 = state->el3104_state.voltage_ch1;
  msg.voltage_ch2 = state->el3104_state.voltage_ch2;
  msg.voltage_ch3 = state->el3104_state.voltage_ch3;
  msg.voltage_ch4 = state->el3104_state.voltage_ch4;
  msg.adc_value_ch1 = state->el3104_state.adc_value_ch1;
  msg.adc_value_ch2 = state->el3104_state.adc_value_ch2;
  msg.adc_value_ch3 = state->el3104_state.adc_value_ch3;
  msg.adc_value_ch4 = state->el3104_state.adc_value_ch4;

  return msg;
}

fcat_msgs::msg::El3162State El3162StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3162State();

  msg.read_time = state->time;
  msg.voltage_ch1 = state->el3162_state.voltage_ch1;
  msg.adc_value_ch1 = state->el3162_state.adc_value_ch1;
  msg.voltage_ch2 = state->el3162_state.voltage_ch2;
  msg.adc_value_ch2 = state->el3162_state.adc_value_ch2;

  return msg;
}

fcat_msgs::msg::El3202State El3202StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3202State();

  msg.read_time = state->time;
  msg.output_ch1 = state->el3202_state.output_eu_ch1;
  msg.adc_value_ch1 = state->el3202_state.adc_value_ch1;
  msg.output_ch2 = state->el3202_state.output_eu_ch2;
  msg.adc_value_ch2 = state->el3202_state.adc_value_ch2;

  return msg;
}

fcat_msgs::msg::El3208State El3208StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3208State();

  msg.read_time = state->time;
  msg.output_ch1 = state->el3208_state.output_ch1;
  msg.adc_value_ch1 = state->el3208_state.adc_value_ch1;
  msg.output_ch2 = state->el3208_state.output_ch2;
  msg.adc_value_ch2 = state->el3208_state.adc_value_ch2;
  msg.output_ch3 = state->el3208_state.output_ch3;
  msg.adc_value_ch3 = state->el3208_state.adc_value_ch3;
  msg.output_ch4 = state->el3208_state.output_ch4;
  msg.adc_value_ch4 = state->el3208_state.adc_value_ch4;
  msg.output_ch5 = state->el3208_state.output_ch5;
  msg.adc_value_ch5 = state->el3208_state.adc_value_ch5;
  msg.output_ch6 = state->el3208_state.output_ch6;
  msg.adc_value_ch6 = state->el3208_state.adc_value_ch6;
  msg.output_ch7 = state->el3208_state.output_ch7;
  msg.adc_value_ch7 = state->el3208_state.adc_value_ch7;
  msg.output_ch8 = state->el3208_state.output_ch8;
  msg.adc_value_ch8 = state->el3208_state.adc_value_ch8;

  return msg;
}

fcat_msgs::msg::El3314State El3314StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3314State();

  msg.read_time = state->time;
  msg.output_ch1 = state->el3314_state.output_eu_ch1;
  msg.adc_value_ch1 = state->el3314_state.adc_value_ch1;
  msg.output_ch2 = state->el3314_state.output_eu_ch2;
  msg.adc_value_ch2 = state->el3314_state.adc_value_ch2;
  msg.output_ch3 = state->el3314_state.output_eu_ch3;
  msg.adc_value_ch3 = state->el3314_state.adc_value_ch3;
  msg.output_ch4 = state->el3314_state.output_eu_ch4;
  msg.adc_value_ch4 = state->el3314_state.adc_value_ch4;

  return msg;
}

fcat_msgs::msg::El3318State El3318StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3318State();

  msg.read_time = state->time;
  msg.output_ch1 = state->el3318_state.output_eu_ch1;
  msg.adc_value_ch1 = state->el3318_state.adc_value_ch1;
  msg.output_ch2 = state->el3318_state.output_eu_ch2;
  msg.adc_value_ch2 = state->el3318_state.adc_value_ch2;
  msg.output_ch3 = state->el3318_state.output_eu_ch3;
  msg.adc_value_ch3 = state->el3318_state.adc_value_ch3;
  msg.output_ch4 = state->el3318_state.output_eu_ch4;
  msg.adc_value_ch4 = state->el3318_state.adc_value_ch4;
  msg.output_ch5 = state->el3318_state.output_eu_ch5;
  msg.adc_value_ch5 = state->el3318_state.adc_value_ch5;
  msg.output_ch6 = state->el3318_state.output_eu_ch6;
  msg.adc_value_ch6 = state->el3318_state.adc_value_ch6;
  msg.output_ch7 = state->el3318_state.output_eu_ch7;
  msg.adc_value_ch7 = state->el3318_state.adc_value_ch7;
  msg.output_ch8 = state->el3318_state.output_eu_ch8;
  msg.adc_value_ch8 = state->el3318_state.adc_value_ch8;

  return msg;
}

fcat_msgs::msg::El3602State El3602StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El3602State();

  msg.read_time = state->time;
  msg.voltage_ch1 = state->el3602_state.voltage_ch1;
  msg.adc_value_ch1 = state->el3602_state.adc_value_ch1;
  msg.voltage_ch2 = state->el3602_state.voltage_ch2;
  msg.adc_value_ch2 = state->el3602_state.adc_value_ch2;

  return msg;
}

fcat_msgs::msg::El4102State El4102StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El4102State();

  msg.read_time = state->time;
  msg.voltage_output_ch1 = state->el4102_state.voltage_output_ch1;
  msg.voltage_output_ch2 = state->el4102_state.voltage_output_ch2;

  return msg;
}

fcat_msgs::msg::El5042State El5042StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::El5042State();

  msg.read_time    = state->time;
  msg.position_ch1 = state->el5042_state.position_ch1;
  msg.warning_ch1  = state->el5042_state.warning_ch1;
  msg.error_ch1    = state->el5042_state.error_ch1;
  msg.ready_ch1    = state->el5042_state.ready_ch1;
  msg.position_ch2 = state->el5042_state.position_ch2;
  msg.warning_ch2  = state->el5042_state.warning_ch2;
  msg.error_ch2    = state->el5042_state.error_ch2;
  msg.ready_ch2    = state->el5042_state.ready_ch2;

  return msg;
}

fcat_msgs::msg::Ild1900State Ild1900StateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::Ild1900State();

  msg.read_time = state->time;
  msg.distance_m = state->ild1900_state.distance_m;
  msg.intensity = state->ild1900_state.intensity;
  msg.distance_raw = state->ild1900_state.distance_raw;
  msg.sensor_timestamp_us = state->ild1900_state.timestamp_us;
  msg.counter = state->ild1900_state.counter;
  msg.error = state->ild1900_state.error;

  return msg;
}

fcat_msgs::msg::CommanderState CommanderStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::CommanderState();

  msg.read_time = state->time;
  msg.enable = state->commander_state.enable;

  return msg;
}

fcat_msgs::msg::ConditionalState ConditionalStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::ConditionalState();

  msg.read_time = state->time;
  msg.output = state->conditional_state.output;

  return msg;
}

fcat_msgs::msg::FaulterState FaulterStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FaulterState();

  msg.read_time = state->time;
  msg.enable = state->faulter_state.enable;
  msg.fault_active = state->faulter_state.fault_active;

  return msg;
}

fcat_msgs::msg::FilterState FilterStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FilterState();

  msg.read_time = state->time;
  msg.output = state->filter_state.output;

  return msg;
}

fcat_msgs::msg::FtsState FtsStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FtsState();

  msg.read_time = state->time;
  msg.raw_fx = state->fts_state.raw_fx;
  msg.raw_fy = state->fts_state.raw_fy;
  msg.raw_fz = state->fts_state.raw_fz;
  msg.raw_tx = state->fts_state.raw_tx;
  msg.raw_ty = state->fts_state.raw_ty;
  msg.raw_tz = state->fts_state.raw_tz;
  msg.tared_fx = state->fts_state.tared_fx;
  msg.tared_fy = state->fts_state.tared_fy;
  msg.tared_fz = state->fts_state.tared_fz;
  msg.tared_tx = state->fts_state.tared_tx;
  msg.tared_ty = state->fts_state.tared_ty;
  msg.tared_tz = state->fts_state.tared_tz;

  return msg;
}

fcat_msgs::msg::FunctionState FunctionStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::FunctionState();

  msg.read_time = state->time;
  msg.output = state->function_state.output;

  return msg;
}

fcat_msgs::msg::PidState PidStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::PidState();

  msg.read_time = state->time;
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

  msg.read_time = state->time;
  msg.output = state->saturation_state.output;

  return msg;
}

fcat_msgs::msg::SchmittTriggerState SchmittTriggerStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::SchmittTriggerState();

  msg.read_time = state->time;
  msg.output = state->schmitt_trigger_state.output;

  return msg;
}

fcat_msgs::msg::SignalGeneratorState SignalGeneratorStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::SignalGeneratorState();

  msg.read_time = state->time;
  msg.output = state->signal_generator_state.output;

  return msg;
}

fcat_msgs::msg::LinearInterpolationState LinearInterpolationStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::LinearInterpolationState();

  msg.read_time = state->time;
  msg.output = state->linear_interpolation_state.output;
  msg.is_saturated = state->linear_interpolation_state.is_saturated;

  return msg;
}

fcat_msgs::msg::ThreeNodeThermalModelState ThreeNodeThermalModelStateToMsg(
  std::shared_ptr<const fastcat::DeviceState> state)
{
  auto msg = fcat_msgs::msg::ThreeNodeThermalModelState();

  msg.read_time = state->time;
  msg.node_1_temp = state->three_node_thermal_model_state.node_1_temp;
  msg.node_2_temp = state->three_node_thermal_model_state.node_2_temp;
  msg.node_3_temp = state->three_node_thermal_model_state.node_3_temp;
  msg.node_4_temp = state->three_node_thermal_model_state.node_4_temp;

  return msg;
}

bool HexOrDecStrToNum(std::string& str, uint16_t& number)
{
  char* end;
  bool success = false;

  unsigned long int ulnum = strtoul(str.c_str(), &end, 0);

  if (*end == '\0') {
    fprintf(
      stderr,
      "HexOrDecStrToNum successfully parsed string (%s) to (%lu or 0x%lx)",
      str.c_str(), ulnum, ulnum);
    fprintf(stderr, "\n");
    number = static_cast<uint16_t>(ulnum);
    success = true;

  } else {
    fprintf(stderr, "HexOrDecStrToNum could not parse string (%s)\n", str.c_str());
  }

  return success;
}

bool TlcStrToNum(std::string& str, uint16_t& number)
{
  if (str.size() != 2) {
    fprintf(stderr, "TLC string: (%s) must be 2 chars. size: (%zu)\n",
            str.c_str(), str.size());
    return false;
  }

  if ((str[0] < 'A') || (str[0] > 'Z') || (str[1] < 'A') || (str[1] > 'Z')) {
    fprintf(stderr, "TLC string: (%s) must be upper chars [A-Z]\n",
            str.c_str());
    return false;
  }

  number = 0x3000 + (26 * (str[0] - 65)) + (str[1] - 65);

  if (number < 0x3000 || number > 0x3FFF) {
    fprintf(
      stderr,
      "TLC conversion is out of range: %s -> 0x%X not in "
      "range (0x3000,0x3FFF)",
      str.c_str(), number);
    fprintf(stderr, "\n");
    return false;
  }

  fprintf(stderr, "Converted TLC string: (%s) to U16: (0x%X)\n",
          str.c_str(), number);
  return true;
}

jsd_sdo_data_type_t jsd_sdo_data_type_from_string(std::string& str)
{
  jsd_sdo_data_type_t type = {};

  if (0 == str.compare("I8")) {
    type = JSD_SDO_DATA_I8;
  } else if (0 == str.compare("I16")) {
    type = JSD_SDO_DATA_I16;
  } else if (0 == str.compare("I32")) {
    type = JSD_SDO_DATA_I32;
  } else if (0 == str.compare("I64")) {
    type = JSD_SDO_DATA_I64;
  } else if (0 == str.compare("F32")) {
    type = JSD_SDO_DATA_FLOAT;
  } else if (0 == str.compare("U8")) {
    type = JSD_SDO_DATA_U8;
  } else if (0 == str.compare("U16")) {
    type = JSD_SDO_DATA_U16;
  } else if (0 == str.compare("U32")) {
    type = JSD_SDO_DATA_U32;
  } else if (0 == str.compare("U64")) {
    type = JSD_SDO_DATA_U64;
  } else {
    type = JSD_SDO_DATA_UNSPECIFIED;
  }

  return type;
}

jsd_sdo_data_t jsd_sdo_data_from_string(jsd_sdo_data_type_t& type,
                                        std::string& str)
{
  jsd_sdo_data_t data = {};

  switch (type) {
    case JSD_SDO_DATA_I8:
      data.as_i8 = static_cast<int8_t>(atoi(str.c_str()));
      break;
    case JSD_SDO_DATA_I16:
      data.as_i16 = static_cast<int16_t>(atoi(str.c_str()));
      break;
    case JSD_SDO_DATA_I32:
      data.as_i32 = static_cast<int32_t>(atol(str.c_str()));
      break;
    case JSD_SDO_DATA_I64:
      data.as_i64 = static_cast<int64_t>(atoll(str.c_str()));
      break;
    case JSD_SDO_DATA_FLOAT:
      data.as_float = static_cast<float>(atof(str.c_str()));
      break;
    case JSD_SDO_DATA_U8:
      data.as_u8 = static_cast<uint8_t>(atoi(str.c_str()));
      break;
    case JSD_SDO_DATA_U16:
      data.as_u16 = static_cast<uint16_t>(atol(str.c_str()));
      break;
    case JSD_SDO_DATA_U32:
      data.as_u32 = static_cast<uint32_t>(atoll(str.c_str()));
      break;
    case JSD_SDO_DATA_U64:
      data.as_u64 = static_cast<uint64_t>(atoll(str.c_str()));
      break;
    default:
      fprintf(stderr, "SDO data type: %d\n", static_cast<int>(type));
  }
  return data;
}

std::string jsd_sdo_request_type_to_string(jsd_sdo_req_type_t req_type)
{
  switch (req_type) {
    case JSD_SDO_REQ_TYPE_READ:
      return "READ";
      break;

    case JSD_SDO_REQ_TYPE_WRITE:
      return "WRITE";
      break;

    default:
      fprintf(stderr, "Invalid request type: %d\n", static_cast<int>(req_type));
  }
  return "INVALID";
}

std::string jsd_sdo_data_type_to_string(jsd_sdo_data_type_t data_type)
{
  switch (data_type) {
    case JSD_SDO_DATA_I8:
      return "I8";
      break;
    case JSD_SDO_DATA_I16:
      return "I16";
      break;
    case JSD_SDO_DATA_I32:
      return "I32";
      break;
    case JSD_SDO_DATA_I64:
      return "I64";
      break;
    case JSD_SDO_DATA_FLOAT:
      return "F32";
      break;
    case JSD_SDO_DATA_U8:
      return "U8";
      break;
    case JSD_SDO_DATA_U16:
      return "U16";
      break;
    case JSD_SDO_DATA_U32:
      return "U32";
      break;
    case JSD_SDO_DATA_U64:
      return "U64";
      break;
    default:
      fprintf(stderr, "Invalid data type: %d\n", static_cast<int>(data_type));
  }
  return "UNSPECIFIED";
}

std::string jsd_sdo_data_to_string(jsd_sdo_data_type_t data_type,
                                   jsd_sdo_data_t data)
{
  switch (data_type) {
    case JSD_SDO_DATA_I8:
      return std::to_string(data.as_i8);
      break;
    case JSD_SDO_DATA_I16:
      return std::to_string(data.as_i16);
      break;
    case JSD_SDO_DATA_I32:
      return std::to_string(data.as_i32);
      break;
    case JSD_SDO_DATA_I64:
      return std::to_string(data.as_i64);
      break;
    case JSD_SDO_DATA_FLOAT:
      return std::to_string(data.as_float);
      break;
    case JSD_SDO_DATA_U8:
      return std::to_string(data.as_u8);
      break;
    case JSD_SDO_DATA_U16:
      return std::to_string(data.as_u16);
      break;
    case JSD_SDO_DATA_U32:
      return std::to_string(data.as_u32);
      break;
    case JSD_SDO_DATA_U64:
      return std::to_string(data.as_u64);
      break;
    default:
      fprintf(stderr, "Bad data type\n");
  }
  return "invalid result";
}
