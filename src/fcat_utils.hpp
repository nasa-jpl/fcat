// Copyright 2021 California Institute of Technology

#ifndef FCAT_UTILS_HPP_
#define FCAT_UTILS_HPP_

#include <memory>
#include <string>

#include "fastcat/fastcat.h"
#include "fcat_msgs/msg/actuator_states.hpp"
#include "fcat_msgs/msg/commander_states.hpp"
#include "fcat_msgs/msg/conditional_states.hpp"
#include "fcat_msgs/msg/egd_states.hpp"
#include "fcat_msgs/msg/el1008_states.hpp"
#include "fcat_msgs/msg/el2124_states.hpp"
#include "fcat_msgs/msg/el2798_states.hpp"
#include "fcat_msgs/msg/el2809_states.hpp"
#include "fcat_msgs/msg/el2828_states.hpp"
#include "fcat_msgs/msg/el3104_states.hpp"
#include "fcat_msgs/msg/el3162_states.hpp"
#include "fcat_msgs/msg/el3202_states.hpp"
#include "fcat_msgs/msg/el3208_states.hpp"
#include "fcat_msgs/msg/el3314_states.hpp"
#include "fcat_msgs/msg/el3318_states.hpp"
#include "fcat_msgs/msg/el3602_states.hpp"
#include "fcat_msgs/msg/el4102_states.hpp"
#include "fcat_msgs/msg/el5042_states.hpp"
#include "fcat_msgs/msg/faulter_states.hpp"
#include "fcat_msgs/msg/filter_states.hpp"
#include "fcat_msgs/msg/fts_states.hpp"
#include "fcat_msgs/msg/function_states.hpp"
#include "fcat_msgs/msg/ild1900_states.hpp"
#include "fcat_msgs/msg/linear_interpolation_states.hpp"
#include "fcat_msgs/msg/pid_states.hpp"
#include "fcat_msgs/msg/saturation_states.hpp"
#include "fcat_msgs/msg/schmitt_trigger_states.hpp"
#include "fcat_msgs/msg/signal_generator_states.hpp"
#include "fcat_msgs/msg/three_node_thermal_model_states.hpp"

fcat_msgs::msg::ActuatorState ActuatorStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::EgdState EgdStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El1008State El1008StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El2124State El2124StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El2809State El2809StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El2798State El2798StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El2828State El2828StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3104State El3104StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3162State El3162StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3202State El3202StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3208State El3208StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3314State El3314StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3318State El3318StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3602State El3602StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El4102State El4102StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El5042State El5042StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::Ild1900State Ild1900StateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::CommanderState CommanderStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::ConditionalState ConditionalStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FaulterState FaulterStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FilterState FilterStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FtsState FtsStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FunctionState FunctionStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::PidState PidStateToMsg(std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::SaturationState SaturationStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::SchmittTriggerState SchmittTriggerStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::SignalGeneratorState SignalGeneratorStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::LinearInterpolationState LinearInterpolationStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::ThreeNodeThermalModelState ThreeNodeThermalModelStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

bool HexOrDecStrToNum(std::string& str, uint16_t& number);
bool TlcStrToNum(std::string& str, uint16_t& number);

jsd_sdo_data_type_t jsd_sdo_data_type_from_string(std::string& str);

jsd_sdo_data_t jsd_sdo_data_from_string(jsd_sdo_data_type_t& type, std::string& str);

std::string jsd_sdo_request_type_to_string(jsd_sdo_req_type_t req_type);
std::string jsd_sdo_data_type_to_string(jsd_sdo_data_type_t data_type);
std::string jsd_sdo_data_to_string(jsd_sdo_data_type_t data_type, jsd_sdo_data_t data);

#endif  // FCAT_UTILS_HPP_
