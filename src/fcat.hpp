#ifndef FCAT_HPP_
#define FCAT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "fcat/fcat_node.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <sys/time.h>

#include "fastcat/fastcat.h"
#include "jsd/jsd_print.h"

// Standard Messages
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench.hpp"

// Cmd Messages
#include "fcat_msgs/msg/actuator_calibrate_cmd.hpp"
#include "fcat_msgs/msg/actuator_csp_cmd.hpp"
#include "fcat_msgs/msg/actuator_cst_cmd.hpp"
#include "fcat_msgs/msg/actuator_csv_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_pos_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_torque_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_vel_cmd.hpp"
#include "fcat_msgs/msg/actuator_set_output_position_cmd.hpp"

#include "fcat_msgs/msg/commander_enable_cmd.hpp"
#include "fcat_msgs/msg/commander_disable_cmd.hpp"

#include "fcat_msgs/msg/el2124_write_all_channels_cmd.hpp"
#include "fcat_msgs/msg/el2124_write_channel_cmd.hpp"
#include "fcat_msgs/msg/faulter_enable_cmd.hpp"
#include "fcat_msgs/msg/fts_tare_cmd.hpp"
#include "fcat_msgs/msg/jed_set_cmd_value_cmd.hpp"
#include "fcat_msgs/msg/pid_activate_cmd.hpp"


// State Messages
#include "fcat_msgs/msg/module_state.hpp"

#include "fcat_msgs/msg/actuator_states.hpp"
#include "fcat_msgs/msg/egd_states.hpp"
#include "fcat_msgs/msg/el2124_states.hpp"
#include "fcat_msgs/msg/el3208_states.hpp"
#include "fcat_msgs/msg/el3602_states.hpp"
#include "fcat_msgs/msg/jed_states.hpp"

#include "fcat_msgs/msg/commander_states.hpp"
#include "fcat_msgs/msg/conditional_states.hpp"
#include "fcat_msgs/msg/faulter_states.hpp"
#include "fcat_msgs/msg/filter_states.hpp"
#include "fcat_msgs/msg/function_states.hpp"
#include "fcat_msgs/msg/pid_states.hpp"
#include "fcat_msgs/msg/saturation_states.hpp"
#include "fcat_msgs/msg/schmitt_trigger_states.hpp"
#include "fcat_msgs/msg/signal_generator_states.hpp"

const unsigned int FCAT_PUB_QUEUE_SIZE = 32;
const unsigned int FCAT_SUB_QUEUE_SIZE = 32;

using WrenchPublisher = rclcpp::Publisher<geometry_msgs::msg::Wrench>;

inline double fcat_get_time_sec()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}

fcat_msgs::msg::ActuatorState ActuatorStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::EgdState EgdStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El2124State El2124StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3208State El3208StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::El3602State El3602StateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::JedState JedStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::CommanderState CommanderStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::ConditionalState ConditionalStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FaulterState FaulterStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FilterState FilterStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::FunctionState FunctionStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::PidState PidStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::SaturationState SaturationStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::SchmittTriggerState SchmittTriggerStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);

fcat_msgs::msg::SignalGeneratorState SignalGeneratorStateToMsg(
    std::shared_ptr<const fastcat::DeviceState> state);


class Fcat: public FcatNode {
public:
  ~Fcat();
  Fcat();

private:

  void Process() override;

  void PopulateDeviceStateFields();
  bool DeviceExistsOnBus(std::string name, fastcat::DeviceStateType type);
  bool TypeExistsOnBus(fastcat::DeviceStateType type);
  void InitializePublishersAndMessages();
  void InitializeSubscribers();

  void UpdateStateMap();

  void PublishModuleState();
  void PublishFtsStates();

  void PublishActuatorStates();
  void PublishEgdStates();
  void PublishEl2124States();
  void PublishEl3208States();
  void PublishEl3602States();
  void PublishJedStates();

  void PublishCommanderStates();
  void PublishConditionalStates();
  void PublishFaulterStates();
  void PublishFilterStates();
  void PublishFunctionStates();
  void PublishPidStates();
  void PublishSaturationStates();
  void PublishSchmittTriggerStates();
  void PublishSignalGeneratorStates();

  /////////////////////////////////////
  /////// Commands Callbacks //////////
  /////////////////////////////////////
  
  // bus reset/fault
  void ResetCb( const std::shared_ptr<std_msgs::msg::Empty> msg);
  void FaultCb( const std::shared_ptr<std_msgs::msg::Empty> msg);

  // Actuator
  void ActuatorCSPCmdCb( const std::shared_ptr<fcat_msgs::msg::ActuatorCspCmd> msg);
  void ActuatorCSVCmdCb( const std::shared_ptr<fcat_msgs::msg::ActuatorCsvCmd> msg);
  void ActuatorCSTCmdCb( const std::shared_ptr<fcat_msgs::msg::ActuatorCstCmd> msg);

  void ActuatorCalibrateCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::ActuatorCalibrateCmd> msg);

  void ActuatorProfPosCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::ActuatorProfPosCmd> msg);
  void ActuatorProfTorqueCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::ActuatorProfTorqueCmd> msg);
  void ActuatorProfVelCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::ActuatorProfVelCmd> msg);

  void ActuatorSetOutputPositionCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::ActuatorSetOutputPositionCmd> msg);

  // Commander
  void CommanderEnableCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::CommanderEnableCmd> msg);
  void CommanderDisableCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::CommanderDisableCmd> msg);

  // El2124
  void El2124WriteAllChannelsCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::El2124WriteAllChannelsCmd> msg);
  void El2124WriteChannelCmdCb( 
      const std::shared_ptr<fcat_msgs::msg::El2124WriteChannelCmd> msg);

  // Faulter
  void FaulterEnableCmdCb( const std::shared_ptr<fcat_msgs::msg::FaulterEnableCmd> msg);

  // Fts
  void FtsTareCmdCb( const std::shared_ptr<fcat_msgs::msg::FtsTareCmd> msg);

  // Jed
  void JedSetCmdValueCmdCb( const std::shared_ptr<fcat_msgs::msg::JedSetCmdValueCmd> msg);

  // Pid
  void PidActivateCmdCb( const std::shared_ptr<fcat_msgs::msg::PidActivateCmd> msg);

  //////////////////////////
  // Topic Subscriptions //
  //////////////////////////
  
  // bus reset/fault
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fault_sub_;
 
  // Actuator
  rclcpp::Subscription<fcat_msgs::msg::ActuatorCspCmd>::SharedPtr actuator_csp_sub_;
  rclcpp::Subscription<fcat_msgs::msg::ActuatorCsvCmd>::SharedPtr actuator_csv_sub_;
  rclcpp::Subscription<fcat_msgs::msg::ActuatorCstCmd>::SharedPtr actuator_cst_sub_;

  rclcpp::Subscription<fcat_msgs::msg::ActuatorCalibrateCmd>::SharedPtr 
    actuator_calibrate_sub_;

  rclcpp::Subscription<fcat_msgs::msg::ActuatorProfPosCmd>::SharedPtr 
    actuator_prof_pos_sub_;
  rclcpp::Subscription<fcat_msgs::msg::ActuatorProfTorqueCmd>::SharedPtr 
    actuator_prof_torque_sub_;
  rclcpp::Subscription<fcat_msgs::msg::ActuatorProfVelCmd>::SharedPtr 
    actuator_prof_vel_sub_;

  rclcpp::Subscription<fcat_msgs::msg::ActuatorSetOutputPositionCmd>::SharedPtr 
    actuator_set_output_position_sub_;

  // Commander
  rclcpp::Subscription<fcat_msgs::msg::CommanderEnableCmd>::SharedPtr  commander_enable_sub_;
  rclcpp::Subscription<fcat_msgs::msg::CommanderDisableCmd>::SharedPtr commander_disable_sub_;

  // El2124
  rclcpp::Subscription<fcat_msgs::msg::El2124WriteAllChannelsCmd>::SharedPtr 
    el2124_write_all_channels_sub_;
  rclcpp::Subscription<fcat_msgs::msg::El2124WriteChannelCmd>::SharedPtr 
    el2124_write_channel_sub_;
  
  // Faulter
  rclcpp::Subscription<fcat_msgs::msg::FaulterEnableCmd>::SharedPtr 
    faulter_enable_sub_;

  // Fts
  rclcpp::Subscription<fcat_msgs::msg::FtsTareCmd>::SharedPtr 
    fts_tare_sub_;

  // Jed
  rclcpp::Subscription<fcat_msgs::msg::JedSetCmdValueCmd>::SharedPtr 
    jed_set_cmd_value_sub_;

  // Pid
  rclcpp::Subscription<fcat_msgs::msg::PidActivateCmd>::SharedPtr pid_activate_sub_;
  
  ////////////////
  // Publishers //
  ////////////////
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_; 
  rclcpp::Publisher<fcat_msgs::msg::ModuleState>::SharedPtr  module_state_pub_;

  rclcpp::Publisher<fcat_msgs::msg::ActuatorStates>::SharedPtr actuator_pub_;
  rclcpp::Publisher<fcat_msgs::msg::EgdStates>::SharedPtr      egd_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3602States>::SharedPtr   el3602_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El2124States>::SharedPtr   el2124_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3208States>::SharedPtr   el3208_pub_;
  rclcpp::Publisher<fcat_msgs::msg::JedStates>::SharedPtr      jed_pub_;

  rclcpp::Publisher<fcat_msgs::msg::SaturationStates>::SharedPtr saturation_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FilterStates>::SharedPtr filter_pub_;
  rclcpp::Publisher<fcat_msgs::msg::PidStates>::SharedPtr pid_pub_;
  rclcpp::Publisher<fcat_msgs::msg::CommanderStates>::SharedPtr commander_pub_;
  rclcpp::Publisher<fcat_msgs::msg::SignalGeneratorStates>::SharedPtr signal_generator_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FaulterStates>::SharedPtr faulter_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FunctionStates>::SharedPtr function_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ConditionalStates>::SharedPtr conditional_pub_;
  rclcpp::Publisher<fcat_msgs::msg::SchmittTriggerStates>::SharedPtr schmitt_trigger_pub_;

  std::unordered_map<std::string, WrenchPublisher::SharedPtr>  fts_raw_pub_map_;
  std::unordered_map<std::string, WrenchPublisher::SharedPtr>  fts_tared_pub_map_;


  ////////////////////
  // ROS Parameters //
  ////////////////////

  bool enable_js_pub_;

  ////////////
  // fields //
  ////////////

  std::vector<std::shared_ptr<const fastcat::DeviceState>> device_state_ptrs_;
  std::unordered_map<std::string,std::shared_ptr<const fastcat::DeviceState>> device_name_state_map_;
  std::unordered_map<fastcat::DeviceStateType, std::vector<std::shared_ptr<const fastcat::DeviceState>>> device_type_vec_map_;

  double loop_period_sec_;
  double last_time_;

  fastcat::Manager fcat_manager_;

  fcat_msgs::msg::ActuatorStates actuator_states_msg_;
  fcat_msgs::msg::EgdStates      egd_states_msg_;
  fcat_msgs::msg::El2124States   el2124_states_msg_;
  fcat_msgs::msg::El3208States   el3208_states_msg_;
  fcat_msgs::msg::El3602States   el3602_states_msg_;
  fcat_msgs::msg::JedStates      jed_states_msg_;

  fcat_msgs::msg::CommanderStates       commander_states_msg_;
  fcat_msgs::msg::ConditionalStates     conditional_states_msg_;
  fcat_msgs::msg::FaulterStates         faulter_states_msg_;
  fcat_msgs::msg::FilterStates          filter_states_msg_;
  fcat_msgs::msg::FunctionStates        function_states_msg_;
  fcat_msgs::msg::PidStates             pid_states_msg_;
  fcat_msgs::msg::SaturationStates      saturation_states_msg_;
  fcat_msgs::msg::SchmittTriggerStates  schmitt_trigger_states_msg_;
  fcat_msgs::msg::SignalGeneratorStates signal_generator_states_msg_;

};
#endif
