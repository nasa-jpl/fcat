#ifndef FCAT__FCAT_HPP_
#define FCAT__FCAT_HPP_

#include "casah_node/fault_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include <sys/time.h>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "fastcat/fastcat.h"
#include "jsd/jsd_print.h"

// Standard Messages
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

// Instant Response services
#include "fcat_msgs/srv/actuator_calibrate_service.hpp"
#include "fcat_msgs/srv/actuator_halt_service.hpp"
#include "fcat_msgs/srv/actuator_prof_pos_service.hpp"
#include "fcat_msgs/srv/actuator_prof_torque_service.hpp"
#include "fcat_msgs/srv/actuator_prof_vel_service.hpp"
#include "fcat_msgs/srv/actuator_set_digital_output_service.hpp"
#include "fcat_msgs/srv/actuator_set_gain_scheduling_index_service.hpp"
#include "fcat_msgs/srv/actuator_set_max_current_service.hpp"
#include "fcat_msgs/srv/actuator_set_output_position_service.hpp"
#include "fcat_msgs/srv/actuator_set_prof_disengaging_timeout_service.hpp"
#include "fcat_msgs/srv/commander_disable_service.hpp"
#include "fcat_msgs/srv/commander_enable_service.hpp"
#include "fcat_msgs/srv/device_trigger_service.hpp"
#include "fcat_msgs/srv/el2124_write_all_channels_service.hpp"
#include "fcat_msgs/srv/el2124_write_channel_service.hpp"
#include "fcat_msgs/srv/el2809_write_all_channels_service.hpp"
#include "fcat_msgs/srv/el2809_write_channel_service.hpp"
#include "fcat_msgs/srv/el2798_write_all_channels_service.hpp"
#include "fcat_msgs/srv/el2798_write_channel_service.hpp"
#include "fcat_msgs/srv/el2828_write_all_channels_service.hpp"
#include "fcat_msgs/srv/el2828_write_channel_service.hpp"
#include "fcat_msgs/srv/el4102_write_all_channels_service.hpp"
#include "fcat_msgs/srv/el4102_write_channel_service.hpp"
#include "fcat_msgs/srv/faulter_enable_service.hpp"
#include "fcat_msgs/srv/pid_activate_service.hpp"

// Cmd Messages
#include "fcat_msgs/msg/async_sdo_read_cmd.hpp"
#include "fcat_msgs/msg/async_sdo_write_cmd.hpp"

#include "fcat_msgs/msg/actuator_calibrate_cmd.hpp"
#include "fcat_msgs/msg/actuator_csp_cmd.hpp"
#include "fcat_msgs/msg/actuator_csp_cmds.hpp"
#include "fcat_msgs/msg/actuator_cst_cmd.hpp"
#include "fcat_msgs/msg/actuator_cst_cmds.hpp"
#include "fcat_msgs/msg/actuator_csv_cmd.hpp"
#include "fcat_msgs/msg/actuator_csv_cmds.hpp"
#include "fcat_msgs/msg/actuator_halt_cmd.hpp"
#include "fcat_msgs/msg/actuator_halt_cmds.hpp"
#include "fcat_msgs/msg/actuator_prof_pos_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_pos_cmds.hpp"
#include "fcat_msgs/msg/actuator_prof_torque_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_vel_cmd.hpp"
#include "fcat_msgs/msg/actuator_set_digital_output_cmd.hpp"
#include "fcat_msgs/msg/actuator_set_max_current_cmd.hpp"
#include "fcat_msgs/msg/actuator_set_output_position_cmd.hpp"
#include "fcat_msgs/msg/actuator_set_prof_disengaging_timeout_cmd.hpp"
#include "fcat_msgs/msg/actuator_set_unit_mode_cmd.hpp"

#include "fcat_msgs/msg/commander_disable_cmd.hpp"
#include "fcat_msgs/msg/commander_enable_cmd.hpp"

#include "fcat_msgs/msg/el2124_write_all_channels_cmd.hpp"
#include "fcat_msgs/msg/el2124_write_channel_cmd.hpp"
#include "fcat_msgs/msg/el2809_write_all_channels_cmd.hpp"
#include "fcat_msgs/msg/el2809_write_channel_cmd.hpp"
#include "fcat_msgs/msg/el2798_write_all_channels_cmd.hpp"
#include "fcat_msgs/msg/el2798_write_channel_cmd.hpp"
#include "fcat_msgs/msg/el2828_write_all_channels_cmd.hpp"
#include "fcat_msgs/msg/el2828_write_channel_cmd.hpp"
#include "fcat_msgs/msg/el4102_write_all_channels_cmd.hpp"
#include "fcat_msgs/msg/el4102_write_channel_cmd.hpp"
#include "fcat_msgs/msg/faulter_enable_cmd.hpp"
#include "fcat_msgs/msg/fts_tare_cmd.hpp"
#include "fcat_msgs/msg/pid_activate_cmd.hpp"

// State Messages
#include "fcat_msgs/msg/async_sdo_response.hpp"
#include "fcat_msgs/msg/module_state.hpp"

#include "fcat/fcat_utils.hpp"

using WrenchPublisher = rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>;

class Fcat : public casah_node::FaultInterface
{
 public:
  ~Fcat();
  Fcat(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  rclcpp::CallbackGroup::SharedPtr get_process_loop_callback_group()
  {
    return process_loop_callback_group_;
  }
  rclcpp::CallbackGroup::SharedPtr get_topic_callback_group()
  {
    return topic_callback_group_;
  }

 private:
  void Process() override;
  void StartProcessTimer();
  void SetRealtimePreempt(int scheduler_priority);
  void PopulateDeviceStateFields();
  void SetCpuAffinity();
  void InitializeActuatorParams(const std::vector<std::string>& actuator_names,
                                int& i);

  void QueueCommand(fastcat::DeviceCmd& cmd)
  {
    command_queue_size_++;
    fcat_manager_.QueueCommand(cmd);
  }

  bool ActuatorExistsOnBus(const std::string& name);
  bool ActuatorExistsOnBus(const std::string& name, std::string& error_message);
  bool DeviceExistsOnBus(const std::string& name,
                         fastcat::DeviceStateType type);
  bool DeviceExistsOnBus(const std::string& name, fastcat::DeviceStateType type,
                         std::string& error_message);

  bool TypeExistsOnBus(fastcat::DeviceStateType type);
  void InitializePublishersAndMessages();
  void InitializeSubscribers();
  void InitializeServices();

  void UpdateStateMap();

  void PublishFcatModuleState();
  void PublishAsyncSdoResponse();

  void PublishFtsStates();

  void PublishActuatorStates();
  void PublishEgdStates();
  void PublishEl1008States();
  void PublishEl2124States();
  void PublishEl2809States();
  void PublishEl2798States();
  void PublishEl2828States();
  void PublishEl3104States();
  void PublishEl3162States();
  void PublishEl3202States();
  void PublishEl3208States();
  void PublishEl3314States();
  void PublishEl3318States();
  void PublishEl3602States();
  void PublishEl4102States();
  void PublishEl5042States();
  void PublishIld1900States();

  void PublishCommanderStates();
  void PublishConditionalStates();
  void PublishFaulterStates();
  void PublishFilterStates();
  void PublishFunctionStates();
  void PublishPidStates();
  void PublishSaturationStates();
  void PublishSchmittTriggerStates();
  void PublishSignalGeneratorStates();
  void PublishLinearInterpolationStates();
  void PublishThreeNodeThermalModelStates();

  void CallActuatorCSP(const fcat_msgs::msg::ActuatorCspCmd& csp_cmd, double t);
  void CallActuatorCSV(const fcat_msgs::msg::ActuatorCsvCmd& csv_cmd);
  void CallActuatorCST(const fcat_msgs::msg::ActuatorCstCmd& cst_cmd);
  void CallActuatorProfPos(const fcat_msgs::msg::ActuatorProfPosCmd& prof_pos_cmd);

  /////////////////////////////////////
  /////// Topic Callbacks //////////
  /////////////////////////////////////

  void ResetCmdCb(const std::shared_ptr<std_msgs::msg::Empty> msg) override;
  void FaultCmdCb(const std::shared_ptr<std_msgs::msg::Empty> msg) override;

  void AsyncSdoReadCmdCb(
    const std::shared_ptr<fcat_msgs::msg::AsyncSdoReadCmd> msg);
  void AsyncSdoWriteCmdCb(
    const std::shared_ptr<fcat_msgs::msg::AsyncSdoWriteCmd> msg);

  void ActuatorCSPCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCspCmd> msg);
  void ActuatorCSVCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCsvCmd> msg);
  void ActuatorCSTCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCstCmd> msg);
  void ActuatorCSPCmdsCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCspCmds> msg);
  void ActuatorCSVCmdsCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCsvCmds> msg);
  void ActuatorCSTCmdsCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCstCmds> msg);
  void ActuatorCalibrateCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorCalibrateCmd> msg);
  void ActuatorProfPosCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfPosCmd> msg);
  void ActuatorProfTorqueCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfTorqueCmd> msg);
  void ActuatorProfVelCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfVelCmd> msg);
  void ActuatorProfPosCmdsCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorProfPosCmds> msg);
  void ActuatorSetOutputPositionCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorSetOutputPositionCmd> msg);
  void ActuatorSetDigitalOutputCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorSetDigitalOutputCmd> msg);
  void ActuatorSetMaxCurrentCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorSetMaxCurrentCmd> msg);
  void ActuatorSetUnitModeCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorSetUnitModeCmd> msg);
  void ActuatorSetProfDisengagingTimeoutCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorSetProfDisengagingTimeoutCmd>
      msg);
  void ActuatorHaltCmdCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorHaltCmd> msg);
  void ActuatorHaltCmdsCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorHaltCmds> msg);

  void CommanderEnableCmdCb(
    const std::shared_ptr<fcat_msgs::msg::CommanderEnableCmd> msg);
  void CommanderDisableCmdCb(
    const std::shared_ptr<fcat_msgs::msg::CommanderDisableCmd> msg);

  void El2124WriteAllChannelsCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2124WriteAllChannelsCmd> msg);
  void El2124WriteChannelCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2124WriteChannelCmd> msg);

  void El2809WriteAllChannelsCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2809WriteAllChannelsCmd> msg);
  void El2809WriteChannelCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El2809WriteChannelCmd> msg);
  
  void El2828WriteAllChannelsCmdCb(
      const std::shared_ptr<fcat_msgs::msg::El2828WriteAllChannelsCmd> msg);
  void El2828WriteChannelCmdCb(
      const std::shared_ptr<fcat_msgs::msg::El2828WriteChannelCmd> msg);

  void El2798WriteAllChannelsCmdCb(
      const std::shared_ptr<fcat_msgs::msg::El2798WriteAllChannelsCmd> msg);
  void El2798WriteChannelCmdCb(
      const std::shared_ptr<fcat_msgs::msg::El2798WriteChannelCmd> msg);

  void El4102WriteAllChannelsCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El4102WriteAllChannelsCmd> msg);
  void El4102WriteChannelCmdCb(
    const std::shared_ptr<fcat_msgs::msg::El4102WriteChannelCmd> msg);

  void FaulterEnableCmdCb(
    const std::shared_ptr<fcat_msgs::msg::FaulterEnableCmd> msg);

  void FtsTareCmdCb(const std::shared_ptr<fcat_msgs::msg::FtsTareCmd> msg);

  void PidActivateCmdCb(
    const std::shared_ptr<fcat_msgs::msg::PidActivateCmd> msg);

  rcl_interfaces::msg::SetParametersResult SetParametersCb(
    const std::vector<rclcpp::Parameter>&);

  ////////////////
  // Publishers //
  ////////////////
  static const unsigned int publisher_queue_size_ = 32;
  static const unsigned int subscription_queue_size_ = 32;
  const rclcpp::QoS subscription_qos_ = rclcpp::QoS(subscription_queue_size_);
  const rclcpp::QoS service_qos_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ModuleState>::SharedPtr module_state_pub_;
  rclcpp::Publisher<fcat_msgs::msg::AsyncSdoResponse>::SharedPtr
    async_sdo_response_pub_;

  rclcpp::Publisher<fcat_msgs::msg::ActuatorStates>::SharedPtr actuator_pub_;
  rclcpp::Publisher<fcat_msgs::msg::EgdStates>::SharedPtr egd_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El1008States>::SharedPtr el1008_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El2124States>::SharedPtr el2124_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El2809States>::SharedPtr el2809_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El2798States>::SharedPtr el2798_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El2828States>::SharedPtr el2828_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3104States>::SharedPtr el3104_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3162States>::SharedPtr el3162_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3202States>::SharedPtr el3202_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3208States>::SharedPtr el3208_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3314States>::SharedPtr el3314_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3318States>::SharedPtr el3318_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El3602States>::SharedPtr el3602_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El4102States>::SharedPtr el4102_pub_;
  rclcpp::Publisher<fcat_msgs::msg::El5042States>::SharedPtr el5042_pub_;
  rclcpp::Publisher<fcat_msgs::msg::Ild1900States>::SharedPtr ild1900_pub_;

  rclcpp::Publisher<fcat_msgs::msg::SaturationStates>::SharedPtr
    saturation_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FilterStates>::SharedPtr filter_pub_;
  rclcpp::Publisher<fcat_msgs::msg::PidStates>::SharedPtr pid_pub_;
  rclcpp::Publisher<fcat_msgs::msg::CommanderStates>::SharedPtr commander_pub_;
  rclcpp::Publisher<fcat_msgs::msg::SignalGeneratorStates>::SharedPtr
    signal_generator_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FaulterStates>::SharedPtr faulter_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FunctionStates>::SharedPtr function_pub_;
  rclcpp::Publisher<fcat_msgs::msg::FtsStates>::SharedPtr fts_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ConditionalStates>::SharedPtr
    conditional_pub_;
  rclcpp::Publisher<fcat_msgs::msg::SchmittTriggerStates>::SharedPtr
    schmitt_trigger_pub_;
  rclcpp::Publisher<fcat_msgs::msg::LinearInterpolationStates>::SharedPtr
    linear_interpolation_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ThreeNodeThermalModelStates>::SharedPtr
    three_node_thermal_model_pub_;

  std::unordered_map<std::string, WrenchPublisher::SharedPtr> fts_raw_pub_map_;
  std::unordered_map<std::string, WrenchPublisher::SharedPtr>
    fts_tared_pub_map_;

  rclcpp::TimerBase::SharedPtr process_timer_ = nullptr;
  ///////////////////////
  // Service Callbacks //
  ///////////////////////

  void ResetSrvCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;

  void FaultSrvCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;

  void ActuatorHaltSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorHaltService::Request> request,
    std::shared_ptr<fcat_msgs::srv::ActuatorHaltService::Response> response);

  void ActuatorSetGainSchedulingIndexSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::ActuatorSetGainSchedulingIndexService::Request>
      request,
    std::shared_ptr<
      fcat_msgs::srv::ActuatorSetGainSchedulingIndexService::Response>
      response);

  void ActuatorSetMaxCurrentSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorSetMaxCurrentService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorSetMaxCurrentService::Response>
      response);

  void ActuatorSetOutputPositionSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::ActuatorSetOutputPositionService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorSetOutputPositionService::Response>
      response);

  void ActuatorSetDigitalOutputSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::ActuatorSetDigitalOutputService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorSetDigitalOutputService::Response>
      response);

  void ActuatorSetProfDisengagingTimeoutSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::ActuatorSetProfDisengagingTimeoutService::Request>
      request,
    std::shared_ptr<
      fcat_msgs::srv::ActuatorSetProfDisengagingTimeoutService::Response>
      response);

  void ActuatorCalibrateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Response>
      response);

  void ActuatorProfPosSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Response> response);

  void ActuatorProfVelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Response> response);

  void ActuatorProfTorqueSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Response>
      response);

  void CommanderEnableSrvCb(
    const std::shared_ptr<fcat_msgs::srv::CommanderEnableService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::CommanderEnableService::Response> response);

  void CommanderDisableSrvCb(
    const std::shared_ptr<fcat_msgs::srv::CommanderDisableService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::CommanderDisableService::Response>
      response);

  void El2124WriteAllChannelsSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::El2124WriteAllChannelsService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::El2124WriteAllChannelsService::Response>
      response);

  void El2124WriteChannelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::El2124WriteChannelService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::El2124WriteChannelService::Response>
      response);

  void El2809WriteAllChannelsSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::El2809WriteAllChannelsService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::El2809WriteAllChannelsService::Response>
      response);

  void El2809WriteChannelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::El2809WriteChannelService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::El2809WriteChannelService::Response>
      response);

  void El2798WriteAllChannelsSrvCb(
      const std::shared_ptr<
          fcat_msgs::srv::El2798WriteAllChannelsService::Request>
          request,
      std::shared_ptr<fcat_msgs::srv::El2798WriteAllChannelsService::Response>
          response);

  void El2798WriteChannelSrvCb(
      const std::shared_ptr<fcat_msgs::srv::El2798WriteChannelService::Request>
          request,
      std::shared_ptr<fcat_msgs::srv::El2798WriteChannelService::Response>
          response);

  void El2828WriteAllChannelsSrvCb(
      const std::shared_ptr<
          fcat_msgs::srv::El2828WriteAllChannelsService::Request>
          request,
      std::shared_ptr<fcat_msgs::srv::El2828WriteAllChannelsService::Response>
          response);

  void El2828WriteChannelSrvCb(
      const std::shared_ptr<fcat_msgs::srv::El2828WriteChannelService::Request>
          request,
      std::shared_ptr<fcat_msgs::srv::El2828WriteChannelService::Response>
          response);

  void El4102WriteAllChannelsSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::El4102WriteAllChannelsService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::El4102WriteAllChannelsService::Response>
      response);

  void El4102WriteChannelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::El4102WriteChannelService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::El4102WriteChannelService::Response>
      response);

  void FaulterEnableSrvCb(
    const std::shared_ptr<fcat_msgs::srv::FaulterEnableService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::FaulterEnableService::Response> response);

  void FtsTareSrvCb(
    const std::shared_ptr<fcat_msgs::srv::DeviceTriggerService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::DeviceTriggerService::Response> response);

  void PidActivateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::PidActivateService::Request> request,
    std::shared_ptr<fcat_msgs::srv::PidActivateService::Response> response);

  rclcpp::CallbackGroup::SharedPtr process_loop_callback_group_;
  rclcpp::CallbackGroup::SharedPtr topic_callback_group_;

  ////////////////////
  // ROS Parameters //
  ////////////////////

  bool use_sim_time_;
  bool enable_js_pub_;
  bool enable_ros_wrench_pub_;
  bool enable_realtime_preempt_ = false;
  int scheduler_priority_ = 49;

  bool fault_on_cycle_slip_ = true;
  double cycle_slip_fault_magnitude_ = 3.0;
  
  ////////////
  // fields //
  ////////////

  std::vector<std::shared_ptr<const fastcat::DeviceState>> device_state_ptrs_;
  std::unordered_map<std::string, std::shared_ptr<const fastcat::DeviceState>>
    device_name_state_map_;
  std::unordered_map<fastcat::DeviceStateType,
                     std::vector<std::shared_ptr<const fastcat::DeviceState>>>
    device_type_vec_map_;
  std::vector<std::any> services_;
  std::vector<std::any> subscriptions_;

  double loop_period_sec_ = 0.0;
  double last_time_ = 0.0;

  bool time_stamp_initialized_ = false;

  int process_loop_cpu_id_ = 0;
  int process_loop_thread_id_ = -1;
  bool cpu_affinity_initialized_ = false;
  bool subscription_cpu_affinity_initialized_ = false;
  bool process_loop_realtime_preempt_initialized_ = false;
  bool cs_subscription_cb_realtime_preempt_initialized_ = false;

  rclcpp::Time publish_time_stamp_;

  fastcat::Manager fcat_manager_;

  fcat_msgs::msg::ModuleState module_state_msg_;
  fcat_msgs::msg::ActuatorStates actuator_states_msg_;
  fcat_msgs::msg::EgdStates egd_states_msg_;
  fcat_msgs::msg::El1008States el1008_states_msg_;
  fcat_msgs::msg::El2124States el2124_states_msg_;
  fcat_msgs::msg::El2809States el2809_states_msg_;
  fcat_msgs::msg::El2798States el2798_states_msg_;
  fcat_msgs::msg::El2828States el2828_states_msg_;
  fcat_msgs::msg::El3104States el3104_states_msg_;
  fcat_msgs::msg::El3162States el3162_states_msg_;
  fcat_msgs::msg::El3202States el3202_states_msg_;
  fcat_msgs::msg::El3208States el3208_states_msg_;
  fcat_msgs::msg::El3314States el3314_states_msg_;
  fcat_msgs::msg::El3318States el3318_states_msg_;
  fcat_msgs::msg::El3602States el3602_states_msg_;
  fcat_msgs::msg::El4102States el4102_states_msg_;
  fcat_msgs::msg::El5042States el5042_states_msg_;
  fcat_msgs::msg::Ild1900States ild1900_states_msg_;

  fcat_msgs::msg::CommanderStates commander_states_msg_;
  fcat_msgs::msg::ConditionalStates conditional_states_msg_;
  fcat_msgs::msg::FaulterStates faulter_states_msg_;
  fcat_msgs::msg::FilterStates filter_states_msg_;
  fcat_msgs::msg::FunctionStates function_states_msg_;
  fcat_msgs::msg::FtsStates fts_states_msg_;
  fcat_msgs::msg::PidStates pid_states_msg_;
  fcat_msgs::msg::SaturationStates saturation_states_msg_;
  fcat_msgs::msg::SchmittTriggerStates schmitt_trigger_states_msg_;
  fcat_msgs::msg::SignalGeneratorStates signal_generator_states_msg_;
  fcat_msgs::msg::LinearInterpolationStates linear_interpolation_states_msg_;
  fcat_msgs::msg::ThreeNodeThermalModelStates
    three_node_thermal_model_states_msg_;

  size_t command_queue_size_ = 0;
};
#endif
