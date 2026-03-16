#ifndef FCAT__FCAT_SERVICES_HPP_
#define FCAT__FCAT_SERVICES_HPP_

#include <sys/time.h>
#include <queue>
#include "casah_node/evr_interface.hpp"
#include "rclcpp/rclcpp.hpp"

// Service messages
#include "fcat_msgs/msg/actuator_set_digital_output_cmd.hpp"
#include "fcat_msgs/srv/actuator_calibrate_service.hpp"
#include "fcat_msgs/srv/actuator_prof_pos_service.hpp"
#include "fcat_msgs/srv/actuator_prof_torque_service.hpp"
#include "fcat_msgs/srv/actuator_prof_vel_service.hpp"
#include "fcat_msgs/srv/actuator_set_digital_output_service.hpp"
#include "fcat_msgs/srv/actuator_set_gain_scheduling_mode_service.hpp"
#include "fcat_msgs/srv/actuator_set_unit_mode_service.hpp"
#include "fcat_msgs/srv/async_sdo_read_service.hpp"
#include "fcat_msgs/srv/async_sdo_write_service.hpp"
#include "fcat_msgs/srv/pid_activate_service.hpp"
#include "fcat_msgs/srv/tlc_read_service.hpp"
#include "fcat_msgs/srv/tlc_write_service.hpp"

// Cmd Messages
#include "fcat_msgs/msg/actuator_calibrate_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_pos_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_torque_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_vel_cmd.hpp"
#include "fcat_msgs/msg/async_sdo_read_cmd.hpp"
#include "fcat_msgs/msg/async_sdo_write_cmd.hpp"
#include "fcat_msgs/msg/pid_activate_cmd.hpp"

// State Messages
#include "fcat_msgs/msg/actuator_states.hpp"
#include "fcat_msgs/msg/async_sdo_response.hpp"
#include "fcat_msgs/msg/module_state.hpp"
#include "fcat_msgs/msg/pid_states.hpp"

class FcatServices : public casah_node::EvrInterface
{
 private:
  typedef enum {
    FCAT_SRV_STATE_IDLE_CHECKING,
    FCAT_SRV_STATE_RUNNING,
  } FcatSrvState;

 public:
  FcatServices(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  static const unsigned int publisher_queue_size_ = 5;
  static const unsigned int subscription_queue_size_ = 5;
  static constexpr double liveliness_duration_sec_ = 1.0;
  static constexpr double loop_rate_hz_ = 100.0;

  const rclcpp::QoS subscription_qos_ = 
    rclcpp::QoS(subscription_queue_size_).best_effort();
  const rclcpp::QoS services_qos_;

  void Process() override{};

  void InitSubscribers();
  void InitPublishers();
  void InitServices();

  // Fcat State Subscription Callbacks
  void FcatModuleStateCb(
    const std::shared_ptr<fcat_msgs::msg::ModuleState> msg);
  void ActuatorStatesCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorStates> msg);
  void PidStatesCb(const std::shared_ptr<fcat_msgs::msg::PidStates> msg);
  void AsyncSdoResponseCb(
    const std::shared_ptr<fcat_msgs::msg::AsyncSdoResponse> msg);

  // Helper functions
  bool ActuatorCmdPrechecks(std::string name, std::string& message);
  bool PidCmdPrechecks(std::string name, std::string& message);
  bool CheckCommonFaultActive(std::string& error_message);
  bool RunActuatorMonitorLoop(std::string& error_message, std::string act_name);
  bool WaitForSdoResponse(std::string& error_message, uint16_t app_id);

  // Commander Services and Callbacks
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

  void ActuatorCalibrateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Response>
      response);

  void ActuatorSetGainSchedulingModeSrvCb(
    const std::shared_ptr<
      fcat_msgs::srv::ActuatorSetGainSchedulingModeService::Request>
      request,
    std::shared_ptr<
      fcat_msgs::srv::ActuatorSetGainSchedulingModeService::Response>
      response);

  void ActuatorSetUnitModeSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorSetUnitModeService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::ActuatorSetUnitModeService::Response>
      response);

  void PidActivateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::PidActivateService::Request> request,
    std::shared_ptr<fcat_msgs::srv::PidActivateService::Response> response);

  void AsyncSdoWriteSrvCb(
    const std::shared_ptr<fcat_msgs::srv::AsyncSdoWriteService::Request>
      request,
    std::shared_ptr<fcat_msgs::srv::AsyncSdoWriteService::Response> response);

  void AsyncSdoReadSrvCb(
    const std::shared_ptr<fcat_msgs::srv::AsyncSdoReadService::Request> request,
    std::shared_ptr<fcat_msgs::srv::AsyncSdoReadService::Response> response);

  void TlcWriteSrvCb(
    const std::shared_ptr<fcat_msgs::srv::TlcWriteService::Request> request,
    std::shared_ptr<fcat_msgs::srv::TlcWriteService::Response> response);

  void TlcReadSrvCb(
    const std::shared_ptr<fcat_msgs::srv::TlcReadService::Request> request,
    std::shared_ptr<fcat_msgs::srv::TlcReadService::Response> response);

 private:
  // Fcat Cmd Publishers
  rclcpp::Publisher<fcat_msgs::msg::ActuatorProfPosCmd>::SharedPtr
    act_prof_pos_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorProfVelCmd>::SharedPtr
    act_prof_vel_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorProfTorqueCmd>::SharedPtr
    act_prof_torque_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorCalibrateCmd>::SharedPtr
    act_calibrate_pub_;
  rclcpp::Publisher<fcat_msgs::msg::PidActivateCmd>::SharedPtr
    pid_activate_pub_;
  rclcpp::Publisher<fcat_msgs::msg::AsyncSdoWriteCmd>::SharedPtr
    async_sdo_write_pub_;
  rclcpp::Publisher<fcat_msgs::msg::AsyncSdoReadCmd>::SharedPtr
    async_sdo_read_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorSetDigitalOutputCmd>::SharedPtr
    act_digital_output_pub_;

  fcat_msgs::msg::ModuleState fcat_module_state_msg_;
  fcat_msgs::msg::ActuatorStates actuator_states_msg_;
  fcat_msgs::msg::PidStates pid_states_msg_;
  fcat_msgs::msg::AsyncSdoResponse async_sdo_response_msg_;

  std::string pub_sub_ns_;
  uint16_t sdo_app_id_ = 0;
  double module_state_last_recv_time_;
  double act_states_last_recv_time_;
  double pid_states_last_recv_time_;
  size_t max_sdo_queue_size_;
  FcatSrvState srv_state_;

  std::unordered_map<std::string, fcat_msgs::msg::ActuatorState> act_state_map_;
  std::unordered_map<std::string, fcat_msgs::msg::PidState> pid_state_map_;
  std::queue<fcat_msgs::msg::AsyncSdoResponse> sdo_response_queue_;

  std::unique_ptr<rclcpp::Rate> rate_;
  rclcpp::CallbackGroup::SharedPtr cb_group_blocking_;
  rclcpp::CallbackGroup::SharedPtr cb_group_non_blocking_;
  std::vector<std::any> subscriptions_;
};

#endif
