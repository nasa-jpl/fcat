#ifndef FCAT_SRVS_HPP_
#define FCAT_SRVS_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sys/time.h>

// Service messages
#include "fcat_msgs/srv/actuator_calibrate_cmd.hpp"
#include "fcat_msgs/srv/actuator_prof_pos_cmd.hpp"
#include "fcat_msgs/srv/actuator_prof_torque_cmd.hpp"
#include "fcat_msgs/srv/actuator_prof_vel_cmd.hpp"
#include "fcat_msgs/srv/pid_activate_cmd.hpp"

// Cmd Messages
#include "fcat_msgs/msg/actuator_calibrate_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_pos_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_torque_cmd.hpp"
#include "fcat_msgs/msg/actuator_prof_vel_cmd.hpp"
#include "fcat_msgs/msg/pid_activate_cmd.hpp"

// State Messages
#include "fcat_msgs/msg/module_state.hpp"
#include "fcat_msgs/msg/actuator_states.hpp"
#include "fcat_msgs/msg/pid_states.hpp"

const unsigned int FCAT_SRVS_PUB_QUEUE_SIZE = 5;
const unsigned int FCAT_SRVS_SUB_QUEUE_SIZE = 5;
const double FCAT_SRVS_LIVELINESS_DURATION  = 1.0; // sec
const double FCAT_SRVS_LOOP_RATE            = 100; // Hz

const rclcpp::QoS FCAT_SRVS_QOS_SUBS = rclcpp::QoS(FCAT_SRVS_SUB_QUEUE_SIZE);
const rmw_qos_profile_t FCAT_SRVS_QOS_SRVS = rmw_qos_profile_services_default;

inline double get_time_sec()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}

class FcatSrvs: public rclcpp::Node{
  public:
    //~FcatSrvs();
    FcatSrvs();

  private:

  void InitSubscribers();
  void InitPublishers();
  void InitServices();
  bool ActuatorCmdPrechecks(std::string name, std::string &message);
  bool PidCmdPrechecks(std::string name, std::string &message);

  // Fcat Cmd Publishers
  rclcpp::Publisher<fcat_msgs::msg::ActuatorProfPosCmd>::SharedPtr act_prof_pos_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorProfVelCmd>::SharedPtr act_prof_vel_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorProfTorqueCmd>::SharedPtr act_prof_torque_pub_;
  rclcpp::Publisher<fcat_msgs::msg::ActuatorCalibrateCmd>::SharedPtr act_calibrate_pub_;
  rclcpp::Publisher<fcat_msgs::msg::PidActivateCmd>::SharedPtr pid_activate_pub_;

  // Commander Services and Callbacks
  rclcpp::Service<fcat_msgs::srv::ActuatorProfPosCmd>::SharedPtr    act_prof_pos_srv_;
  rclcpp::Service<fcat_msgs::srv::ActuatorProfVelCmd>::SharedPtr    act_prof_vel_srv_;
  rclcpp::Service<fcat_msgs::srv::ActuatorProfTorqueCmd>::SharedPtr act_prof_torque_srv_;
  rclcpp::Service<fcat_msgs::srv::ActuatorCalibrateCmd>::SharedPtr  act_calibrate_srv_;
  rclcpp::Service<fcat_msgs::srv::PidActivateCmd>::SharedPtr        pid_activate_srv_;

  void ActuatorProfPosSrvCb(
      const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosCmd::Request> request,
      std::shared_ptr<fcat_msgs::srv::ActuatorProfPosCmd::Response>      response);

  void ActuatorProfVelSrvCb(
      const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelCmd::Request> request,
      std::shared_ptr<fcat_msgs::srv::ActuatorProfVelCmd::Response>      response);

  void ActuatorProfTorqueSrvCb(
      const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueCmd::Request> request,
      std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueCmd::Response>      response);

  void ActuatorCalibrateSrvCb(
      const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateCmd::Request> request,
      std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateCmd::Response>      response);

  void PidActivateSrvCb(
      const std::shared_ptr<fcat_msgs::srv::PidActivateCmd::Request> request,
      std::shared_ptr<fcat_msgs::srv::PidActivateCmd::Response>      response);


  // Fcat State Subscribers and Callbacks
  void FcatModuleStateCb(const std::shared_ptr<fcat_msgs::msg::ModuleState> msg);
  void ActuatorStatesCb(const std::shared_ptr<fcat_msgs::msg::ActuatorStates> msg);
  void PidStatesCb(const std::shared_ptr<fcat_msgs::msg::PidStates> msg);

  rclcpp::Subscription<fcat_msgs::msg::ModuleState>::SharedPtr    fcat_module_state_sub_;
  rclcpp::Subscription<fcat_msgs::msg::ActuatorStates>::SharedPtr act_states_sub_;
  rclcpp::Subscription<fcat_msgs::msg::PidStates>::SharedPtr      pid_states_sub_;

  fcat_msgs::msg::ModuleState    fcat_module_state_msg_;
  fcat_msgs::msg::ActuatorStates actuator_states_msg_;
  fcat_msgs::msg::PidStates      pid_states_msg_;

  double module_state_last_recv_time_;
  double act_states_last_recv_time_;
  double pid_states_last_recv_time_;

  std::unordered_map<std::string, fcat_msgs::msg::ActuatorState> act_state_map_;
  std::unordered_map<std::string, fcat_msgs::msg::PidState>      pid_state_map_;

  rclcpp::CallbackGroup::SharedPtr cb_group_blocking_;
  rclcpp::CallbackGroup::SharedPtr cb_group_non_blocking_;

  double loop_rate_hz_;
  double position_tolerance_;
  double velocity_tolerance_;
  double current_tolerance_;

};

#endif
