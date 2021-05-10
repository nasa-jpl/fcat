#include "fcat/fcat_srvs.hpp"
#include "jsd/jsd_print.h"
#include "fastcat/jsd/actuator.h"

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

//FcatSrvs::~FcatSrvs(){ }

FcatSrvs::FcatSrvs() : Node("fcat_srvs", "fcat") {

  cb_group_blocking_     = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_non_blocking_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  this->declare_parameter<double>("loop_rate_hz", 100);
  this->get_parameter("loop_rate_hz", loop_rate_hz_);

  this->declare_parameter<double>("position_tolerance", 1.0e-2);
  this->get_parameter("position_tolerance", position_tolerance_);

  this->declare_parameter<double>("velocity_tolerance", 1.0e-2);
  this->get_parameter("velocity_tolerance", velocity_tolerance_);

  this->declare_parameter<double>("current_tolerance", 1.0e-2);
  this->get_parameter("current_tolerance", current_tolerance_);


  InitSubscribers();
  InitPublishers();
  InitServices();

  module_state_last_recv_time_ = 0;
  act_states_last_recv_time_ = 0;
  pid_states_last_recv_time_ = 0;
  MSG("Spinning...");
}

void FcatSrvs::InitSubscribers()
{
  auto sub_opts = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
  sub_opts.callback_group = cb_group_non_blocking_;

  fcat_module_state_sub_ = this->create_subscription<fcat_msgs::msg::ModuleState>(
      "state/module_state",
      FCAT_SRVS_QOS_SUBS, 
      std::bind(&FcatSrvs::FcatModuleStateCb, this, _1),
      sub_opts);

  act_states_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorStates>(
      "state/actuators", 
      FCAT_SRVS_QOS_SUBS, 
      std::bind(&FcatSrvs::ActuatorStatesCb, this, _1),
      sub_opts);

  pid_states_sub_ = this->create_subscription<fcat_msgs::msg::PidStates>(
      "state/pids", 
      FCAT_SRVS_QOS_SUBS, 
      std::bind(&FcatSrvs::PidStatesCb, this, _1),
      sub_opts);
}

void FcatSrvs::InitPublishers()
{
  act_prof_pos_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorProfPosCmd>(
      "cmd/actuator_prof_pos", FCAT_SRVS_PUB_QUEUE_SIZE);

  act_prof_vel_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorProfVelCmd>(
      "cmd/actuator_prof_vel", FCAT_SRVS_PUB_QUEUE_SIZE);

  act_prof_torque_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorProfTorqueCmd>(
      "cmd/actuator_prof_torque", FCAT_SRVS_PUB_QUEUE_SIZE);

  act_calibrate_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorCalibrateCmd>(
      "cmd/actuator_calibrate", FCAT_SRVS_PUB_QUEUE_SIZE);

  pid_activate_pub_ = this->create_publisher<fcat_msgs::msg::PidActivateCmd>(
      "cmd/pid_activate", FCAT_SRVS_PUB_QUEUE_SIZE);
}

void FcatSrvs::InitServices()
{
  act_prof_pos_srv_=
    this->create_service<fcat_msgs::srv::ActuatorProfPosCmd>(
        "service/actuator_prof_pos", 
        std::bind(&FcatSrvs::ActuatorProfPosSrvCb, this, _1, _2),
        FCAT_SRVS_QOS_SRVS,
        cb_group_blocking_);

  act_prof_vel_srv_=
    this->create_service<fcat_msgs::srv::ActuatorProfVelCmd>(
        "service/actuator_prof_vel", 
        std::bind(&FcatSrvs::ActuatorProfVelSrvCb, this, _1, _2),
        FCAT_SRVS_QOS_SRVS,
        cb_group_blocking_);

  act_prof_torque_srv_=
    this->create_service<fcat_msgs::srv::ActuatorProfTorqueCmd>(
        "service/actuator_prof_torque", 
        std::bind(&FcatSrvs::ActuatorProfTorqueSrvCb, this, _1, _2),
        FCAT_SRVS_QOS_SRVS,
        cb_group_blocking_);

  act_calibrate_srv_=
    this->create_service<fcat_msgs::srv::ActuatorCalibrateCmd>(
        "service/actuator_calibrate", 
        std::bind(&FcatSrvs::ActuatorCalibrateSrvCb, this, _1, _2),
        FCAT_SRVS_QOS_SRVS,
        cb_group_blocking_);

  pid_activate_srv_=
    this->create_service<fcat_msgs::srv::PidActivateCmd>(
        "service/pid_activate", 
        std::bind(&FcatSrvs::PidActivateSrvCb, this, _1, _2),
        FCAT_SRVS_QOS_SRVS,
        cb_group_blocking_);
}

bool FcatSrvs::ActuatorCmdPrechecks(std::string name, std::string &message)
{
  //1) Check fcat module state liveliness check
  if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
    message = "stale fcat module state topic, fcat is not running";
    WARNING("bad command: %s", message.c_str());
    return false;
  }

  //2) Check fcat module is not faulted
  if(fcat_module_state_msg_.faulted){
    message = "fcat is faulted, reset fcat first";
    WARNING("bad command: %s", message.c_str());
    return false;
  }

  //3) Check Actuator States liveliness check
  if((get_time_sec() - act_states_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
    message = "stale fcat actuator states topic, check fastcat YAML has any actuators";
    WARNING("bad command: %s", message.c_str());
    return false;
  }

  //4) Check Actuator name is on the bus
  if(act_state_map_.find(name) == act_state_map_.end()){
    message = "actuator name not found, check device name";
    WARNING("bad command: %s", message.c_str());
    return false;
  }
  return true;
}

bool FcatSrvs::PidCmdPrechecks(std::string name, std::string &message){
  //1) Check fcat module state liveliness check
  if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
    message = "stale fcat module state topic, fcat is not running";
    WARNING("bad command: %s", message.c_str());
    return false;
  }

  //2) Check fcat module is not faulted
  if(fcat_module_state_msg_.faulted){
    message = "fcat is faulted, reset fcat first";
    WARNING("bad command: %s", message.c_str());
    return false;
  }

  //3) Check Pid States liveliness check
  if((get_time_sec() - pid_states_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
    message = "stale fcat pid states topic, check fastcat YAML has any pid devices";
    WARNING("bad command: %s", message.c_str());
    return false;
  }

  //4) Check Pid name is on the bus
  if(pid_state_map_.find(name) == pid_state_map_.end()){
    message = "pid name not found, check device name";
    WARNING("bad command: %s", message.c_str());
    return false;
  }
  return true;
}

void FcatSrvs::FcatModuleStateCb(
    const std::shared_ptr<fcat_msgs::msg::ModuleState> msg)
{
  module_state_last_recv_time_ = get_time_sec();
  fcat_module_state_msg_ = *msg;
}

void FcatSrvs::ActuatorStatesCb(
    const std::shared_ptr<fcat_msgs::msg::ActuatorStates> msg)
{
  act_states_last_recv_time_ = get_time_sec();
  actuator_states_msg_ = *msg;

  for(size_t i = 0; i < actuator_states_msg_.names.size(); ++i){
    act_state_map_[actuator_states_msg_.names[i]] = actuator_states_msg_.states[i];
  }
}

void FcatSrvs::PidStatesCb(
    const std::shared_ptr<fcat_msgs::msg::PidStates> msg)
{
  pid_states_last_recv_time_ = get_time_sec();
  pid_states_msg_ = *msg;

  for(size_t i = 0; i < pid_states_msg_.names.size(); ++i){
    pid_state_map_[pid_states_msg_.names[i]] = pid_states_msg_.states[i];
  }
}

void FcatSrvs::ActuatorProfPosSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosCmd::Request> request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfPosCmd::Response>      response)
{
  MSG("fcat_srv recived new actuator_prof_pos command");
  response->success = false;

  if(!ActuatorCmdPrechecks(request->name, response->message)){
    return;
  }

  MSG("actuator_prof_pos: issuing topic cmd and watching for completion");

  //Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfPosCmd();

  cmd_msg.name             = request->name;
  cmd_msg.target_position  = request->target_position;
  cmd_msg.profile_velocity = request->profile_velocity;
  cmd_msg.profile_accel    = request->profile_accel;
  cmd_msg.relative         = request->relative;

  act_prof_pos_pub_->publish(cmd_msg);


  // Continuously check for 
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - actuator state machine progress

  fastcat::ActuatorStateMachineState sms;

  double pos;
  rclcpp::Rate timer(loop_rate_hz_); //Hz

  while(true){
    if(!rclcpp::ok()){
      response->message = "fcat_srvs node shutdown";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
      response->message = "stale fcat module state topic, fcat has stopped ";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    sms = static_cast<fastcat::ActuatorStateMachineState>(
        act_state_map_[request->name].actuator_state_machine_state);
    pos = act_state_map_[request->name].actual_position;

    if(sms == fastcat::ACTUATOR_SMS_FAULTED){
      response->message = std::string("Actuator unexpectedly faulted");
      ERROR("Failure: %s", response->message.c_str());
      return;
    }


    if(sms != fastcat::ACTUATOR_SMS_PROF_POS &&
        fabs(pos - request->target_position) < position_tolerance_){
      break;
    }

    timer.sleep();
  }

  MSG("Completed ACTUATOR_PROF_POS for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}

void FcatSrvs::ActuatorProfVelSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelCmd::Request> request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfVelCmd::Response>      response)
{
  MSG("fcat_srv recived new actuator_prof_vel command");
  response->success = false;

  if(!ActuatorCmdPrechecks(request->name, response->message)){
    return;
  }

  MSG("actuator_prof_vel: issuing topic cmd and watching for completion");

  //Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfVelCmd();

  cmd_msg.name            = request->name;
  cmd_msg.target_velocity = request->target_velocity;
  cmd_msg.profile_accel   = request->profile_accel;
  cmd_msg.max_duration    = request->max_duration;

  act_prof_vel_pub_->publish(cmd_msg);

  // Continuously check for 
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - actuator state machine progress

  fastcat::ActuatorStateMachineState sms;
  double velocity;
  rclcpp::Rate timer(loop_rate_hz_); //Hz

  while(true){
    if(!rclcpp::ok()){
      response->message = "fcat_srvs node shutdown";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
      response->message = "stale fcat module state topic, fcat has stopped ";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    sms = static_cast<fastcat::ActuatorStateMachineState>(
        act_state_map_[request->name].actuator_state_machine_state);

    velocity = act_state_map_[request->name].cmd_velocity;

    if(sms == fastcat::ACTUATOR_SMS_FAULTED){
      response->message = std::string("Actuator unexpectedly faulted");
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if(sms == fastcat::ACTUATOR_SMS_PROF_VEL && 
        fabs(velocity - request->target_velocity) < velocity_tolerance_){
      MSG("Actuator is in PROF_VEL mode and achieved target velocity");
      break;
    }

    timer.sleep();
  }

  MSG("Completed ACTUATOR_PROF_VEL for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}

void FcatSrvs::ActuatorProfTorqueSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueCmd::Request> request,
    std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueCmd::Response>      response)
{
  MSG("fcat_srv recived new actuator_prof_torque command");
  response->success = false;

  if(!ActuatorCmdPrechecks(request->name, response->message)){
    return;
  }

  MSG("actuator_prof_torque: issuing topic cmd and watching for completion");

  //Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfTorqueCmd();

  cmd_msg.name               = request->name;
  cmd_msg.target_torque_amps = request->target_torque_amps;
  cmd_msg.max_duration       = request->max_duration;

  act_prof_torque_pub_->publish(cmd_msg);

  // Continuously check for 
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - actuator state machine progress

  fastcat::ActuatorStateMachineState sms;
  double current;
  rclcpp::Rate timer(loop_rate_hz_); //Hz

  while(true){
    if(!rclcpp::ok()){
      response->message = "fcat_srvs node shutdown";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
      response->message = "stale fcat module state topic, fcat has stopped ";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    sms = static_cast<fastcat::ActuatorStateMachineState>(
        act_state_map_[request->name].actuator_state_machine_state);

    current = act_state_map_[request->name].cmd_current;

    if(sms == fastcat::ACTUATOR_SMS_FAULTED){
      response->message = std::string("Actuator unexpectedly faulted");
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if(sms == fastcat::ACTUATOR_SMS_PROF_TORQUE && 
        fabs(current - request->target_torque_amps) < current_tolerance_){
      MSG("Actuator is in PROF_TORQUE mode and achieved target current");
      break;
    }

    timer.sleep();
  }

  MSG("Completed ACTUATOR_PROF_TORQUE for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}

void FcatSrvs::ActuatorCalibrateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateCmd::Request> request,
    std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateCmd::Response>      response)
{
  MSG("fcat_srv recived new actuator_calibrate command");
  response->success = false;

  if(!ActuatorCmdPrechecks(request->name, response->message)){
    return;
  }

  MSG("actuator_calibrate: issuing topic cmd and watching for completion");

  //Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorCalibrateCmd();

  cmd_msg.name        = request->name;
  cmd_msg.velocity    = request->velocity;
  cmd_msg.accel       = request->accel;
  cmd_msg.max_current = request->max_current;

  act_calibrate_pub_->publish(cmd_msg);

  // Continuously check for 
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - actuator state machine progress

  fastcat::ActuatorStateMachineState sms;
  fastcat::ActuatorStateMachineState last_sms;
  last_sms = static_cast<fastcat::ActuatorStateMachineState>(
      act_state_map_[request->name].actuator_state_machine_state);

  rclcpp::Rate timer(loop_rate_hz_); //Hz

  while(true){
    if(!rclcpp::ok()){
      response->message = "fcat_srvs node shutdown";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
      response->message = "stale fcat module state topic, fcat has stopped ";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    sms = static_cast<fastcat::ActuatorStateMachineState>(
        act_state_map_[request->name].actuator_state_machine_state);

    if(sms == fastcat::ACTUATOR_SMS_FAULTED){
      response->message = std::string("Actuator unexpectedly faulted");
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if(sms != fastcat::ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP && 
         last_sms == fastcat::ACTUATOR_SMS_CAL_MOVE_TO_SOFTSTOP){
      break;
    }

    last_sms = sms;
    timer.sleep();
  }

  MSG("Completed ACTUATOR_CALIBRATE for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}

void FcatSrvs::PidActivateSrvCb(
    const std::shared_ptr<fcat_msgs::srv::PidActivateCmd::Request> request,
    std::shared_ptr<fcat_msgs::srv::PidActivateCmd::Response>      response)
{
  MSG("fcat_srv recived new pid_activate command");
  response->success = false;

  if(!PidCmdPrechecks(request->name, response->message)){
    return;
  }

  MSG("pid_activate: issuing topic cmd and watching for completion");

  //Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::PidActivateCmd();

  cmd_msg.name                 = request->name;
  cmd_msg.setpoint             = request->setpoint;
  cmd_msg.deadband             = request->deadband;
  cmd_msg.persistence_duration = request->persistence_duration;
  cmd_msg.max_duration         = request->max_duration;

  pid_activate_pub_->publish(cmd_msg);

  // Continuously check for 
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - pid active flag
  bool pid_is_active;
  bool last_pid_is_active = pid_state_map_[request->name].active;

  rclcpp::Rate timer(loop_rate_hz_); //Hz

  while(true){
    if(!rclcpp::ok()){
      response->message = "fcat_srvs node shutdown";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    if((get_time_sec() - module_state_last_recv_time_) > FCAT_SRVS_LIVELINESS_DURATION){ 
      response->message = "stale fcat module state topic, fcat has stopped ";
      ERROR("Failure: %s", response->message.c_str());
      return;
    }

    pid_is_active = pid_state_map_[request->name].active;

    if(!pid_is_active && last_pid_is_active){
      break;
    }

    last_pid_is_active = pid_is_active;
    timer.sleep();
  }

  MSG("Completed ACTUATOR_PROF_TORQUE for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}
