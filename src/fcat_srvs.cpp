#include "fcat/fcat_services.hpp"
#include "rclcpp/rclcpp.hpp"
#include "fastcat/jsd/actuator.h"
#include "fcat/fcat_utils.hpp"
#include "jsd/jsd_print.h"

#include "casah_node/utils.hpp"

#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

FcatSrvs::FcatSrvs(const rclcpp::NodeOptions& options)
    : casah_node::BaseInterface("fcat_services", "fcat", options),
      services_qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default), rmw_qos_profile_services_default),
      module_state_last_recv_time_(0),
      act_states_last_recv_time_(0),
      pid_states_last_recv_time_(0),
      srv_state_(FCAT_SRV_STATE_IDLE_CHECKING)
{
  cb_group_blocking_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cb_group_non_blocking_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Init Parameters
  pub_sub_ns_ =
    DeclareInitParameterString("pub_sub_namespace", "/fcat/",
                               "The namespace of the outgoing publisher and "
                               "incoming subscription topics");

  sdo_app_id_ = static_cast<uint16_t>(
    DeclareInitParameterInt("starting_sdo_app_id", 1000,
                            "Starting value of the SDO tracking app_id. The "
                            "value is incremented each request",
                            0, 65535));

  max_sdo_queue_size_ = static_cast<size_t>(DeclareInitParameterInt(
    "max_sdo_queue_size", 32,
    "Max size of the SDO Response queue. This queue is not checked unless an"
    " SDO blocking service is active. This prevents the queue from allocating"
    " too much memory when left running for a long time",
    1, 10000));

  // Rutime Parameters
  DeclareRuntimeParameterInt(
    "idle_persist_rti", 5,
    "Number of RTI loops to wait to see if device state changes from idle to "
    "moving."
    "If this number of RTIs passes, then the command returns success, "
    "assumed device "
    "did not need to change in order or achieve the command.",
    1, 1000);

  DeclareRuntimeParameterDouble("tolerance", 1.0e-8,
                                "Tolerance used to check if argument is near "
                                "zero e.g. fabs(arg) < tolerance",
                                0, 10);

  DeclareRuntimeParameterDouble(
    "sdo_wait_duration_sec", 2.0,
    "Maximum time to wait for an Async SDO response. If this timer expires"
    " the service will terminate with a failure",
    0, 60);

  InitSubscribers();
  InitPublishers();
  InitServices();

  // Call this only after setting all initialization array params
  PreventArrayParamSet();
  InitializeTimerRate();

  // No need to call CasahNode::StartTimer(). Relying on executor.spin(),
  //   callbackgroups, and rclcpp::Rate instead
  rate_ = std::make_unique<rclcpp::Rate>(this->GetTimerRate());

  SetActive();
}

void FcatSrvs::InitSubscribers()
{
  auto sub_opts =
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
  sub_opts.callback_group = cb_group_non_blocking_;

  subscriptions_.push_back(
    this->create_subscription<fcat_msgs::msg::ModuleState>(
      pub_sub_ns_ + "state/module_state", subscription_qos_,
      std::bind(&FcatSrvs::FcatModuleStateCb, this, _1), sub_opts));

  subscriptions_.push_back(
    this->create_subscription<fcat_msgs::msg::ActuatorStates>(
      pub_sub_ns_ + "state/actuators", subscription_qos_,
      std::bind(&FcatSrvs::ActuatorStatesCb, this, _1), sub_opts));

  subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::PidStates>(
    pub_sub_ns_ + "state/pids", subscription_qos_,
    std::bind(&FcatSrvs::PidStatesCb, this, _1), sub_opts));

  subscriptions_.push_back(
    this->create_subscription<fcat_msgs::msg::AsyncSdoResponse>(
      pub_sub_ns_ + "state/async_sdo_response", subscription_qos_,
      std::bind(&FcatSrvs::AsyncSdoResponseCb, this, _1), sub_opts));
}

void FcatSrvs::InitPublishers()
{
  act_prof_pos_pub_ =
    this->create_publisher<fcat_msgs::msg::ActuatorProfPosCmd>(
      pub_sub_ns_ + "impl/actuator_prof_pos", publisher_queue_size_);

  act_prof_vel_pub_ =
    this->create_publisher<fcat_msgs::msg::ActuatorProfVelCmd>(
      pub_sub_ns_ + "impl/actuator_prof_vel", publisher_queue_size_);

  act_prof_torque_pub_ =
    this->create_publisher<fcat_msgs::msg::ActuatorProfTorqueCmd>(
      pub_sub_ns_ + "impl/actuator_prof_torque", publisher_queue_size_);

  act_digital_output_pub_ =
    this->create_publisher<fcat_msgs::msg::ActuatorSetDigitalOutputCmd>(
      pub_sub_ns_ + "impl/actuator_set_digital_output", publisher_queue_size_);

  act_calibrate_pub_ =
    this->create_publisher<fcat_msgs::msg::ActuatorCalibrateCmd>(
      pub_sub_ns_ + "impl/actuator_calibrate", publisher_queue_size_);

  pid_activate_pub_ = this->create_publisher<fcat_msgs::msg::PidActivateCmd>(
    pub_sub_ns_ + "impl/pid_activate", publisher_queue_size_);

  async_sdo_write_pub_ =
    this->create_publisher<fcat_msgs::msg::AsyncSdoWriteCmd>(
      pub_sub_ns_ + "impl/async_sdo_write", publisher_queue_size_);

  async_sdo_read_pub_ = this->create_publisher<fcat_msgs::msg::AsyncSdoReadCmd>(
    pub_sub_ns_ + "impl/async_sdo_read", publisher_queue_size_);
}

void FcatSrvs::InitServices()
{
  DeclareServiceCommand<fcat_msgs::srv::ActuatorProfPosService>(
    this, pub_sub_ns_ + "srv/actuator_prof_pos",
    &FcatSrvs::ActuatorProfPosSrvCb, services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Command a Profiled Position motion profile to the Actuator",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "target_position",
         /*description*/ "Goal Position target of the motion profile",
         /*Units*/ "EU"),
       CommandArgumentDescriptor(
         /*arg name*/ "profile_velocity",
         /*description*/ "The desired cruise rate of the motion profile",
         /*Units*/ "EU / sec"),
       CommandArgumentDescriptor(
         /*arg name*/ "profile_accel",
         /*description*/ "The desired acceleration of the motion profile",
         /*Units*/ "EU/sec^2"),
       CommandArgumentDescriptor(
         /*arg name*/ "relative",
         /*description*/ "If true, the target_position will computed "
                         "relative "
                         "to actual position")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorProfVelService>(
    this, pub_sub_ns_ + "srv/actuator_prof_vel",
    &FcatSrvs::ActuatorProfVelSrvCb, services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Command a Profiled Velocity motion profile to the Actuator",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "target_velocity",
         /*description*/ "The desired cruise rate of the motion profile",
         /*Units*/ "EU/sec"),
       CommandArgumentDescriptor(
         /*arg name*/ "profile_accel",
         /*description*/ "The desired acceleration of the motion profile",
         /*Units*/ "EU/sec^2"),
       CommandArgumentDescriptor(
         /*arg name*/ "max_duration",
         /*description*/
         "The duration of the command, negative values result "
         "in indefinite profiles",
         /*Units*/ "sec")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorProfTorqueService>(
    this, pub_sub_ns_ + "srv/actuator_prof_torque",
    &FcatSrvs::ActuatorProfTorqueSrvCb, services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Command a Profiled Torque motion profile to the Actuator. Here, "
      "Torque"
      "is equivalent to Current (frome the DS-402 specification)",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "target_torque_amps",
         /*description*/ "The desired effort in current",
         /*Units*/ "Amps"),
       CommandArgumentDescriptor(
         /*arg name*/ "max_duration",
         /*description*/
         "The duration of the command, "
         "negative values result in indefinite profiles",
         /*Units*/ "sec")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorCalibrateService>(
    this, pub_sub_ns_ + "srv/actuator_calibrate",
    &FcatSrvs::ActuatorCalibrateSrvCb, services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Command hardstop calibration to the Actuator. The actuator will "
      "move in "
      "the signed velocity direction with the specified lowered current "
      "limit "
      "to detect the hardstop. From there, the position is set to the "
      "calibration "
      "limit defined in the configuration file. Finally, the original "
      "current is "
      "restored and the actuator is commanded to the command limit defined "
      "in the "
      "configuration file.",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "velocity",
         /*description*/
         "The desired approach velocity for all motions during "
         "hardstop calibration",
         /*Units*/ "EU/sec"),
       CommandArgumentDescriptor(
         /*arg name*/ "accel",
         /*description*/ "The desired acceleration of the motion profile",
         /*Units*/ "EU/sec^2"),
       CommandArgumentDescriptor(
         /*arg name*/ "max_current",
         /*description*/
         "Overrides the Peak current setting but only during "
         "hardstop contact motion",
         /*Units*/ "Amps")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorSetGainSchedulingModeService>(
    this, pub_sub_ns_ + "srv/actuator_set_gain_scheduling_mode",
    &FcatSrvs::ActuatorSetGainSchedulingModeSrvCb, services_qos_,
    cb_group_blocking_,
    CommandDescriptor(
      "Set the Gain Schedule Mode for the actuator GS[2]. "
      "0 - No Gain Scheduling. "
      "[1,63] - Specific controller from table. "
      "64 - Enable Speed-based scheduling. "
      "65 - Enable position-based scheduling. "
      "66 - Best Settling. "
      "67 - manually scheduled, high-bits. "
      "68 - manually scheduled, low-bits."
      "Use this interface only if you know what you are doing, Caveat "
      "Emptor!",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "gain_scheduling_mode",
         /*description*/ "The desired gain schedule mode value")}));

  DeclareServiceCommand<fcat_msgs::srv::ActuatorSetUnitModeService>(
    this, pub_sub_ns_ + "srv/actuator_set_unit_mode",
    &FcatSrvs::ActuatorSetUnitModeSrvCb, services_qos_, cb_group_blocking_,
    CommandDescriptor("Sets the Unit Mode UM[1]. "
                      "1 - Torque Control. "
                      "2 - Speed Control. "
                      "3 - Stepper Control. "
                      "5 - Position Control (default). "
                      "6 - Stepper Open or Closed Loop.",
                      {CommandArgumentDescriptor(
                         /*arg name*/ "name",
                         /*description*/ "The Fastcat Device Name"),
                       CommandArgumentDescriptor(
                         /*arg name*/ "gain_scheduling_mode",
                         /*description*/ "The desired Unit Mode UM[1]. ")}));

  DeclareServiceCommand<fcat_msgs::srv::PidActivateService>(
    this, pub_sub_ns_ + "srv/pid_activate", &FcatSrvs::PidActivateSrvCb,
    services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Start the PID Controller with specified parameters.  "
      "Returns failure if it does not converge in the specified duration. "
      "Returns success and self-disables if the controller converges.",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "setpoint",
         /*description*/ "The controller setpoint",
         /*Units*/ "EU"),
       CommandArgumentDescriptor(
         /*arg name*/ "deadband",
         /*description*/
         "The controller deadband tolerance around the setpoint",
         /*Units*/ "EU"),
       CommandArgumentDescriptor(
         /*arg name*/ "persistence_duration",
         /*description*/
         "The amount of time the signal must remain with the "
         "deadband to end control",
         /*Units*/ "sec"),
       CommandArgumentDescriptor(
         /*arg name*/ "max_duration",
         /*description*/
         "The maximum duration of the command. Beware: "
         "negative values can result in indefinite runtime",
         /*Units*/ "sec")}));

  DeclareServiceCommand<fcat_msgs::srv::AsyncSdoWriteService>(
    this, pub_sub_ns_ + "srv/async_sdo_write",
    &FcatSrvs::AsyncSdoWriteSrvCb, services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Issues an Asynchronous SDO parameter Write and waits for the "
      "result of the operation to indicate success or failure.",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "sdo_index",
         /*description*/ "The SDO register index in hexidecimal (0x3034) "
                         "or "
                         "decimal (12340) form"),
       CommandArgumentDescriptor(
         /*arg name*/ "sdo_subindex",
         /*description*/ "The SDO register subindex (e.g. the 1 in "
                         "0x3034:1)"),
       CommandArgumentDescriptor(
         /*arg name*/ "data",
         /*description*/ "Data payload to write to the SDO"),
       CommandArgumentDescriptor(
         /*arg name*/ "data_type",
         /*description*/ "The type of the data payload, must be one of: "
                         "{I8, I16, I32, I64, F32, U8, U16, U32, "
                         "U64}")}));

  DeclareServiceCommand<fcat_msgs::srv::AsyncSdoReadService>(
    this, pub_sub_ns_ + "srv/async_sdo_read", &FcatSrvs::AsyncSdoReadSrvCb,
    services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Issues an Asynchronous SDO parameter Read and waits for the "
      "result of the operation to indicate success or failure.",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "sdo_index",
         /*description*/ "The SDO register index in hexidecimal (0x3034) "
                         "or "
                         "decimal (12340) form"),
       CommandArgumentDescriptor(
         /*arg name*/ "sdo_subindex",
         /*description*/ "The SDO register subindex (the 1 in 0x3034:1)"),
       CommandArgumentDescriptor(
         /*arg name*/ "data_type",
         /*description*/ "The type of the data payload, must be one of: "
                         "{I8, I16, I32, I64, F32, U8, U16, U32, "
                         "U64}")}));

  DeclareServiceCommand<fcat_msgs::srv::TlcWriteService>(
    this, pub_sub_ns_ + "srv/tlc_write", &FcatSrvs::TlcWriteSrvCb,
    services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Issues a ELMO Two-Letter Command (TLC) which in turn issues an "
      "Asynchronous SDO "
      "parameter Write and waits for the result to indicate success or "
      "failure. Consult "
      "The MAN-G-CR document for full listing of drive data objects.",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "tlc",
         /*description*/ "The Two-Letter Command (TLC) found in the ELMO "
                         "MAN-G-CR Command Reference Document. These two "
                         "chars "
                         "are converted to an SDO index. (e.g. PL, CL)"),
       CommandArgumentDescriptor(
         /*arg name*/ "subindex",
         /*description*/ "The TLC/SDO register subindex (the 1 in "
                         "PL[1])"),
       CommandArgumentDescriptor(
         /*arg name*/ "data",
         /*description*/ "Data payload to write to the drive"),
       CommandArgumentDescriptor(
         /*arg name*/ "data_type",
         /*description*/ "The type of the data payload, must be one of: "
                         "{I8, I16, I32, I64, F32, U8, U16, U32, "
                         "U64}")}));

  DeclareServiceCommand<fcat_msgs::srv::TlcReadService>(
    this, pub_sub_ns_ + "srv/tlc_read", &FcatSrvs::TlcReadSrvCb,
    services_qos_, cb_group_blocking_,
    CommandDescriptor(
      "Issues a ELMO Two-Letter Command (TLC) which in turn issues an "
      "Asynchronous SDO "
      "parameter Read and waits for the result to indicate success or "
      "failure. Consult "
      "The MAN-G-CR document for full listing of drive data objects.",
      {CommandArgumentDescriptor(
         /*arg name*/ "name",
         /*description*/ "The Fastcat Device Name"),
       CommandArgumentDescriptor(
         /*arg name*/ "tlc",
         /*description*/ "The Two-Letter Command (TLC) found in the ELMO "
                         "MAN-G-CR Command Reference Document. These two "
                         "chars "
                         "are converted to an SDO index. (e.g. PL, CL)"),
       CommandArgumentDescriptor(
         /*arg name*/ "subindex",
         /*description*/ "The TLC/SDO register subindex (the 1 in "
                         "PL[1])"),
       CommandArgumentDescriptor(
         /*arg name*/ "data_type",
         /*description*/ "The type of the data payload, must be one of: "
                         "{I8, I16, I32, I64, F32, U8, U16, U32, "
                         "U64}")}));
}

//
// Fcat State Subscription Callbacks
//

void FcatSrvs::FcatModuleStateCb(
  const std::shared_ptr<fcat_msgs::msg::ModuleState> msg)
{
  module_state_last_recv_time_ = this->now().seconds();
  fcat_module_state_msg_ = *msg;
}

void FcatSrvs::ActuatorStatesCb(
  const std::shared_ptr<fcat_msgs::msg::ActuatorStates> msg)
{
  act_states_last_recv_time_ = this->now().seconds();
  actuator_states_msg_ = *msg;

  for (size_t i = 0; i < actuator_states_msg_.names.size(); ++i) {
    act_state_map_[actuator_states_msg_.names[i]] =
      actuator_states_msg_.states[i];
  }
}

void FcatSrvs::PidStatesCb(
  const std::shared_ptr<fcat_msgs::msg::PidStates> msg)
{
  pid_states_last_recv_time_ = this->now().seconds();
  pid_states_msg_ = *msg;

  for (size_t i = 0; i < pid_states_msg_.names.size(); ++i) {
    pid_state_map_[pid_states_msg_.names[i]] = pid_states_msg_.states[i];
  }
}

void FcatSrvs::AsyncSdoResponseCb(
  const std::shared_ptr<fcat_msgs::msg::AsyncSdoResponse> msg)
{
  // prevent the queue from filling up too much
  if (sdo_response_queue_.size() >= max_sdo_queue_size_) {
    sdo_response_queue_.pop();
  }
  sdo_response_queue_.push((*msg));
}

//
// Helper Functions
//
bool FcatSrvs::ActuatorCmdPrechecks(std::string name, std::string& message)
{
  // 1) Check fcat module state liveliness check
  if ((this->now().seconds() - module_state_last_recv_time_) >
      liveliness_duration_sec_) {
    message = "stale fcat module state topic, fcat is not running";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }

  // 2) Check fcat module is not faulted
  if (fcat_module_state_msg_.faulted) {
    message = "fcat is faulted, reset fcat first";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }

  // 3) Check Actuator States liveliness check
  if ((this->now().seconds() - act_states_last_recv_time_) >
      liveliness_duration_sec_) {
    message =
      "stale fcat actuator states topic, check fastcat YAML has any "
      "actuators";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }

  // 4) Check Actuator name is on the bus
  if (act_state_map_.find(name) == act_state_map_.end()) {
    message = "actuator name not found, check device name";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }
  return true;
}

bool FcatSrvs::PidCmdPrechecks(std::string name, std::string& message)
{
  // 1) Check fcat module state liveliness check
  if ((this->now().seconds() - module_state_last_recv_time_) >
      liveliness_duration_sec_) {
    message = "stale fcat module state topic, fcat is not running";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }

  // 2) Check fcat module is not faulted
  if (fcat_module_state_msg_.faulted) {
    message = "fcat is faulted, reset fcat first";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }

  // 3) Check Pid States liveliness check
  if ((this->now().seconds() - pid_states_last_recv_time_) >
      liveliness_duration_sec_) {
    message =
      "stale fcat pid states topic, check fastcat YAML has any pid devices";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }

  // 4) Check Pid name is on the bus
  if (pid_state_map_.find(name) == pid_state_map_.end()) {
    message = "pid name not found, check device name";
    RCLCPP_WARN(this->get_logger(), "bad command: %s", message.c_str());
    return false;
  }
  return true;
}

bool FcatSrvs::CheckCommonFaultActive(std::string& error_message)
{
  if (!rclcpp::ok()) {
    error_message = "fcat_srvs node shutdown";
    RCLCPP_FATAL(this->get_logger(), "Failure: %s", error_message.c_str());
    return true;
  }

  if ((this->now().seconds() - module_state_last_recv_time_) >
      liveliness_duration_sec_) {
    error_message = "stale fcat module state topic, fcat has stopped ";
    RCLCPP_WARN(this->get_logger(), "Failure: %s", error_message.c_str());
    return true;
  }

  if (fcat_module_state_msg_.faulted) {
    error_message = "fcat is encountered a fault";
    RCLCPP_WARN(this->get_logger(), "%s", error_message.c_str());
    return false;
  }

  return false;
}

bool FcatSrvs::RunActuatorMonitorLoop(std::string& error_message,
                                          std::string act_name)
{
  fastcat::ActuatorStateMachineState sms;
  srv_state_ = FCAT_SRV_STATE_IDLE_CHECKING;

  int64_t idle_persist_rti = this->get_parameter("idle_persist_rti").as_int();
  int64_t idle_rti_count = 0;

  // Continuously check for
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - actuator state machine progress

  while (true) {
    rate_->sleep();

    if (CheckCommonFaultActive(error_message)) {
      return false;
    }

    sms = static_cast<fastcat::ActuatorStateMachineState>(
      act_state_map_[act_name].actuator_state_machine_state);

    if (sms == fastcat::ACTUATOR_SMS_FAULTED) {
      error_message = std::string("Actuator unexpectedly faulted");
      RCLCPP_WARN(this->get_logger(), "Failure: %s", error_message.c_str());
      return false;
    }

    switch (srv_state_) {
      case FCAT_SRV_STATE_IDLE_CHECKING:

        // In the event that the command is trivial, the actuator state machine
        // may not change in response to a queued command sent to the fastcat
        // manager. This check responds "success" if no state machine change is
        // detected
        if (sms == fastcat::ACTUATOR_SMS_HALTED ||
            sms == fastcat::ACTUATOR_SMS_HOLDING) {
          if (idle_rti_count >= idle_persist_rti) {
            RCLCPP_INFO(this->get_logger(),
              "Max number of RTIs (%ld) elapsed with no Actuator SMS change, "
              "interpreting this result as a success",
              idle_persist_rti);

            return true;
          }
          idle_rti_count++;
        } else {
          // must be in motion by now (faulted would be caught above)
          srv_state_ = FCAT_SRV_STATE_RUNNING;
        }
        break;

      case FCAT_SRV_STATE_RUNNING:
        if (sms == fastcat::ACTUATOR_SMS_HALTED ||
            sms == fastcat::ACTUATOR_SMS_HOLDING) {
          return true;
        }
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "bad srv state, aborting command");
        return false;
    }
  }

  return true;
}

bool FcatSrvs::WaitForSdoResponse(std::string& message, uint16_t app_id)
{
  char print_str[512];
  double sdo_wait_duration_sec =
    this->get_parameter("sdo_wait_duration_sec").as_double();
  double start_time = this->now().seconds();
  while (this->now().seconds() < (sdo_wait_duration_sec + start_time)) {
    // Check for FCAT fault
    if (CheckCommonFaultActive(message)) {
      return false;
    }

    if (!sdo_response_queue_.empty()) {
      async_sdo_response_msg_ = sdo_response_queue_.front();
      sdo_response_queue_.pop();

      // Check the app id, discard and keep waiting if it does not match
      if (app_id != async_sdo_response_msg_.app_id) {
        CFW_DEBUG(
          "SDO Response received but actual app_id:(%u) does not match "
          "expected app_id:(%u). Continuing to wait...",
          async_sdo_response_msg_.app_id, app_id);
        continue;
      }

      bool success = async_sdo_response_msg_.success;

      if (!success) {
        sprintf(print_str,
                "SDO Response Failed {Name:(%s) Index:(0x%x)"
                " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
                async_sdo_response_msg_.device_name.c_str(),
                async_sdo_response_msg_.sdo_index,
                async_sdo_response_msg_.sdo_subindex,
                async_sdo_response_msg_.data.c_str(),
                async_sdo_response_msg_.data_type.c_str(),
                async_sdo_response_msg_.app_id);
        message = print_str;
        RCLCPP_WARN(this->get_logger(), "%s", print_str);
      } else {
        sprintf(print_str,
                "SDO Response Success {Name:(%s) Index:(0x%x)"
                " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
                async_sdo_response_msg_.device_name.c_str(),
                async_sdo_response_msg_.sdo_index,
                async_sdo_response_msg_.sdo_subindex,
                async_sdo_response_msg_.data.c_str(),
                async_sdo_response_msg_.data_type.c_str(),
                async_sdo_response_msg_.app_id);
        message = print_str;
        RCLCPP_INFO(this->get_logger(), "%s", print_str);
      }
      return success;
    }

    rate_->sleep();
  }

  // if here, runout timer has expired
  sprintf(print_str,
          "SDO Runout timer expired for app_id:(%u). "
          "Parameter 'sdo_wait_duration_sec' is set to (%lf)",
          app_id, sdo_wait_duration_sec);
  message = print_str;
  RCLCPP_WARN(this->get_logger(), "%s", print_str);
  return false;
}

//
// Service Callbacks
//

void FcatSrvs::ActuatorProfPosSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorProfPosService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Prof Pos Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfPosCmd();

  cmd_msg.name = request->name;
  cmd_msg.target_position = request->target_position;
  cmd_msg.profile_velocity = request->profile_velocity;
  cmd_msg.profile_accel = request->profile_accel;
  cmd_msg.relative = request->relative;

  act_prof_pos_pub_->publish(cmd_msg);

  double goal_pos = request->target_position;
  if (request->relative) {
    goal_pos += act_state_map_[request->name].actual_position;
  }

  RCLCPP_INFO(this->get_logger(), "actuator:%s Profile Position command from: %lf to goal: %lf",
                  request->name.c_str(),
                  act_state_map_[request->name].actual_position, goal_pos);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Completed ACTUATOR_PROF_POS for: %s",
                    request->name.c_str());
    response->message = "";  // not strictly needed
  }
}

void FcatSrvs::ActuatorProfVelSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorProfVelService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Prof Vel Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  double tol = this->get_parameter("tolerance").as_double();
  if (fabs(request->max_duration) < tol) {
    RCLCPP_WARN(this->get_logger(),
      "Max Duration (%E) < tolerance (%E) indicating this runs forever. "
      "In this case, use the topic (non-blocking) interface instead of the "
      "service (blocking) interface",
      request->max_duration, tol);
    return;
  }

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfVelCmd();

  cmd_msg.name = request->name;
  cmd_msg.target_velocity = request->target_velocity;
  cmd_msg.profile_accel = request->profile_accel;
  cmd_msg.max_duration = request->max_duration;

  act_prof_vel_pub_->publish(cmd_msg);

  RCLCPP_INFO(this->get_logger(), "actuator:%s to goal velocity value: %lf",
                  request->name.c_str(), request->target_velocity);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Completed ACTUATOR_PROF_VEL for: %s",
                    request->name.c_str());
    response->message = "";
  }
}

void FcatSrvs::ActuatorProfTorqueSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorProfTorqueService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Prof Torque Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
    "actuator_prof_torque: issuing topic cmd and watching for completion");

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorProfTorqueCmd();

  cmd_msg.name = request->name;
  cmd_msg.target_torque_amps = request->target_torque_amps;
  cmd_msg.max_duration = request->max_duration;

  act_prof_torque_pub_->publish(cmd_msg);

  RCLCPP_INFO(this->get_logger(), "fcat_srv actuator:%s to goal current value: %lf",
                  request->name.c_str(), request->target_torque_amps);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Completed ACTUATOR_PROF_TORQUE for: %s",
                    request->name.c_str());
    response->message = "";
  }
}

void FcatSrvs::ActuatorCalibrateSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorCalibrateService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Calibrate Command");
  response->success = false;

  if (!ActuatorCmdPrechecks(request->name, response->message)) {
    return;
  }

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::ActuatorCalibrateCmd();

  cmd_msg.name = request->name;
  cmd_msg.velocity = request->velocity;
  cmd_msg.accel = request->accel;
  cmd_msg.max_current = request->max_current;

  act_calibrate_pub_->publish(cmd_msg);

  response->success = RunActuatorMonitorLoop(response->message, request->name);
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Completed ACTUATOR_CALIBRATE for: %s",
                    request->name.c_str());
    response->message = "";
  }
}

void FcatSrvs::ActuatorSetGainSchedulingModeSrvCb(
  const std::shared_ptr<
    fcat_msgs::srv::ActuatorSetGainSchedulingModeService::Request>
    request,
  std::shared_ptr<
    fcat_msgs::srv::ActuatorSetGainSchedulingModeService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Gain Scheduling Mode Command");
  response->success = false;

  auto tlc_req = std::make_shared<fcat_msgs::srv::TlcWriteService::Request>();
  auto tlc_res = std::make_shared<fcat_msgs::srv::TlcWriteService::Response>();

  tlc_req->name = request->name;
  tlc_req->tlc = "GS";
  tlc_req->subindex = 2;
  tlc_req->data = std::to_string(request->gain_scheduling_mode);
  tlc_req->data_type = "I32";

  TlcWriteSrvCb(tlc_req, tlc_res);

  response->message = tlc_res->message;
  response->success = tlc_res->success;
}

void FcatSrvs::ActuatorSetUnitModeSrvCb(
  const std::shared_ptr<fcat_msgs::srv::ActuatorSetUnitModeService::Request>
    request,
  std::shared_ptr<fcat_msgs::srv::ActuatorSetUnitModeService::Response>
    response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Actuator Set Unit Mode Command");
  response->success = false;

  auto tlc_req = std::make_shared<fcat_msgs::srv::TlcWriteService::Request>();
  auto tlc_res = std::make_shared<fcat_msgs::srv::TlcWriteService::Response>();

  tlc_req->name = request->name;
  tlc_req->tlc = "UM";
  tlc_req->subindex = 1;
  tlc_req->data = std::to_string(request->mode);
  tlc_req->data_type = "I32";

  TlcWriteSrvCb(tlc_req, tlc_res);

  response->message = tlc_res->message;
  response->success = tlc_res->success;
}

void FcatSrvs::PidActivateSrvCb(
  const std::shared_ptr<fcat_msgs::srv::PidActivateService::Request> request,
  std::shared_ptr<fcat_msgs::srv::PidActivateService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Pid Activate Command");
  response->success = false;

  if (!PidCmdPrechecks(request->name, response->message)) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "pid_activate: issuing topic cmd and watching for completion");

  // Publish topic to start the service
  auto cmd_msg = fcat_msgs::msg::PidActivateCmd();

  cmd_msg.name = request->name;
  cmd_msg.setpoint = request->setpoint;
  cmd_msg.deadband = request->deadband;
  cmd_msg.persistence_duration = request->persistence_duration;
  cmd_msg.max_duration = request->max_duration;

  pid_activate_pub_->publish(cmd_msg);

  // Continuously check for
  //  - fcat_srvs shutdown request
  //  - fcat faults
  //  - pid active flag
  bool pid_is_active;
  bool last_pid_is_active = pid_state_map_[request->name].active;

  while (true) {
    if (CheckCommonFaultActive(response->message)) {
      return;
    }

    pid_is_active = pid_state_map_[request->name].active;

    // This logic may be a little brittle if the
    // command uses low or zero persistence. If an issue is ever
    // reported, solve it here just like for the actuator case.
    if (!pid_is_active && last_pid_is_active) {
      break;
    }

    last_pid_is_active = pid_is_active;
    rate_->sleep();
  }

  RCLCPP_INFO(this->get_logger(), "Completed PID_ACTIVATE for: %s", request->name.c_str());
  response->message = "";
  response->success = true;
}

void FcatSrvs::AsyncSdoWriteSrvCb(
  const std::shared_ptr<fcat_msgs::srv::AsyncSdoWriteService::Request> request,
  std::shared_ptr<fcat_msgs::srv::AsyncSdoWriteService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling SDO Write command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!HexOrDecStrToNum(request->sdo_index, parsed_sdo_index)) {
    sprintf(print_str, "Invalid SDO index string:(%s) conversion to U16",
            request->sdo_index.c_str());

    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);

    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Write data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());
    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);
    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoWriteCmd msg;

  msg.name = request->name;
  msg.sdo_index = parsed_sdo_index;
  msg.sdo_subindex = request->sdo_subindex;
  msg.data = request->data;
  msg.data_type = request->data_type;
  msg.app_id = sdo_app_id_++;

  async_sdo_write_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "Async SDO Write issued {Name:(%s) Index:(0x%x)"
    " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
    msg.name.c_str(), msg.sdo_index, msg.sdo_subindex, msg.data.c_str(),
    msg.data_type.c_str(), msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}

void FcatSrvs::AsyncSdoReadSrvCb(
  const std::shared_ptr<fcat_msgs::srv::AsyncSdoReadService::Request> request,
  std::shared_ptr<fcat_msgs::srv::AsyncSdoReadService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling SDO Read command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!HexOrDecStrToNum(request->sdo_index, parsed_sdo_index)) {
    sprintf(print_str, "Invalid SDO index string:(%s) conversion to U16",
            request->sdo_index.c_str());

    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);
    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Read data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());

    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);
    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoReadCmd msg;

  msg.name = request->name;
  msg.sdo_index = parsed_sdo_index;
  msg.sdo_subindex = request->sdo_subindex;
  msg.data_type = request->data_type;
  msg.app_id = sdo_app_id_++;

  async_sdo_read_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "Async SDO Read issued {Name:(%s) Index:(0x%x)"
    " Subindex:(%u) DataType:(%s) AppId:(%u)}",
    msg.name.c_str(), msg.sdo_index, msg.sdo_subindex, msg.data_type.c_str(),
    msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}

void FcatSrvs::TlcWriteSrvCb(
  const std::shared_ptr<fcat_msgs::srv::TlcWriteService::Request> request,
  std::shared_ptr<fcat_msgs::srv::TlcWriteService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Two-Letter Command (TLC) Write command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!TlcStrToNum(request->tlc, parsed_sdo_index)) {
    sprintf(print_str, "Invalid TLC string:(%s) conversion to SDO",
            request->tlc.c_str());

    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);
    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Write data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());
    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);

    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoWriteCmd msg;

  msg.name = request->name;
  msg.sdo_index = parsed_sdo_index;
  msg.sdo_subindex = request->subindex;
  msg.data = request->data;
  msg.data_type = request->data_type;
  msg.app_id = sdo_app_id_++;

  async_sdo_write_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "Async SDO Write issued {Name:(%s) TLC:(%s) Index:(0x%x)"
    " Subindex:(%u) Data:(%s) DataType:(%s) AppId:(%u)}",
    msg.name.c_str(), request->tlc.c_str(), msg.sdo_index, msg.sdo_subindex,
    msg.data.c_str(), msg.data_type.c_str(), msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}

void FcatSrvs::TlcReadSrvCb(
  const std::shared_ptr<fcat_msgs::srv::TlcReadService::Request> request,
  std::shared_ptr<fcat_msgs::srv::TlcReadService::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Handling Two-Letter Command (TLC) Read command");
  char print_str[512];
  response->message = "";

  // 1) Parse input args
  uint16_t parsed_sdo_index = 0;
  if (!TlcStrToNum(request->tlc, parsed_sdo_index)) {
    sprintf(print_str, "Invalid TLC string:(%s) conversion to SDO",
            request->tlc.c_str());

    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);
    return;
  }

  if (JSD_SDO_DATA_UNSPECIFIED ==
      jsd_sdo_data_type_from_string(request->data_type)) {
    sprintf(print_str,
            "Invalid SDO Read data_type:(%s) Must be one of "
            "{'I8', 'I16', 'I32', 'I64', 'F32', 'U8', 'U16', 'U32', 'U64'}",
            request->data_type.c_str());

    response->message = print_str;
    RCLCPP_WARN(this->get_logger(), "%s", print_str);
    return;
  }

  // 2) Dispatch the SDO request to FCAT
  fcat_msgs::msg::AsyncSdoReadCmd msg;

  msg.name = request->name;
  msg.sdo_index = parsed_sdo_index;
  msg.sdo_subindex = request->subindex;
  msg.data_type = request->data_type;
  msg.app_id = sdo_app_id_++;

  async_sdo_read_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "Async SDO Read issued {Name:(%s) TLC:(%s) Index:(0x%x)"
    " Subindex:(%u) DataType:(%s) AppId:(%u)}",
    msg.name.c_str(), request->tlc.c_str(), msg.sdo_index, msg.sdo_subindex,
    msg.data_type.c_str(), msg.app_id);

  // 3) Wait for the SDO Response from FCAT
  response->success = WaitForSdoResponse(response->message, msg.app_id);
}
