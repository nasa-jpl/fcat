// Copyright 2021 California Institute of Technology

#include "fcat.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <limits>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

Fcat::~Fcat() { fcat_manager_.Shutdown(); }

Fcat::Fcat(const rclcpp::NodeOptions& options)
    : FcatNode("fcat", "fcat", options),
      service_qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default),
                   rmw_qos_profile_services_default) {
  process_loop_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  topic_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  SetTimerCallbackGroup(process_loop_callback_group_);

  std::string fastcat_config_path;
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Location of the the fastcat topology YAML file";
    descriptor.read_only = true;
    fastcat_config_path = this->declare_parameter<std::string>(
        "fastcat_config_path",
        "",  // must be passed, cannot generally reason about this file path
        descriptor);
  }

  RCLCPP_INFO(this->get_logger(), "loading Yaml from %s", fastcat_config_path.c_str());
  YAML::Node node = YAML::LoadFile(fastcat_config_path);

  if (!fcat_manager_.ConfigFromYaml(node)) {
    throw std::invalid_argument("Fastcat Manager failed to process bus configuration YAML file.");
  }

  std::vector<std::string> gold_actuator_names;
  fcat_manager_.GetDeviceNamesByType(gold_actuator_names, fastcat::GOLD_ACTUATOR_STATE);

  std::vector<std::string> platinum_actuator_names;
  fcat_manager_.GetDeviceNamesByType(platinum_actuator_names, fastcat::PLATINUM_ACTUATOR_STATE);

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Total number of fastcat actuators";
    descriptor.read_only = true;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = std::numeric_limits<int64_t>::max();
    range.step = 1;
    descriptor.integer_range.push_back(range);
    this->declare_parameter<int64_t>(
        "fastcat_actuator_count",
        static_cast<int64_t>(gold_actuator_names.size() + platinum_actuator_names.size()),
        descriptor, true);
  }

  int i = 0;
  InitializeActuatorParams(gold_actuator_names, i);
  InitializeActuatorParams(platinum_actuator_names, i);

  use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "If true, FCAT creates and publishes the \"/joint_states\" topic";
    descriptor.read_only = true;
    enable_js_pub_ = this->declare_parameter<bool>("create_joint_state_pub", true, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "If true, FCAT creates and publishes individual topics for FTS devices";
    descriptor.read_only = true;
    enable_ros_wrench_pub_ =
        this->declare_parameter<bool>("create_ros_wrench_pub", true, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Set the CPU ID for the main Process() thread; this settings should be "
        "used in conjunction with setting the kernel parameter 'isolcpus=0', so "
        "that the Process() thread is the only process running on CPU0; set to -1 "
        "to disable CPU pinning; it is expected that most applications will use "
        "either 0 or -1, but there may be exceptional cases where it is necessary "
        "to pin to a different CPU core; fcat will throw a fatal error if you "
        "attempt "
        "to pin to a CPU core value greater than the number of cores available "
        "(default: 0)";
    descriptor.read_only = true;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = -1;
    range.to_value = 128;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    process_loop_cpu_id_ =
        static_cast<int>(this->declare_parameter<int64_t>("process_loop_cpu_id", 0, descriptor));
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "If true, FCAT's ethercat thread will use the FIFO scheduler, allowing "
        "it to preempt the kernel. This setting requires the realtime kernel";
    descriptor.read_only = true;
    enable_realtime_preempt_ = this->declare_parameter<bool>("enable_realtime_preempt",
                                                             enable_realtime_preempt_, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Set real-time scheduler priority level, only used if "
        "'enable_realtime_preempt' "
        "parameter is set to 'true', which requires the realtime kernel. Note "
        "that this value is used to set the 'rt_priority' real-time priority "
        "setting which is different from a processes 'niceness' setting. "
        "'rt_priority' ranges from 1 to 99 with 99 being the highest priority "
        "(default: 49), ";
    descriptor.read_only = true;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 99;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    scheduler_priority_ = static_cast<int>(
        this->declare_parameter<int64_t>("scheduler_priority", scheduler_priority_, descriptor));
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Fault the ethercat bus if large cycle slips detected (default: true)";
    descriptor.read_only = true;
    fault_on_cycle_slip_ =
        this->declare_parameter<bool>("fault_on_cycle_slip", fault_on_cycle_slip_, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Fault if cycle slip is greater than a multiple of the the nominal "
        "process loop period (default: 3.0)";
    descriptor.read_only = true;
    cycle_slip_fault_magnitude_ = this->declare_parameter<double>(
        "cycle_slip_fault_magnitude", cycle_slip_fault_magnitude_, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Delays the onset of a motion profile in explicit interpolation mode "
        "by a number of cycles of the calling module; if set to 3, then fcat "
        "will wait until 4 messages csp messages accumulate in the buffer "
        "before initiating motion profile.";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 10;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    this->declare_parameter<int64_t>("csp_explicit_interpolation_cycles_delay", 3, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "When fastcat is in CSP interpolation mode, module will transition to "
        "HOLDING state when it has not received a new CSP setpoint within the "
        "number of internal loop cycles configured by this parameter";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 4;
    range.to_value = 40;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    this->declare_parameter<int64_t>("csp_interpolation_cycles_stale", 10, descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Specify the algorithm used for explicit interpolation; one of "
        "['cubic', 'linear']: cubic interpolation is a third order "
        "interpolation of position and velocity with c2 continuity between csp "
        "set points; linear interpolation linearly interpolates both position "
        "and velocity between csp setpoints";
    this->declare_parameter<std::string>("csp_explicit_interpolation_algorithm", "cubic",
                                         descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Specify the source for the timestamp used for csp interpolation in "
        "explicit mode; If set to 'csp_message', fastcat uses the request_time "
        "from the csp message, if set to 'clock', fastcat uses the current "
        "clock on receipt of the message; ['csp_message', 'clock']";
    this->declare_parameter<std::string>("csp_explicit_interpolation_timestamp_source",
                                         "csp_message", descriptor);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "If true, fcat emits logs to report timer slips";
    this->declare_parameter<bool>("report_cycle_slips", true, descriptor);
  }

  PopulateDeviceStateFields();
  
  // Must populate certain device state fields before initializing pub/sub
  InitializePublishersAndMessages();
  InitializeSubscribers();
  InitializeServices();
  
  param_cb_handles_.push_back(
    this->add_on_set_parameters_callback(std::bind(&Fcat::SetParametersCb, this, _1)));

  // pull the Timer Rate from the Fastcat YAML
  InitializeTimerRate(fcat_manager_.GetTargetLoopRate());
  RCLCPP_INFO(this->get_logger(), "Starting Wall Timer at %lf hz",
  fcat_manager_.GetTargetLoopRate());

  loop_period_sec_ = 1.0 / fcat_manager_.GetTargetLoopRate();
  last_time_ = this->now().seconds();
  module_state_msg_.faulted = false;

  if (!fcat_manager_.InitHardware()) {
    throw std::invalid_argument("Fastcat Manager failed to initialize EtherCAT hardware.");
  }

  fcat_state_ = FcatState::INACTIVE;
  StartProcessTimer();
  fcat_state_ = FcatState::ACTIVE;
}

void Fcat::SetRealtimePreempt(int scheduler_priority) {
  // this method must be called from within a thread
  struct sched_param param;
  param.sched_priority = scheduler_priority;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_WARN(this->get_logger(),
                "Unable to set realtime FIFO scheduler with priority of "
                "%d; the realtime kernel must be installed to use this feature",
                scheduler_priority);
    return;
  }

  // Lock memory
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    RCLCPP_WARN(this->get_logger(), "Unable to lock memory; 'mlockall' failed");
  }

  // Pre-fault our stack
  const size_t max_safe_stack = 8 * 1024;
  unsigned char dummy[max_safe_stack];
  memset(dummy, 0, max_safe_stack);
}

rcl_interfaces::msg::SetParametersResult Fcat::SetParametersCb(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "csp_explicit_interpolation_cycles_delay") {
      size_t delay = static_cast<size_t>(parameter.as_int());
      fcat_manager_.SetExplicitInterpolationCyclesDelay(delay);
      result.successful = true;
    } else if (parameter.get_name() == "csp_interpolation_cycles_stale") {
      size_t cycles = static_cast<size_t>(parameter.as_int());
      fcat_manager_.SetInterpolationCyclesStale(cycles);
      result.successful = true;
    } else if (parameter.get_name() == "csp_explicit_interpolation_algorithm") {
      auto value = parameter.as_string();
      if (value == "cubic") {
        fcat_manager_.SetExplicitInterpolationAlgorithmCubic();
        result.successful = true;
      } else if (value == "linear") {
        fcat_manager_.SetExplicitInterpolationAlgorithmLinear();
        result.successful = true;
      } else {
        result.reason = "Invalid algorithm requested, must be one of ['cubic', 'linear']";
        result.successful = false;
      }
    } else if (parameter.get_name() == "csp_explicit_interpolation_timestamp_source") {
      auto value = parameter.as_string();
      if (value == "csp_message") {
        fcat_manager_.SetExplicitInterpolationTimestampSourceCspMessage();
        result.successful = true;
      } else if (value == "clock") {
        fcat_manager_.SetExplicitInterpolationTimestampSourceClock();
        result.successful = true;
      } else {
        result.reason =
            "Invalid algorithm requested, must be one of ['csp_message', "
            "'clock']";
        result.successful = false;
      }
    }
  }
  return result;
}

void Fcat::InitializeActuatorParams(const std::vector<std::string>& actuator_names, int& i) {
  fastcat::Actuator::ActuatorParams actuator_params;
  for (auto& actuator_name : actuator_names) {
    if (fcat_manager_.GetActuatorParams(actuator_name, actuator_params)) {
      std::string parameter_prefix = "fastcat_actuator_" + std::to_string(++i) + "_";
      std::string description_prefix = "Actuator " + std::to_string(i) + " ";
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "name";
        descriptor.read_only = true;
        this->declare_parameter<std::string>(parameter_prefix + "name", actuator_name, descriptor,
                                             true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "actuator type";
        descriptor.read_only = true;
        this->declare_parameter<std::string>(parameter_prefix + "actuator_type",
                                             actuator_params.actuator_type_str, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "gear ratio";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "gear_ratio", actuator_params.gear_ratio,
                                        descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "counts per revolution";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "counts_per_rev",
                                        actuator_params.counts_per_rev, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "max speed [eu/sec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "max_speed_eu_per_sec",
                                        actuator_params.max_speed_eu_per_sec, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "max acceleration [eu/sec^2]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "max_accel_eu_per_sec2",
                                        actuator_params.max_accel_eu_per_sec2, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "over speed multiplier";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "over_speed_multiplier",
                                        actuator_params.over_speed_multiplier, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "velocity tracking error [eu/sec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "vel_tracking_error_eu_per_sec",
                                        actuator_params.vel_tracking_error_eu_per_sec, descriptor,
                                        true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "position tracking error [eu]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "pos_tracking_error_eu",
                                        actuator_params.pos_tracking_error_eu, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "peak current limit [amps]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "peak_current_limit_amps",
                                        actuator_params.peak_current_limit_amps, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "peak current time [sec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "peak_current_time_sec",
                                        actuator_params.peak_current_time_sec, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "continuous current limit [amps]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "continuous_current_limit_amps",
                                        actuator_params.continuous_current_limit_amps, descriptor,
                                        true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "torque slope [amps/sec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "torque_slope_amps_per_sec",
                                        actuator_params.torque_slope_amps_per_sec, descriptor,
                                        true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "low position calibration limit [eu]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = std::numeric_limits<double>::lowest();
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "low_pos_cal_limit_eu",
                                        actuator_params.low_pos_cal_limit_eu, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "low position command limit [eu]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = std::numeric_limits<double>::lowest();
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "low_pos_cmd_limit_eu",
                                        actuator_params.low_pos_cmd_limit_eu, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "high position calibration limit [eu]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = std::numeric_limits<double>::lowest();
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "high_pos_cal_limit_eu",
                                        actuator_params.high_pos_cal_limit_eu, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "high position command limit [eu]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = std::numeric_limits<double>::lowest();
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "high_pos_cmd_limit_eu",
                                        actuator_params.high_pos_cmd_limit_eu, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "holding duration [sec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "holding_duration_sec",
                                        actuator_params.holding_duration_sec, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "brake engage time [msec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "elmo_brake_engage_msec",
                                        actuator_params.elmo_brake_engage_msec, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "brake disengage time [msec]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "elmo_brake_disengage_msec",
                                        actuator_params.elmo_brake_disengage_msec, descriptor,
                                        true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "crc value";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = std::numeric_limits<double>::lowest();
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "elmo_crc", actuator_params.elmo_crc,
                                        descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "drive max current limit [amps]";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "elmo_drive_max_cur_limit_amps",
                                        actuator_params.elmo_drive_max_cur_limit_amps, descriptor,
                                        true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "smoothing factor";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "smooth_factor",
                                        actuator_params.smooth_factor, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "torque constant";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "torque_constant",
                                        actuator_params.torque_constant, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "winding resistance";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "winding_resistance",
                                        actuator_params.winding_resistance, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "brake power";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "brake_power",
                                        actuator_params.brake_power, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "motor encoder gear ratio";
        descriptor.read_only = true;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = std::numeric_limits<double>::max();
        range.step = 0.0;
        descriptor.floating_point_range.push_back(range);
        this->declare_parameter<double>(parameter_prefix + "motor_encoder_gear_ratio",
                                        actuator_params.motor_encoder_gear_ratio, descriptor, true);
      }
      {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = description_prefix + "has absolute encoder";
        descriptor.read_only = true;
        this->declare_parameter<bool>(parameter_prefix + "has_absolute_encoder",
                                      actuator_params.actuator_absolute_encoder, descriptor, true);
      }
    }
  }
}

void Fcat::PopulateDeviceStateFields() {
  device_state_ptrs_ = fcat_manager_.GetDeviceStatePointers();

  for (auto it = device_state_ptrs_.begin(); it != device_state_ptrs_.end(); ++it) {
    RCLCPP_INFO(this->get_logger(), "Populating device_name_state_map_[%s]", (*it)->name.c_str());

    device_name_state_map_[(*it)->name] = *it;

    device_type_vec_map_[(*it)->type].push_back(*it);

    RCLCPP_INFO(this->get_logger(), "counts[%d] = %ld", (*it)->type,
                device_type_vec_map_[(*it)->type].size());
  }
}

bool Fcat::ActuatorExistsOnBus(const std::string& name) {
  std::string error_message;
  if (DeviceExistsOnBus(name, fastcat::GOLD_ACTUATOR_STATE, error_message)) {
    return true;
  } else if (DeviceExistsOnBus(name, fastcat::PLATINUM_ACTUATOR_STATE, error_message)) {
    return true;
  }
  // handle other future Actuator device types here
  RCLCPP_WARN(this->get_logger(), "%s", error_message.c_str());
  return false;
}

bool Fcat::ActuatorExistsOnBus(const std::string& name, std::string& error_message) {
  if (DeviceExistsOnBus(name, fastcat::GOLD_ACTUATOR_STATE, error_message)) {
    return true;
  } else if (DeviceExistsOnBus(name, fastcat::PLATINUM_ACTUATOR_STATE, error_message)) {
    return true;
  }
  // handle other future Actuator device types here
  return false;
}

bool Fcat::DeviceExistsOnBus(const std::string& name, fastcat::DeviceStateType type,
                             std::string& error_message) {
  char str[512];
  if (!(device_name_state_map_.end() != device_name_state_map_.find(name))) {
    snprintf(str, sizeof(str), "Device %s does not exist on bus", name.c_str());
    error_message = str;
    return false;
  }
  if (device_name_state_map_[name]->type != type) {
    snprintf(str, sizeof(str), "Device type does not match for %s", name.c_str());
    error_message = str;
    return false;
  }
  return true;
}

bool Fcat::DeviceExistsOnBus(const std::string& name, fastcat::DeviceStateType type) {
  std::string error_message;
  bool success = DeviceExistsOnBus(name, type, error_message);
  if (!success) {
    RCLCPP_WARN(this->get_logger(), "%s", error_message.c_str());
  }
  return success;
}

bool Fcat::TypeExistsOnBus(fastcat::DeviceStateType type) {
  if (device_type_vec_map_[type].size() > 0) {
    return true;
  }
  return false;
}

void Fcat::InitializePublishersAndMessages() {
  RCLCPP_INFO(this->get_logger(), "Fcat::InitializePublishers()");

  rclcpp::QoS qos_profile(publisher_queue_size_);
  qos_profile.best_effort();

  // Async SDO Response
  async_sdo_response_pub_ = this->create_publisher<fcat_msgs::msg::AsyncSdoResponse>(
      "state/async_sdo_response", qos_profile);

  // Actuators
  auto gold_vec_state_ptrs = device_type_vec_map_[fastcat::GOLD_ACTUATOR_STATE];
  auto platinum_vec_state_ptrs = device_type_vec_map_[fastcat::PLATINUM_ACTUATOR_STATE];
  size_t num_actuators = gold_vec_state_ptrs.size() + platinum_vec_state_ptrs.size();

  if (num_actuators > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating actuator pub");
    actuator_pub_ =
        this->create_publisher<fcat_msgs::msg::ActuatorStates>("state/actuators", qos_profile);

    actuator_states_msg_.names.resize(num_actuators);
    actuator_states_msg_.states.resize(num_actuators);

    if (enable_js_pub_) {
      RCLCPP_INFO(this->get_logger(), "Creating joint_states pub");

      // This is the only topic that broadcasts in the global
      // namespace. This prevents needing to remap `fcat/state/joint_states`
      // to `/joint_states` in launch files or through command line args.
      // It can always be reverted if this causes any issues down the road.
      joint_state_pub_ =
          this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos_profile);
    }
  }

  // Egd
  auto vec_state_ptrs = device_type_vec_map_[fastcat::EGD_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Egd pub");
    egd_pub_ = this->create_publisher<fcat_msgs::msg::EgdStates>("state/egds", qos_profile);

    egd_states_msg_.names.resize(vec_state_ptrs.size());
    egd_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El1008
  vec_state_ptrs = device_type_vec_map_[fastcat::EL1008_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El1008 pub");
    el1008_pub_ =
        this->create_publisher<fcat_msgs::msg::El1008States>("state/el1008s", qos_profile);

    el1008_states_msg_.names.resize(vec_state_ptrs.size());
    el1008_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El2124
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2124_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El2124 pub");
    el2124_pub_ =
        this->create_publisher<fcat_msgs::msg::El2124States>("state/el2124s", qos_profile);

    el2124_states_msg_.names.resize(vec_state_ptrs.size());
    el2124_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El2809
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2809_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El2809 pub");
    el2809_pub_ =
        this->create_publisher<fcat_msgs::msg::El2809States>("state/el2809s", qos_profile);

    el2809_states_msg_.names.resize(vec_state_ptrs.size());
    el2809_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El2798
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2798_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El2798 pub");
    el2798_pub_ =
        this->create_publisher<fcat_msgs::msg::El2798States>("state/el2798s", qos_profile);

    el2798_states_msg_.names.resize(vec_state_ptrs.size());
    el2798_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El2828
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2828_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El2828 pub");
    el2828_pub_ =
        this->create_publisher<fcat_msgs::msg::El2828States>("state/el2828s", qos_profile);

    el2828_states_msg_.names.resize(vec_state_ptrs.size());
    el2828_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3104
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3104_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3104 pub");
    el3104_pub_ =
        this->create_publisher<fcat_msgs::msg::El3104States>("state/el3104s", qos_profile);

    el3104_states_msg_.names.resize(vec_state_ptrs.size());
    el3104_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3162
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3162_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3162 pub");
    el3162_pub_ =
        this->create_publisher<fcat_msgs::msg::El3162States>("state/el3162s", qos_profile);

    el3162_states_msg_.names.resize(vec_state_ptrs.size());
    el3162_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3202
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3202_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3202 pub");
    el3202_pub_ =
        this->create_publisher<fcat_msgs::msg::El3202States>("state/el3202s", qos_profile);

    el3202_states_msg_.names.resize(vec_state_ptrs.size());
    el3202_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3208
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3208_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3208 pub");
    el3208_pub_ =
        this->create_publisher<fcat_msgs::msg::El3208States>("state/el3208s", qos_profile);

    el3208_states_msg_.names.resize(vec_state_ptrs.size());
    el3208_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3314
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3314_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3314 pub");
    el3314_pub_ =
        this->create_publisher<fcat_msgs::msg::El3314States>("state/el3314s", qos_profile);

    el3314_states_msg_.names.resize(vec_state_ptrs.size());
    el3314_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3318
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3318_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3318 pub");
    el3318_pub_ =
        this->create_publisher<fcat_msgs::msg::El3318States>("state/el3318s", qos_profile);

    el3318_states_msg_.names.resize(vec_state_ptrs.size());
    el3318_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3602
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3602_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El3602 pub");
    el3602_pub_ =
        this->create_publisher<fcat_msgs::msg::El3602States>("state/el3602s", qos_profile);

    el3602_states_msg_.names.resize(vec_state_ptrs.size());
    el3602_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El4102
  vec_state_ptrs = device_type_vec_map_[fastcat::EL4102_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El4102 pub");
    el4102_pub_ =
        this->create_publisher<fcat_msgs::msg::El4102States>("state/el4102s", qos_profile);

    el4102_states_msg_.names.resize(vec_state_ptrs.size());
    el4102_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El5042
  vec_state_ptrs = device_type_vec_map_[fastcat::EL5042_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating El5042 pub");
    el5042_pub_ =
        this->create_publisher<fcat_msgs::msg::El5042States>("state/el5042s", qos_profile);

    el5042_states_msg_.names.resize(vec_state_ptrs.size());
    el5042_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // ILD1900
  vec_state_ptrs = device_type_vec_map_[fastcat::ILD1900_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Ild1900 pub");
    ild1900_pub_ =
        this->create_publisher<fcat_msgs::msg::Ild1900States>("state/ild1900s", qos_profile);

    ild1900_states_msg_.names.resize(vec_state_ptrs.size());
    ild1900_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Commander
  vec_state_ptrs = device_type_vec_map_[fastcat::COMMANDER_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Commander pub");
    commander_pub_ =
        this->create_publisher<fcat_msgs::msg::CommanderStates>("state/commanders", qos_profile);

    commander_states_msg_.names.resize(vec_state_ptrs.size());
    commander_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Conditional
  vec_state_ptrs = device_type_vec_map_[fastcat::CONDITIONAL_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Conditional pub");
    conditional_pub_ = this->create_publisher<fcat_msgs::msg::ConditionalStates>(
        "state/conditionals", qos_profile);

    conditional_states_msg_.names.resize(vec_state_ptrs.size());
    conditional_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Faulter
  vec_state_ptrs = device_type_vec_map_[fastcat::FAULTER_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Faulter pub");
    faulter_pub_ =
        this->create_publisher<fcat_msgs::msg::FaulterStates>("state/faulters", qos_profile);

    faulter_states_msg_.names.resize(vec_state_ptrs.size());
    faulter_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Filter
  vec_state_ptrs = device_type_vec_map_[fastcat::FILTER_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Filter pub");
    filter_pub_ = this->create_publisher<fcat_msgs::msg::FilterStates>("state/filter", qos_profile);

    filter_states_msg_.names.resize(vec_state_ptrs.size());
    filter_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Function
  vec_state_ptrs = device_type_vec_map_[fastcat::FUNCTION_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Function pub");
    function_pub_ =
        this->create_publisher<fcat_msgs::msg::FunctionStates>("state/functions", qos_profile);

    function_states_msg_.names.resize(vec_state_ptrs.size());
    function_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Pid
  vec_state_ptrs = device_type_vec_map_[fastcat::PID_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Pid pub");
    pid_pub_ = this->create_publisher<fcat_msgs::msg::PidStates>("state/pids", qos_profile);

    pid_states_msg_.names.resize(vec_state_ptrs.size());
    pid_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Saturation
  vec_state_ptrs = device_type_vec_map_[fastcat::SATURATION_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Saturation pub");
    saturation_pub_ =
        this->create_publisher<fcat_msgs::msg::SaturationStates>("state/saturations", qos_profile);

    saturation_states_msg_.names.resize(vec_state_ptrs.size());
    saturation_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Schmitt Trigger
  vec_state_ptrs = device_type_vec_map_[fastcat::SCHMITT_TRIGGER_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Schmitt Trigger pub");
    schmitt_trigger_pub_ = this->create_publisher<fcat_msgs::msg::SchmittTriggerStates>(
        "state/schmitt_triggers", qos_profile);

    schmitt_trigger_states_msg_.names.resize(vec_state_ptrs.size());
    schmitt_trigger_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Signal Generator
  vec_state_ptrs = device_type_vec_map_[fastcat::SIGNAL_GENERATOR_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating Signal Generator pub");
    signal_generator_pub_ = this->create_publisher<fcat_msgs::msg::SignalGeneratorStates>(
        "state/signal_generators", qos_profile);

    signal_generator_states_msg_.names.resize(vec_state_ptrs.size());
    signal_generator_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Linear Interpolation
  vec_state_ptrs = device_type_vec_map_[fastcat::LINEAR_INTERPOLATION_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating LinearInterpolation pub");
    linear_interpolation_pub_ = this->create_publisher<fcat_msgs::msg::LinearInterpolationStates>(
        "state/linear_interpolators", qos_profile);

    linear_interpolation_states_msg_.names.resize(vec_state_ptrs.size());
    linear_interpolation_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Three Node Thermal Model
  vec_state_ptrs = device_type_vec_map_[fastcat::THREE_NODE_THERMAL_MODEL_STATE];
  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating ThreeNodeThermalModel pub");
    three_node_thermal_model_pub_ =
        this->create_publisher<fcat_msgs::msg::ThreeNodeThermalModelStates>(
            "state/three_node_thermal_models", qos_profile);

    three_node_thermal_model_states_msg_.names.resize(vec_state_ptrs.size());
    three_node_thermal_model_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // FTS, AtiFts, VirtualFts
  vec_state_ptrs = device_type_vec_map_[fastcat::FTS_STATE];

  if (vec_state_ptrs.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Creating FTS pub");

    fts_pub_ = this->create_publisher<fcat_msgs::msg::FtsStates>("state/fts", qos_profile);

    fts_states_msg_.names.resize(vec_state_ptrs.size());
    fts_states_msg_.states.resize(vec_state_ptrs.size());

    if (enable_ros_wrench_pub_) {
      for (auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end();
           dev_state_ptr++) {
        std::string device = (*dev_state_ptr)->name;
        fts_raw_pub_map_[device] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "fts/" + device + "/raw_wrench", qos_profile);

        fts_tared_pub_map_[device] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "fts/" + device + "/tared_wrench", qos_profile);

        RCLCPP_INFO(this->get_logger(), "Creating ROS WrenchStamped Topics for %s", device.c_str());
      }
    }
  }

  // Module State (always published)
  module_state_pub_ =
      this->create_publisher<fcat_msgs::msg::ModuleState>("state/module_state", qos_profile);
}

void Fcat::InitializeSubscribers() {
  rclcpp::SubscriptionOptions options;

  options.callback_group = topic_callback_group_;

  // bus reset/fault
  subscriptions_.push_back(this->create_subscription<std_msgs::msg::Empty>(
      "impl/reset", 1, std::bind(&Fcat::ResetCmdCb, this, _1), options));

  subscriptions_.push_back(this->create_subscription<std_msgs::msg::Empty>(
      "impl/fault", 1, std::bind(&Fcat::FaultCmdCb, this, _1), options));

  // Async SDO
  subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::AsyncSdoReadCmd>(
      "impl/async_sdo_read", subscription_queue_size_,
      std::bind(&Fcat::AsyncSdoReadCmdCb, this, _1), options));

  subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::AsyncSdoWriteCmd>(
      "impl/async_sdo_write", subscription_queue_size_,
      std::bind(&Fcat::AsyncSdoWriteCmdCb, this, _1), options));

  // Actuator
  if (TypeExistsOnBus(fastcat::GOLD_ACTUATOR_STATE) ||
      TypeExistsOnBus(fastcat::PLATINUM_ACTUATOR_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCspCmd>(
        "impl/actuator_csp", subscription_queue_size_, std::bind(&Fcat::ActuatorCSPCmdCb, this, _1),
        options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCsvCmd>(
        "impl/actuator_csv", subscription_queue_size_, std::bind(&Fcat::ActuatorCSVCmdCb, this, _1),
        options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCstCmd>(
        "impl/actuator_cst", subscription_queue_size_, std::bind(&Fcat::ActuatorCSTCmdCb, this, _1),
        options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCspCmds>(
        "impl/actuator_csp_multi", subscription_queue_size_,
        std::bind(&Fcat::ActuatorCSPCmdsCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCsvCmds>(
        "impl/actuator_csv_multi", subscription_queue_size_,
        std::bind(&Fcat::ActuatorCSVCmdsCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCstCmds>(
        "impl/actuator_cst_multi", subscription_queue_size_,
        std::bind(&Fcat::ActuatorCSTCmdsCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorCalibrateCmd>(
        "impl/actuator_calibrate", subscription_queue_size_,
        std::bind(&Fcat::ActuatorCalibrateCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorProfPosCmd>(
        "impl/actuator_prof_pos", subscription_queue_size_,
        std::bind(&Fcat::ActuatorProfPosCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorProfTorqueCmd>(
        "impl/actuator_prof_torque", subscription_queue_size_,
        std::bind(&Fcat::ActuatorProfTorqueCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorProfVelCmd>(
        "impl/actuator_prof_vel", subscription_queue_size_,
        std::bind(&Fcat::ActuatorProfVelCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorProfPosCmds>(
        "impl/actuator_prof_pos_multi", subscription_queue_size_,
        std::bind(&Fcat::ActuatorProfPosCmdsCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorSetOutputPositionCmd>(
            "impl/actuator_set_output_position", subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetOutputPositionCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorSetDigitalOutputCmd>(
        "impl/actuator_set_digital_output", subscription_queue_size_,
        std::bind(&Fcat::ActuatorSetDigitalOutputCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorSetMaxCurrentCmd>(
        "impl/actuator_set_max_current", subscription_queue_size_,
        std::bind(&Fcat::ActuatorSetMaxCurrentCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorSetUnitModeCmd>(
        "impl/actuator_set_unit_mode", subscription_queue_size_,
        std::bind(&Fcat::ActuatorSetUnitModeCmdCb, this, _1), options));

    subscriptions_.push_back(
        this->create_subscription<fcat_msgs::msg::ActuatorSetProfDisengagingTimeoutCmd>(
            "impl/actuator_set_prof_disengaging_timeout", subscription_queue_size_,
            std::bind(&Fcat::ActuatorSetProfDisengagingTimeoutCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorHaltCmd>(
        "impl/actuator_halt", subscription_queue_size_,
        std::bind(&Fcat::ActuatorHaltCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::ActuatorHaltCmds>(
        "impl/actuator_halt_multi", subscription_queue_size_,
        std::bind(&Fcat::ActuatorHaltCmdsCb, this, _1), options));
  }

  // Commander
  if (TypeExistsOnBus(fastcat::COMMANDER_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::CommanderEnableCmd>(
        "impl/commander_enable", subscription_queue_size_,
        std::bind(&Fcat::CommanderEnableCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::CommanderDisableCmd>(
        "impl/commander_disable", subscription_queue_size_,
        std::bind(&Fcat::CommanderDisableCmdCb, this, _1), options));
  }

  // El2124
  if (TypeExistsOnBus(fastcat::EL2124_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2124WriteAllChannelsCmd>(
        "impl/el2124_write_all_channels", subscription_queue_size_,
        std::bind(&Fcat::El2124WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2124WriteChannelCmd>(
        "impl/el2124_write_channel", subscription_queue_size_,
        std::bind(&Fcat::El2124WriteChannelCmdCb, this, _1), options));
  }

  // El2809
  if (TypeExistsOnBus(fastcat::EL2809_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2809WriteAllChannelsCmd>(
        "impl/el2809_write_all_channels", subscription_queue_size_,
        std::bind(&Fcat::El2809WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2809WriteChannelCmd>(
        "impl/el2809_write_channel", subscription_queue_size_,
        std::bind(&Fcat::El2809WriteChannelCmdCb, this, _1), options));
  }

  // El2798
  if (TypeExistsOnBus(fastcat::EL2798_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2798WriteAllChannelsCmd>(
        "impl/el2798_write_all_channels", subscription_queue_size_,
        std::bind(&Fcat::El2798WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2798WriteChannelCmd>(
        "impl/el2798_write_channel", subscription_queue_size_,
        std::bind(&Fcat::El2798WriteChannelCmdCb, this, _1), options));
  }

  // El2828
  if (TypeExistsOnBus(fastcat::EL2828_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2828WriteAllChannelsCmd>(
        "impl/el2828_write_all_channels", subscription_queue_size_,
        std::bind(&Fcat::El2828WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El2828WriteChannelCmd>(
        "impl/el2828_write_channel", subscription_queue_size_,
        std::bind(&Fcat::El2828WriteChannelCmdCb, this, _1), options));
  }

  // EL4102
  if (TypeExistsOnBus(fastcat::EL4102_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El4102WriteAllChannelsCmd>(
        "impl/el4102_write_all_channels", subscription_queue_size_,
        std::bind(&Fcat::El4102WriteAllChannelsCmdCb, this, _1), options));

    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::El4102WriteChannelCmd>(
        "impl/el4102_write_channel", subscription_queue_size_,
        std::bind(&Fcat::El4102WriteChannelCmdCb, this, _1), options));
  }

  // Faulter
  if (TypeExistsOnBus(fastcat::FAULTER_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::FaulterEnableCmd>(
        "impl/faulter_enable", subscription_queue_size_,
        std::bind(&Fcat::FaulterEnableCmdCb, this, _1), options));
  }

  // Fts
  if (TypeExistsOnBus(fastcat::FTS_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::FtsTareCmd>(
        "impl/fts_tare", subscription_queue_size_, std::bind(&Fcat::FtsTareCmdCb, this, _1),
        options));
  }

  // Pid
  if (TypeExistsOnBus(fastcat::PID_STATE)) {
    subscriptions_.push_back(this->create_subscription<fcat_msgs::msg::PidActivateCmd>(
        "impl/pid_activate", subscription_queue_size_, std::bind(&Fcat::PidActivateCmdCb, this, _1),
        options));
  }
}

void Fcat::InitializeServices() {
  // bus reset/fault
  services_.push_back(this->create_service<std_srvs::srv::Trigger>(
      "cmd/reset", std::bind(&Fcat::ResetSrvCb, this, _1, _2), service_qos_));

  services_.push_back(this->create_service<std_srvs::srv::Trigger>(
      "cmd/fault", std::bind(&Fcat::FaultSrvCb, this, _1, _2), service_qos_));

  // Actuator
  if (TypeExistsOnBus(fastcat::GOLD_ACTUATOR_STATE) ||
      TypeExistsOnBus(fastcat::PLATINUM_ACTUATOR_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorHaltService>(
        "cmd/actuator_halt", std::bind(&Fcat::ActuatorHaltSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorSetGainSchedulingIndexService>(
        "cmd/actuator_set_gain_scheduling_index",
        std::bind(&Fcat::ActuatorSetGainSchedulingIndexSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorSetMaxCurrentService>(
        "cmd/actuator_set_max_current", std::bind(&Fcat::ActuatorSetMaxCurrentSrvCb, this, _1, _2),
        service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorSetOutputPositionService>(
        "cmd/actuator_set_output_position",
        std::bind(&Fcat::ActuatorSetOutputPositionSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorSetDigitalOutputService>(
        "cmd/actuator_set_digital_output",
        std::bind(&Fcat::ActuatorSetDigitalOutputSrvCb, this, _1, _2), service_qos_));

    services_.push_back(
        this->create_service<fcat_msgs::srv::ActuatorSetProfDisengagingTimeoutService>(
            "cmd/actuator_set_prof_disengaging_timeout",
            std::bind(&Fcat::ActuatorSetProfDisengagingTimeoutSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorProfPosService>(
        "cmd/actuator_prof_pos", std::bind(&Fcat::ActuatorProfPosSrvCb, this, _1, _2),
        service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorProfVelService>(
        "cmd/actuator_prof_vel", std::bind(&Fcat::ActuatorProfVelSrvCb, this, _1, _2),
        service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorProfTorqueService>(
        "cmd/actuator_prof_torque", std::bind(&Fcat::ActuatorProfTorqueSrvCb, this, _1, _2),
        service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::ActuatorCalibrateService>(
        "cmd/actuator_calibrate", std::bind(&Fcat::ActuatorCalibrateSrvCb, this, _1, _2),
        service_qos_));
  }  // end Actuator Service Declarations

  // Commander
  if (TypeExistsOnBus(fastcat::COMMANDER_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::CommanderEnableService>(
        "cmd/commander_enable", std::bind(&Fcat::CommanderEnableSrvCb, this, _1, _2),
        service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::CommanderDisableService>(
        "cmd/commander_disable", std::bind(&Fcat::CommanderDisableSrvCb, this, _1, _2),
        service_qos_));
  }  // end Commander Service Declarations

  // El2124
  if (TypeExistsOnBus(fastcat::EL2124_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::El2124WriteAllChannelsService>(
        "cmd/el2124_write_all_channels",
        std::bind(&Fcat::El2124WriteAllChannelsSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::El2124WriteChannelService>(
        "cmd/el2124_write_channel", std::bind(&Fcat::El2124WriteChannelSrvCb, this, _1, _2),
        service_qos_));
  }  // end El2124 Service Declarations

  // El2809
  if (TypeExistsOnBus(fastcat::EL2809_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::El2809WriteAllChannelsService>(
        "cmd/el2809_write_all_channels",
        std::bind(&Fcat::El2809WriteAllChannelsSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::El2809WriteChannelService>(
        "cmd/el2809_write_channel", std::bind(&Fcat::El2809WriteChannelSrvCb, this, _1, _2),
        service_qos_));
  }  // end El2809 Service Declarations

  // El2798
  if (TypeExistsOnBus(fastcat::EL2798_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::El2798WriteAllChannelsService>(
        "cmd/el2798_write_all_channels",
        std::bind(&Fcat::El2798WriteAllChannelsSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::El2798WriteChannelService>(
        "cmd/el2798_write_channel", std::bind(&Fcat::El2798WriteChannelSrvCb, this, _1, _2),
        service_qos_));
  }  // end El2798 Service Declarations

  // El2828
  if (TypeExistsOnBus(fastcat::EL2828_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::El2828WriteAllChannelsService>(
        "cmd/el2828_write_all_channels",
        std::bind(&Fcat::El2828WriteAllChannelsSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::El2828WriteChannelService>(
        "cmd/el2828_write_channel", std::bind(&Fcat::El2828WriteChannelSrvCb, this, _1, _2),
        service_qos_));
  }  // end El2828 Service Declarations

  // El4102
  if (TypeExistsOnBus(fastcat::EL4102_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::El4102WriteAllChannelsService>(
        "cmd/el4102_write_all_channels",
        std::bind(&Fcat::El4102WriteAllChannelsSrvCb, this, _1, _2), service_qos_));

    services_.push_back(this->create_service<fcat_msgs::srv::El4102WriteChannelService>(
        "cmd/el4102_write_channel", std::bind(&Fcat::El4102WriteChannelSrvCb, this, _1, _2),
        service_qos_));
  }  // end El4102 Service Declarations

  // Faulter
  if (TypeExistsOnBus(fastcat::FAULTER_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::FaulterEnableService>(
        "cmd/faulter_enable", std::bind(&Fcat::FaulterEnableSrvCb, this, _1, _2), service_qos_));
  }  // end Faulter Service Declarations

  // FTS
  if (TypeExistsOnBus(fastcat::FTS_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::DeviceTriggerService>(
        "cmd/fts_tare", std::bind(&Fcat::FtsTareSrvCb, this, _1, _2), service_qos_));
  }  // end FTS Service Declarations

  // PID
  if (TypeExistsOnBus(fastcat::PID_STATE)) {
    services_.push_back(this->create_service<fcat_msgs::srv::PidActivateService>(
        "cmd/pid_activate", std::bind(&Fcat::PidActivateSrvCb, this, _1, _2), service_qos_));
  }  // end PID Service Declarations
}

void Fcat::SetCpuAffinity() {
#ifdef _GNU_SOURCE
  if (gettid() != getpid()) {
    int cpus_available = get_nprocs_conf();
    RCLCPP_INFO(this->get_logger(), "This system has %d CPUs configured", cpus_available);
    if (process_loop_cpu_id_ >= cpus_available) {
      RCLCPP_FATAL(this->get_logger(),
                   "User requested to pin Process() function to CPU ID %d, but "
                   "only %d CPUs are available on this machine",
                   process_loop_cpu_id_, cpus_available);
      rclcpp::shutdown();
    } else {
      cpu_set_t affinity;
      CPU_ZERO(&affinity);
      CPU_SET(process_loop_cpu_id_, &affinity);

      RCLCPP_INFO(this->get_logger(), "Setting CPU affinity for thread ID: %d", gettid());
      int result = sched_setaffinity(gettid(), sizeof(cpu_set_t), &affinity);
      if (result != 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not set process affinity to CPU %d for thread ID: %d, "
                    "sched_setaffinity returned result: %d",
                    process_loop_cpu_id_, gettid(), result);
      } else {
        RCLCPP_INFO(this->get_logger(), "Set CPU affinity to CPU %d for thread ID: %d",
                    process_loop_cpu_id_, gettid());
      }
      process_loop_thread_id_ = gettid();
    }
  }
#else
  RCLCPP_WARN(this->get_logger(),
              "GNU extensions are not available on this machine/compiler; the Process() "
              "loop "
              "cannot be moved to CPU %d",
              process_loop_thread_id_);
#endif
}

void Fcat::Process() {
  fprintf(stderr, "Handling Process() loop\n");
  auto now = this->get_clock()->now();

  bool report_cycle_slips = this->get_parameter("report_cycle_slips").as_bool();
  if (report_cycle_slips && time_stamp_initialized_ && cpu_affinity_initialized_) {
    double dt = now.seconds() - publish_time_stamp_.seconds();
    if (dt > 2.0 * loop_period_sec_) {
      RCLCPP_WARN(this->get_logger(),
                  "Cycle slip detected; seconds since last Process() call: "
                  "%f",
                  dt);
    }
    if (fault_on_cycle_slip_ && dt > cycle_slip_fault_magnitude_ * loop_period_sec_) {
      fcat_manager_.ExecuteAllDeviceFaults();
      const std::shared_ptr<std_msgs::msg::Empty> msg;
      Fault(); // TODO: use lifecyle nodes
      publish_time_stamp_ = now;
      return;
    }
  }

  publish_time_stamp_ = now;
  time_stamp_initialized_ = true;

#ifdef _GNU_SOURCE
  // sched_affinity is a GNU extension, and may not be available in some
  // compilers
  if (!cpu_affinity_initialized_) {
    if (process_loop_cpu_id_ < 0) {
      // no thread pinning requested
      cpu_affinity_initialized_ = true;
    } else if (gettid() != getpid()) {
      int cpus_available = get_nprocs_conf();
      RCLCPP_INFO(this->get_logger(), "This system has %d CPUs configured", cpus_available);
      if (process_loop_cpu_id_ >= cpus_available) {
        RCLCPP_FATAL(this->get_logger(),
                     "User requested to pin Process() function to CPU ID %d, but "
                     "only %d CPUs are available on this machine",
                     process_loop_cpu_id_, cpus_available);
        rclcpp::shutdown();
      } else {
        cpu_set_t affinity;
        CPU_ZERO(&affinity);
        CPU_SET(process_loop_cpu_id_, &affinity);

        RCLCPP_INFO(this->get_logger(), "Process loop started in thread ID: %d", gettid());
        int result = sched_setaffinity(gettid(), sizeof(cpu_set_t), &affinity);
        if (result != 0) {
          RCLCPP_WARN(this->get_logger(),
                      "Could not set process affinity to CPU %d for thread ID: %d, "
                      "sched_setaffinity returned result: %d",
                      process_loop_cpu_id_, gettid(), result);
        } else {
          RCLCPP_INFO(this->get_logger(), "Set CPU affinity to CPU %d for thread ID: %d",
                      process_loop_cpu_id_, gettid());
        }
        process_loop_thread_id_ = gettid();
        cpu_affinity_initialized_ = true;
      }
    }
  }
#else
  RCLCPP_WARN(this->get_logger(),
              "GNU extensions are not available on this machine/compiler; the Process() "
              "loop cannot be moved to CPU %d",
              process_loop_thread_id_);
  cpu_affinity_initialized_ = true;
#endif

  if (!process_loop_realtime_preempt_initialized_) {
    if (enable_realtime_preempt_) {
      RCLCPP_INFO(this->get_logger(), "Setting realtime priority for process loop to %d",
                  scheduler_priority_);
      SetRealtimePreempt(scheduler_priority_);
    }
    process_loop_realtime_preempt_initialized_ = true;
  }

  if (use_sim_time_) {
    fcat_manager_.Process(publish_time_stamp_.seconds());
  } else {
    fcat_manager_.Process();
  }

  PublishAsyncSdoResponse();
  PublishFcatModuleState();
  PublishFtsStates();
  PublishActuatorStates();
  PublishEgdStates();
  PublishEl1008States();
  PublishEl2124States();
  PublishEl2809States();
  PublishEl2798States();
  PublishEl2828States();
  PublishEl3104States();
  PublishEl3162States();
  PublishEl3202States();
  PublishEl3208States();
  PublishEl3314States();
  PublishEl3318States();
  PublishEl3602States();
  PublishEl4102States();
  PublishEl5042States();
  PublishIld1900States();

  PublishCommanderStates();
  PublishConditionalStates();
  PublishFaulterStates();
  PublishFilterStates();
  PublishFunctionStates();
  PublishPidStates();
  PublishSaturationStates();
  PublishSchmittTriggerStates();
  PublishSignalGeneratorStates();
  PublishLinearInterpolationStates();
  PublishThreeNodeThermalModelStates();
}

void Fcat::PublishFcatModuleState() {
  // Check for fault status and emit logs
  bool is_faulted = fcat_manager_.IsFaulted();
  if (!module_state_msg_.faulted && is_faulted) {
    Fault(); // TODO: use lifecycle nodes
    RCLCPP_WARN(this->get_logger(), "Fastcat bus fault detected");
  } else if (module_state_msg_.faulted && !is_faulted) {
    RCLCPP_INFO(this->get_logger(), "Fastcat bus fault cleared");
  }

  module_state_msg_.faulted = is_faulted;
  module_state_msg_.command_queue_size = command_queue_size_;
  command_queue_size_ = 0;

  // Populate the rest of the message
  double t = this->now().seconds();

  // jitter is defined as the delta between the last time the module
  // was serviced and the nominal loop period. e.g. if a module
  // is serviced exactly at its nominal loop period, it will have zero jitter
  module_state_msg_.jitter = t - last_time_ - loop_period_sec_;
  last_time_ = t;

  module_state_msg_.header.stamp = publish_time_stamp_;

  module_state_pub_->publish(module_state_msg_);
}

void Fcat::PublishAsyncSdoResponse() {
  fastcat::SdoResponse sdo_resp;
  if (fcat_manager_.PopSdoResponseQueue(sdo_resp)) {
    fcat_msgs::msg::AsyncSdoResponse msg;

    msg.device_name = sdo_resp.device_name;
    msg.request_type = jsd_sdo_request_type_to_string(sdo_resp.response.request_type);
    msg.sdo_index = sdo_resp.response.sdo_index;
    msg.sdo_subindex = sdo_resp.response.sdo_subindex;
    msg.data_type = jsd_sdo_data_type_to_string(sdo_resp.response.data_type);
    msg.app_id = sdo_resp.response.app_id;
    msg.success = sdo_resp.response.success;

    msg.data = jsd_sdo_data_to_string(sdo_resp.response.data_type, sdo_resp.response.data);

    fprintf(stderr, "Publishing new AsyncSdoResponse\n");

    async_sdo_response_pub_->publish(msg);
  }
}

void Fcat::PublishFtsStates() {
  // FTS, AtiFts, VirtualFts
  auto vec_state_ptrs = device_type_vec_map_[fastcat::FTS_STATE];
  if (vec_state_ptrs.size() > 0) {
    size_t index = 0;

    fts_states_msg_.header.stamp = publish_time_stamp_;
    for (auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end();
         dev_state_ptr++) {
      // Aggregate FTS state message
      fts_states_msg_.names[index] = (*dev_state_ptr)->name;
      fts_states_msg_.states[index] = FtsStateToMsg(*dev_state_ptr);
      index++;

      if (enable_ros_wrench_pub_) {
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();

        wrench_msg.header.stamp = publish_time_stamp_;

        std::string device = (*dev_state_ptr)->name;
        fastcat::FtsState fts_state = (*dev_state_ptr)->fts_state;

        // Publish Raw FTS
        wrench_msg.header.stamp = publish_time_stamp_;

        wrench_msg.wrench.force.x = fts_state.raw_fx;
        wrench_msg.wrench.force.y = fts_state.raw_fy;
        wrench_msg.wrench.force.z = fts_state.raw_fz;

        wrench_msg.wrench.torque.x = fts_state.raw_tx;
        wrench_msg.wrench.torque.y = fts_state.raw_ty;
        wrench_msg.wrench.torque.z = fts_state.raw_tz;

        fts_raw_pub_map_[device]->publish(wrench_msg);

        // Publish Tared FTS
        wrench_msg.header.stamp = publish_time_stamp_;

        wrench_msg.wrench.force.x = fts_state.tared_fx;
        wrench_msg.wrench.force.y = fts_state.tared_fy;
        wrench_msg.wrench.force.z = fts_state.tared_fz;

        wrench_msg.wrench.torque.x = fts_state.tared_tx;
        wrench_msg.wrench.torque.y = fts_state.tared_ty;
        wrench_msg.wrench.torque.z = fts_state.tared_tz;

        fts_tared_pub_map_[device]->publish(wrench_msg);
      }
    }

    // dont forget to publish the aggregated fts states message
    fts_pub_->publish(fts_states_msg_);
  }
}

void Fcat::PublishActuatorStates() {
  auto& gold_act_state_vec = device_type_vec_map_[fastcat::GOLD_ACTUATOR_STATE];
  auto& platinum_act_state_vec = device_type_vec_map_[fastcat::PLATINUM_ACTUATOR_STATE];
  size_t num_actuators = gold_act_state_vec.size() + platinum_act_state_vec.size();

  if (num_actuators > 0) {
    size_t index = 0;
    auto js_msg = sensor_msgs::msg::JointState();
    js_msg.name.reserve(num_actuators);
    js_msg.position.reserve(num_actuators);
    js_msg.velocity.reserve(num_actuators);

    actuator_states_msg_.header.stamp = publish_time_stamp_;
    js_msg.header.stamp = publish_time_stamp_;

    for (auto state_ptr : gold_act_state_vec) {
      // Populate ActuatorStates message
      actuator_states_msg_.names[index] = state_ptr->name;
      actuator_states_msg_.states[index] = ActuatorStateToMsg(state_ptr);
      index++;

      // Populate JointState message
      js_msg.name.push_back(state_ptr->name);
      js_msg.position.push_back(state_ptr->gold_actuator_state.actual_position);
      js_msg.velocity.push_back(state_ptr->gold_actuator_state.actual_velocity);
    }

    for (auto state_ptr : platinum_act_state_vec) {
      // Populate ActuatorStates message
      actuator_states_msg_.names[index] = state_ptr->name;
      actuator_states_msg_.states[index] = ActuatorStateToMsg(state_ptr);
      index++;

      // Populate JointState message
      js_msg.name.push_back(state_ptr->name);
      js_msg.position.push_back(state_ptr->platinum_actuator_state.actual_position);
      js_msg.velocity.push_back(state_ptr->platinum_actuator_state.actual_velocity);
    }

    actuator_pub_->publish(actuator_states_msg_);

    js_msg.header.stamp = this->now();
    if (enable_js_pub_) {
      joint_state_pub_->publish(js_msg);
    }
  }
}

void Fcat::PublishEgdStates() {
  auto state_vec = device_type_vec_map_[fastcat::EGD_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    egd_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      egd_states_msg_.names[index] = (*state_ptr)->name;
      egd_states_msg_.states[index] = EgdStateToMsg(*state_ptr);
      index++;
    }
    egd_pub_->publish(egd_states_msg_);
  }
}

void Fcat::PublishThreeNodeThermalModelStates() {
  // FTS, AtiFts, VirtualFts
  auto vec_state_ptrs = device_type_vec_map_[fastcat::THREE_NODE_THERMAL_MODEL_STATE];
  if (vec_state_ptrs.size() > 0) {
    size_t index = 0;

    three_node_thermal_model_states_msg_.header.stamp = publish_time_stamp_;
    for (auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end();
         dev_state_ptr++) {
      three_node_thermal_model_states_msg_.names[index] = (*dev_state_ptr)->name;
      three_node_thermal_model_states_msg_.states[index] =
          ThreeNodeThermalModelStateToMsg(*dev_state_ptr);
      index++;
    }
    three_node_thermal_model_pub_->publish(three_node_thermal_model_states_msg_);
  }
}

void Fcat::PublishEl1008States() {
  auto state_vec = device_type_vec_map_[fastcat::EL1008_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el1008_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el1008_states_msg_.names[index] = (*state_ptr)->name;
      el1008_states_msg_.states[index] = El1008StateToMsg(*state_ptr);
      index++;
    }
    el1008_pub_->publish(el1008_states_msg_);
  }
}

void Fcat::PublishEl2124States() {
  auto state_vec = device_type_vec_map_[fastcat::EL2124_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el2124_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el2124_states_msg_.names[index] = (*state_ptr)->name;
      el2124_states_msg_.states[index] = El2124StateToMsg(*state_ptr);
      index++;
    }
    el2124_pub_->publish(el2124_states_msg_);
  }
}

void Fcat::PublishEl2809States() {
  auto state_vec = device_type_vec_map_[fastcat::EL2809_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el2809_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el2809_states_msg_.names[index] = (*state_ptr)->name;
      el2809_states_msg_.states[index] = El2809StateToMsg(*state_ptr);
      index++;
    }
    el2809_pub_->publish(el2809_states_msg_);
  }
}

void Fcat::PublishEl2798States() {
  auto state_vec = device_type_vec_map_[fastcat::EL2798_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el2798_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el2798_states_msg_.names[index] = (*state_ptr)->name;
      el2798_states_msg_.states[index] = El2798StateToMsg(*state_ptr);
      index++;
    }
    el2798_pub_->publish(el2798_states_msg_);
  }
}

void Fcat::PublishEl2828States() {
  auto state_vec = device_type_vec_map_[fastcat::EL2828_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el2828_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el2828_states_msg_.names[index] = (*state_ptr)->name;
      el2828_states_msg_.states[index] = El2828StateToMsg(*state_ptr);
      index++;
    }
    el2828_pub_->publish(el2828_states_msg_);
  }
}

void Fcat::PublishEl3104States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3104_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3104_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3104_states_msg_.names[index] = (*state_ptr)->name;
      el3104_states_msg_.states[index] = El3104StateToMsg(*state_ptr);
      index++;
    }
    el3104_pub_->publish(el3104_states_msg_);
  }
}

void Fcat::PublishEl3162States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3162_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3162_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3162_states_msg_.names[index] = (*state_ptr)->name;
      el3162_states_msg_.states[index] = El3162StateToMsg(*state_ptr);
      index++;
    }
    el3162_pub_->publish(el3162_states_msg_);
  }
}

void Fcat::PublishEl3202States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3202_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3202_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3202_states_msg_.names[index] = (*state_ptr)->name;
      el3202_states_msg_.states[index] = El3202StateToMsg(*state_ptr);
      index++;
    }
    el3202_pub_->publish(el3202_states_msg_);
  }
}

void Fcat::PublishEl3208States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3208_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3208_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3208_states_msg_.names[index] = (*state_ptr)->name;
      el3208_states_msg_.states[index] = El3208StateToMsg(*state_ptr);
      index++;
    }
    el3208_pub_->publish(el3208_states_msg_);
  }
}

void Fcat::PublishEl3314States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3314_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3314_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3314_states_msg_.names[index] = (*state_ptr)->name;
      el3314_states_msg_.states[index] = El3314StateToMsg(*state_ptr);
      index++;
    }
    el3314_pub_->publish(el3314_states_msg_);
  }
}

void Fcat::PublishEl3318States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3318_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3318_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3318_states_msg_.names[index] = (*state_ptr)->name;
      el3318_states_msg_.states[index] = El3318StateToMsg(*state_ptr);
      index++;
    }
    el3318_pub_->publish(el3318_states_msg_);
  }
}

void Fcat::PublishEl3602States() {
  auto state_vec = device_type_vec_map_[fastcat::EL3602_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el3602_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el3602_states_msg_.names[index] = (*state_ptr)->name;
      el3602_states_msg_.states[index] = El3602StateToMsg(*state_ptr);
      index++;
    }
    el3602_pub_->publish(el3602_states_msg_);
  }
}

void Fcat::PublishEl4102States() {
  auto state_vec = device_type_vec_map_[fastcat::EL4102_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el4102_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el4102_states_msg_.names[index] = (*state_ptr)->name;
      el4102_states_msg_.states[index] = El4102StateToMsg(*state_ptr);
      index++;
    }
    el4102_pub_->publish(el4102_states_msg_);
  }
}

void Fcat::PublishEl5042States() {
  auto state_vec = device_type_vec_map_[fastcat::EL5042_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    el5042_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      el5042_states_msg_.names[index] = (*state_ptr)->name;
      el5042_states_msg_.states[index] = El5042StateToMsg(*state_ptr);
      index++;
    }
    el5042_pub_->publish(el5042_states_msg_);
  }
}

void Fcat::PublishIld1900States() {
  auto state_vec = device_type_vec_map_[fastcat::ILD1900_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    ild1900_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      ild1900_states_msg_.names[index] = (*state_ptr)->name;
      ild1900_states_msg_.states[index] = Ild1900StateToMsg(*state_ptr);
      index++;
    }
    ild1900_pub_->publish(ild1900_states_msg_);
  }
}

void Fcat::PublishCommanderStates() {
  auto state_vec = device_type_vec_map_[fastcat::COMMANDER_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    commander_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      commander_states_msg_.names[index] = (*state_ptr)->name;
      commander_states_msg_.states[index] = CommanderStateToMsg(*state_ptr);
      index++;
    }
    commander_pub_->publish(commander_states_msg_);
  }
}

void Fcat::PublishConditionalStates() {
  auto state_vec = device_type_vec_map_[fastcat::CONDITIONAL_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    conditional_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      conditional_states_msg_.names[index] = (*state_ptr)->name;
      conditional_states_msg_.states[index] = ConditionalStateToMsg(*state_ptr);
      index++;
    }
    conditional_pub_->publish(conditional_states_msg_);
  }
}

void Fcat::PublishFaulterStates() {
  auto state_vec = device_type_vec_map_[fastcat::FAULTER_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    faulter_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      faulter_states_msg_.names[index] = (*state_ptr)->name;
      faulter_states_msg_.states[index] = FaulterStateToMsg(*state_ptr);
      index++;
    }
    faulter_pub_->publish(faulter_states_msg_);
  }
}

void Fcat::PublishFilterStates() {
  auto state_vec = device_type_vec_map_[fastcat::FILTER_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    filter_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      filter_states_msg_.names[index] = (*state_ptr)->name;
      filter_states_msg_.states[index] = FilterStateToMsg(*state_ptr);
      index++;
    }
    filter_pub_->publish(filter_states_msg_);
  }
}

void Fcat::PublishFunctionStates() {
  auto state_vec = device_type_vec_map_[fastcat::FUNCTION_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    function_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      function_states_msg_.names[index] = (*state_ptr)->name;
      function_states_msg_.states[index] = FunctionStateToMsg(*state_ptr);
      index++;
    }
    function_pub_->publish(function_states_msg_);
  }
}

void Fcat::PublishPidStates() {
  auto state_vec = device_type_vec_map_[fastcat::PID_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    pid_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      pid_states_msg_.names[index] = (*state_ptr)->name;
      pid_states_msg_.states[index] = PidStateToMsg(*state_ptr);
      index++;
    }
    pid_pub_->publish(pid_states_msg_);
  }
}

void Fcat::PublishSaturationStates() {
  auto state_vec = device_type_vec_map_[fastcat::SATURATION_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    saturation_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      saturation_states_msg_.names[index] = (*state_ptr)->name;
      saturation_states_msg_.states[index] = SaturationStateToMsg(*state_ptr);
      index++;
    }
    saturation_pub_->publish(saturation_states_msg_);
  }
}

void Fcat::PublishSchmittTriggerStates() {
  auto state_vec = device_type_vec_map_[fastcat::SCHMITT_TRIGGER_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    schmitt_trigger_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      schmitt_trigger_states_msg_.names[index] = (*state_ptr)->name;
      schmitt_trigger_states_msg_.states[index] = SchmittTriggerStateToMsg(*state_ptr);
      index++;
    }
    schmitt_trigger_pub_->publish(schmitt_trigger_states_msg_);
  }
}

void Fcat::PublishSignalGeneratorStates() {
  auto state_vec = device_type_vec_map_[fastcat::SIGNAL_GENERATOR_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    signal_generator_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      signal_generator_states_msg_.names[index] = (*state_ptr)->name;
      signal_generator_states_msg_.states[index] = SignalGeneratorStateToMsg(*state_ptr);
      index++;
    }
    signal_generator_pub_->publish(signal_generator_states_msg_);
  }
}

void Fcat::PublishLinearInterpolationStates() {
  auto state_vec = device_type_vec_map_[fastcat::LINEAR_INTERPOLATION_STATE];
  if (state_vec.size() > 0) {
    size_t index = 0;

    linear_interpolation_states_msg_.header.stamp = publish_time_stamp_;
    for (auto state_ptr = state_vec.begin(); state_ptr != state_vec.end(); state_ptr++) {
      linear_interpolation_states_msg_.names[index] = (*state_ptr)->name;
      linear_interpolation_states_msg_.states[index] = LinearInterpolationStateToMsg(*state_ptr);
      index++;
    }
    linear_interpolation_pub_->publish(linear_interpolation_states_msg_);
  }
}
