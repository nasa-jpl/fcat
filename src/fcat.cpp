#include "fcat/fcat.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

Fcat::~Fcat(){
  fcat_manager_.Shutdown();
}

Fcat::Fcat() : Node("fcat", "fcat") {

  std::string fastcat_config_path;
  this->declare_parameter<std::string>("fastcat_config_path", "");
  this->get_parameter("fastcat_config_path", fastcat_config_path);

  MSG("loading Yaml from %s", fastcat_config_path.c_str());
  YAML::Node node = YAML::LoadFile(fastcat_config_path);

  if (!fcat_manager_.ConfigFromYaml(node)) {
    throw std::invalid_argument(
        "Fastcat Manager failed to process bus configuration YAML file.");
  }
  MSG("Created Fcat Manager!");

  this->declare_parameter<bool>("create_joint_state_pub", true);
  this->get_parameter("create_joint_state_pub", enable_js_pub_);

  PopulateDeviceStateFields();

  // Must populate certain device state fields before initializing pub/sub
  InitializePublishersAndMessages();
  InitializeSubscribers();

  loop_period_sec_ = 1.0e3 / fcat_manager_.GetTargetLoopRate();
  chrono_loop_duration_ = std::chrono::duration<double,std::milli>(loop_period_sec_);
  timer_ = this->create_wall_timer( chrono_loop_duration_, 
      std::bind(&Fcat::Process, this));
  MSG("Starting Wall Timer with Period of %lf msec", loop_period_sec_);

  last_time_ = fcat_get_time_sec();
}

void Fcat::PopulateDeviceStateFields(){

  device_state_ptrs_ = fcat_manager_.GetDeviceStatePointers();

  for(auto it = device_state_ptrs_.begin(); it != device_state_ptrs_.end(); ++it){
    MSG("Populating device_name_state_map_[%s]", (*it)->name.c_str());

    device_name_state_map_[(*it)->name] = *it;

    device_type_vec_map_[(*it)->type].push_back(*it);

    MSG("counts[%d] = %ld", (*it)->type, device_type_vec_map_[(*it)->type].size());
  }
}

bool Fcat::DeviceExistsOnBus(std::string name, fastcat::DeviceStateType type)
{
  if(!(device_name_state_map_.end() != device_name_state_map_.find(name))){
    ERROR("Device %s does not exist on bus", name.c_str());
    return false;
  }
  if(device_name_state_map_[name]->type != type){
    ERROR("Device type does not match for %s", name.c_str());
    return false;
  }
  return true;
}

bool Fcat::TypeExistsOnBus(fastcat::DeviceStateType type){
  if(device_type_vec_map_[type].size() > 0){
    return true;
  }
  return false;
}

void Fcat::InitializePublishersAndMessages(){
  MSG("Fcat::InitializePublishers()");

  // Actuator
  auto vec_state_ptrs = device_type_vec_map_[fastcat::ACTUATOR_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating actuator pub");
    actuator_pub_ = this->create_publisher<fcat_msgs::msg::ActuatorStates>(
        "state/actuators", FCAT_PUB_QUEUE_SIZE);

    actuator_states_msg_.names.resize(vec_state_ptrs.size());
    actuator_states_msg_.states.resize(vec_state_ptrs.size());

    if(enable_js_pub_){
      MSG("Creating joint_states pub");

      // This is the only topic that broadcasts in the global 
      // namespace. This prevents needing to remap `fcat/state/joint_states` 
      // to `/joint_states` in launch files or through command line args.
      // It can always be reverted if this causes any issues down the road.
      joint_state_pub_ = 
        this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", FCAT_PUB_QUEUE_SIZE);
    }
  }

  // Egd
  vec_state_ptrs = device_type_vec_map_[fastcat::EGD_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Egd pub");
    egd_pub_ = this->create_publisher<fcat_msgs::msg::EgdStates>(
        "state/egds", FCAT_PUB_QUEUE_SIZE);

    egd_states_msg_.names.resize(vec_state_ptrs.size());
    egd_states_msg_.states.resize(vec_state_ptrs.size());
  }


  // El2124
  vec_state_ptrs = device_type_vec_map_[fastcat::EL2124_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating El2124 pub");
    el2124_pub_ = this->create_publisher<fcat_msgs::msg::El2124States>(
        "state/el2124s", FCAT_PUB_QUEUE_SIZE);

    el2124_states_msg_.names.resize(vec_state_ptrs.size());
    el2124_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3208
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3208_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating El3208 pub");
    el3208_pub_ = this->create_publisher<fcat_msgs::msg::El3208States>(
        "state/el3208s", FCAT_PUB_QUEUE_SIZE);

    el3208_states_msg_.names.resize(vec_state_ptrs.size());
    el3208_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // El3602
  vec_state_ptrs = device_type_vec_map_[fastcat::EL3602_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating El3602 pub");
    el3602_pub_ = this->create_publisher<fcat_msgs::msg::El3602States>(
        "state/el3602s", FCAT_PUB_QUEUE_SIZE);

    el3602_states_msg_.names.resize(vec_state_ptrs.size());
    el3602_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Jed
  vec_state_ptrs = device_type_vec_map_[fastcat::JED_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Jed pub");
    jed_pub_ = this->create_publisher<fcat_msgs::msg::JedStates>(
        "state/jeds", FCAT_PUB_QUEUE_SIZE);

    jed_states_msg_.names.resize(vec_state_ptrs.size());
    jed_states_msg_.states.resize(vec_state_ptrs.size());
  }


  // Commander
  vec_state_ptrs = device_type_vec_map_[fastcat::COMMANDER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Commander pub");
    commander_pub_ = this->create_publisher<fcat_msgs::msg::CommanderStates>(
        "state/commanders", FCAT_PUB_QUEUE_SIZE);

    commander_states_msg_.names.resize(vec_state_ptrs.size());
    commander_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Conditional
  vec_state_ptrs = device_type_vec_map_[fastcat::CONDITIONAL_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Conditional pub");
    conditional_pub_ = this->create_publisher<fcat_msgs::msg::ConditionalStates>(
        "state/conditionals", FCAT_PUB_QUEUE_SIZE);

    conditional_states_msg_.names.resize(vec_state_ptrs.size());
    conditional_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Faulter
  vec_state_ptrs = device_type_vec_map_[fastcat::FAULTER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Faulter pub");
    faulter_pub_ = this->create_publisher<fcat_msgs::msg::FaulterStates>(
        "state/faulters", FCAT_PUB_QUEUE_SIZE);

    faulter_states_msg_.names.resize(vec_state_ptrs.size());
    faulter_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Filter
  vec_state_ptrs = device_type_vec_map_[fastcat::FILTER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Filter pub");
    filter_pub_ = this->create_publisher<fcat_msgs::msg::FilterStates>(
        "state/filter", FCAT_PUB_QUEUE_SIZE);

    filter_states_msg_.names.resize(vec_state_ptrs.size());
    filter_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Function
  vec_state_ptrs = device_type_vec_map_[fastcat::FUNCTION_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Function pub");
    function_pub_ = this->create_publisher<fcat_msgs::msg::FunctionStates>(
        "state/functions", FCAT_PUB_QUEUE_SIZE);

    function_states_msg_.names.resize(vec_state_ptrs.size());
    function_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Pid
  vec_state_ptrs = device_type_vec_map_[fastcat::PID_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Pid pub");
    pid_pub_ = this->create_publisher<fcat_msgs::msg::PidStates>(
        "state/pids", FCAT_PUB_QUEUE_SIZE);

    pid_states_msg_.names.resize(vec_state_ptrs.size());
    pid_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Saturation
  vec_state_ptrs = device_type_vec_map_[fastcat::SATURATION_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Saturation pub");
    saturation_pub_ = this->create_publisher<fcat_msgs::msg::SaturationStates>(
        "state/saturations", FCAT_PUB_QUEUE_SIZE);

    saturation_states_msg_.names.resize(vec_state_ptrs.size());
    saturation_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Schmitt Trigger
  vec_state_ptrs = device_type_vec_map_[fastcat::SCHMITT_TRIGGER_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Schmitt Trigger pub");
    schmitt_trigger_pub_ = this->create_publisher<fcat_msgs::msg::SchmittTriggerStates>(
        "state/schmitt_triggers", FCAT_PUB_QUEUE_SIZE);

    schmitt_trigger_states_msg_.names.resize(vec_state_ptrs.size());
    schmitt_trigger_states_msg_.states.resize(vec_state_ptrs.size());
  }

  // Signal Generator
  vec_state_ptrs = device_type_vec_map_[fastcat::SIGNAL_GENERATOR_STATE];
  if(vec_state_ptrs.size() > 0){

    MSG("Creating Signal Generator pub");
    signal_generator_pub_ = this->create_publisher<fcat_msgs::msg::SignalGeneratorStates>(
        "state/signal_generators", FCAT_PUB_QUEUE_SIZE);

    signal_generator_states_msg_.names.resize(vec_state_ptrs.size());

    signal_generator_states_msg_.states.resize(vec_state_ptrs.size());
  }

  
  // FTS, AtiFts, VirtualFts
  vec_state_ptrs = device_type_vec_map_[fastcat::FTS_STATE];
  for(auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++){

     std::string device = (*dev_state_ptr)->name;
     fts_raw_pub_map_[device] = 
       this->create_publisher<geometry_msgs::msg::Wrench>(
         "fts/" + device + "/raw_wrench", FCAT_PUB_QUEUE_SIZE);

     fts_tared_pub_map_[device] = 
       this->create_publisher<geometry_msgs::msg::Wrench>(
         "fts/" + device + "/tared_wrench", FCAT_PUB_QUEUE_SIZE);
   }

  module_state_pub_ = this->create_publisher<fcat_msgs::msg::ModuleState>(
      "state/module_state", FCAT_PUB_QUEUE_SIZE);
}


void Fcat::InitializeSubscribers(){

  // bus reset/fault
  reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "cmd/reset",
      FCAT_SUB_QUEUE_SIZE,
      std::bind(&Fcat::ResetCb, this, _1));

  fault_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "cmd/fault",
      FCAT_SUB_QUEUE_SIZE,
      std::bind(&Fcat::FaultCb, this, _1));


  // Actuator
  if(TypeExistsOnBus(fastcat::ACTUATOR_STATE)){

    actuator_csp_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCspCmd>(
        "cmd/actuator_csp", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorCSPCmdCb, this, _1));

    actuator_csv_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCsvCmd>(
        "cmd/actuator_csv", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorCSVCmdCb, this, _1));

    actuator_cst_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCstCmd>(
        "cmd/actuator_cst", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorCSTCmdCb, this, _1));

    actuator_calibrate_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorCalibrateCmd>(
        "cmd/actuator_calibrate", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorCalibrateCmdCb, this, _1));

    actuator_prof_pos_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorProfPosCmd>(
        "cmd/actuator_prof_pos", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorProfPosCmdCb, this, _1));

    actuator_prof_torque_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorProfTorqueCmd>(
        "cmd/actuator_prof_torque", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorProfTorqueCmdCb, this, _1));

    actuator_prof_vel_sub_ = this->create_subscription<fcat_msgs::msg::ActuatorProfVelCmd>(
        "cmd/actuator_prof_vel", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::ActuatorProfVelCmdCb, this, _1));

    actuator_set_output_position_sub_ = 
      this->create_subscription<fcat_msgs::msg::ActuatorSetOutputPositionCmd>(
          "cmd/actuator_set_output_position", 
          FCAT_SUB_QUEUE_SIZE, 
          std::bind(&Fcat::ActuatorSetOutputPositionCmdCb, this, _1));
  }

  // Commander
  if(TypeExistsOnBus(fastcat::COMMANDER_STATE)){
    commander_enable_sub_ = 
      this->create_subscription<fcat_msgs::msg::CommanderEnableCmd>(
          "cmd/commander_enable", 
          FCAT_SUB_QUEUE_SIZE, 
          std::bind(&Fcat::CommanderEnableCmdCb, this, _1));

    commander_disable_sub_ = 
      this->create_subscription<fcat_msgs::msg::CommanderDisableCmd>(
          "cmd/commander_disable", 
          FCAT_SUB_QUEUE_SIZE, 
          std::bind(&Fcat::CommanderDisableCmdCb, this, _1));
  }

  // El2124
  if(TypeExistsOnBus(fastcat::EL2124_STATE)){
    el2124_write_all_channels_sub_ = 
      this->create_subscription<fcat_msgs::msg::El2124WriteAllChannelsCmd>(
          "cmd/el2124_write_all_channels", 
          FCAT_SUB_QUEUE_SIZE, 
          std::bind(&Fcat::El2124WriteAllChannelsCmdCb, this, _1));

    el2124_write_channel_sub_ = 
      this->create_subscription<fcat_msgs::msg::El2124WriteChannelCmd>(
          "cmd/el2124_write_channel", 
          FCAT_SUB_QUEUE_SIZE, 
          std::bind(&Fcat::El2124WriteChannelCmdCb, this, _1));
  }

  // Faulter
  if(TypeExistsOnBus(fastcat::FAULTER_STATE)){
    faulter_enable_sub_ = this->create_subscription<fcat_msgs::msg::FaulterEnableCmd>(
        "cmd/faulter_enable", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::FaulterEnableCmdCb, this, _1));
  }

  // Fts
  if(TypeExistsOnBus(fastcat::FTS_STATE)){
    fts_tare_sub_= this->create_subscription<fcat_msgs::msg::FtsTareCmd>(
        "cmd/fts_tare", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::FtsTareCmdCb, this, _1));
  }

  // Jed
  if(TypeExistsOnBus(fastcat::JED_STATE)){
    jed_set_cmd_value_sub_= this->create_subscription<fcat_msgs::msg::JedSetCmdValueCmd>(
        "cmd/jed_set_cmd_value", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::JedSetCmdValueCmdCb, this, _1));
  }

  // Pid
  if(TypeExistsOnBus(fastcat::PID_STATE)){
    pid_activate_sub_= this->create_subscription<fcat_msgs::msg::PidActivateCmd>(
        "cmd/pid_activate", 
        FCAT_SUB_QUEUE_SIZE, 
        std::bind(&Fcat::PidActivateCmdCb, this, _1));
  }
}


void Fcat::Process(){

  fcat_manager_.Process();

  PublishModuleState();

  PublishFtsStates();

  PublishActuatorStates();
  PublishEgdStates();
  PublishEl2124States();
  PublishEl3208States();
  PublishEl3602States();
  PublishJedStates();

  PublishCommanderStates();
  PublishConditionalStates();
  PublishFaulterStates();
  PublishFilterStates();
  PublishFunctionStates();
  PublishPidStates();
  PublishSaturationStates();
  PublishSchmittTriggerStates();
  PublishSignalGeneratorStates();
}

void Fcat::PublishModuleState(){
  auto module_state_msg = fcat_msgs::msg::ModuleState();

  module_state_msg.faulted = fcat_manager_.IsFaulted();

  double now = fcat_get_time_sec();
  module_state_msg.jitter = now - last_time_;
  last_time_ = now;

  module_state_pub_->publish(module_state_msg);
}

void Fcat::PublishFtsStates(){

  // FTS, AtiFts, VirtualFts
  auto vec_state_ptrs = device_type_vec_map_[fastcat::FTS_STATE];

  for(auto dev_state_ptr = vec_state_ptrs.begin(); dev_state_ptr != vec_state_ptrs.end(); dev_state_ptr++){
  auto wrench_msg = geometry_msgs::msg::Wrench();

    std::string device = (*dev_state_ptr)->name;
    fastcat::FtsState fts_state = (*dev_state_ptr)->fts_state;

    wrench_msg.force.x = fts_state.raw_fx;
    wrench_msg.force.y = fts_state.raw_fy;
    wrench_msg.force.z = fts_state.raw_fz;

    wrench_msg.torque.x = fts_state.raw_tx;
    wrench_msg.torque.y = fts_state.raw_ty;
    wrench_msg.torque.z = fts_state.raw_tz;

    fts_raw_pub_map_[device]->publish(wrench_msg);


    wrench_msg.force.x = fts_state.tared_fx;
    wrench_msg.force.y = fts_state.tared_fy;
    wrench_msg.force.z = fts_state.tared_fz;

    wrench_msg.torque.x = fts_state.tared_tx;
    wrench_msg.torque.y = fts_state.tared_ty;
    wrench_msg.torque.z = fts_state.tared_tz;

    fts_tared_pub_map_[device]->publish(wrench_msg);
   }

}

void Fcat::PublishActuatorStates(){

  auto act_state_vec = device_type_vec_map_[fastcat::ACTUATOR_STATE];
  if(act_state_vec.size() > 0){
    size_t index = 0;
    auto js_msg = sensor_msgs::msg::JointState(); // TODO preallocate

    for(auto state_ptr = act_state_vec.begin(); 
        state_ptr != act_state_vec.end(); state_ptr++)
    {
      // Populate ActuatorStates message
      actuator_states_msg_.names[index] = (*state_ptr)->name;
      actuator_states_msg_.states[index] = ActuatorStateToMsg(*state_ptr);
      index++;

      // Populate JointState message
      js_msg.name.push_back((*state_ptr)->name);
      js_msg.position.push_back((*state_ptr)->actuator_state.actual_position);
      js_msg.velocity.push_back((*state_ptr)->actuator_state.actual_velocity);
    }

    actuator_pub_->publish(actuator_states_msg_);

    js_msg.header.stamp = now();
    if(enable_js_pub_){
      joint_state_pub_->publish(js_msg);
    }

  }
}

void Fcat::PublishEgdStates(){

  auto state_vec = device_type_vec_map_[fastcat::EGD_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      egd_states_msg_.names[index] = (*state_ptr)->name;
      egd_states_msg_.states[index] = EgdStateToMsg(*state_ptr);
      index++;
    }
    egd_pub_->publish(egd_states_msg_);
  }
}

void Fcat::PublishEl2124States(){

  auto state_vec = device_type_vec_map_[fastcat::EL2124_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      el2124_states_msg_.names[index] = (*state_ptr)->name;
      el2124_states_msg_.states[index] = El2124StateToMsg(*state_ptr);
      index++;
    }
    el2124_pub_->publish(el2124_states_msg_);
  }
}

void Fcat::PublishEl3208States(){

  auto state_vec = device_type_vec_map_[fastcat::EL3208_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      el3208_states_msg_.names[index] = (*state_ptr)->name;
      el3208_states_msg_.states[index] = El3208StateToMsg(*state_ptr);
      index++;
    }
    el3208_pub_->publish(el3208_states_msg_);
  }
}

void Fcat::PublishEl3602States(){

  auto state_vec = device_type_vec_map_[fastcat::EL3602_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      el3602_states_msg_.names[index] = (*state_ptr)->name;
      el3602_states_msg_.states[index] = El3602StateToMsg(*state_ptr);
      index++;
    }
    el3602_pub_->publish(el3602_states_msg_);
  }
}

void Fcat::PublishJedStates(){

  auto state_vec = device_type_vec_map_[fastcat::JED_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      jed_states_msg_.names[index] = (*state_ptr)->name;
      jed_states_msg_.states[index] = JedStateToMsg(*state_ptr);
      index++;
    }
    jed_pub_->publish(jed_states_msg_);
  }
}

void Fcat::PublishCommanderStates(){

  auto state_vec = device_type_vec_map_[fastcat::COMMANDER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      commander_states_msg_.names[index] = (*state_ptr)->name;
      commander_states_msg_.states[index] = CommanderStateToMsg(*state_ptr);
      index++;
    }
    commander_pub_->publish(commander_states_msg_);
  }
}

void Fcat::PublishConditionalStates(){

  auto state_vec = device_type_vec_map_[fastcat::CONDITIONAL_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      conditional_states_msg_.names[index] = (*state_ptr)->name;
      conditional_states_msg_.states[index] = ConditionalStateToMsg(*state_ptr);
      index++;
    }
    conditional_pub_->publish(conditional_states_msg_);
  }
}

void Fcat::PublishFaulterStates(){

  auto state_vec = device_type_vec_map_[fastcat::FAULTER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      faulter_states_msg_.names[index] = (*state_ptr)->name;
      faulter_states_msg_.states[index] = FaulterStateToMsg(*state_ptr);
      index++;
    }
    faulter_pub_->publish(faulter_states_msg_);
  }
}

void Fcat::PublishFilterStates(){

  auto state_vec = device_type_vec_map_[fastcat::FILTER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      filter_states_msg_.names[index] = (*state_ptr)->name;
      filter_states_msg_.states[index] = FilterStateToMsg(*state_ptr);
      index++;
    }
    filter_pub_->publish(filter_states_msg_);
  }
}

void Fcat::PublishFunctionStates(){

  auto state_vec = device_type_vec_map_[fastcat::FUNCTION_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      function_states_msg_.names[index] = (*state_ptr)->name;
      function_states_msg_.states[index] = FunctionStateToMsg(*state_ptr);
      index++;
    }
    function_pub_->publish(function_states_msg_);
  }
}

void Fcat::PublishPidStates(){

  auto state_vec = device_type_vec_map_[fastcat::PID_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      pid_states_msg_.names[index] = (*state_ptr)->name;
      pid_states_msg_.states[index] = PidStateToMsg(*state_ptr);
      index++;
    }
    pid_pub_->publish(pid_states_msg_);
  }
}

void Fcat::PublishSaturationStates(){

  auto state_vec = device_type_vec_map_[fastcat::SATURATION_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      saturation_states_msg_.names[index] = (*state_ptr)->name;
      saturation_states_msg_.states[index] = SaturationStateToMsg(*state_ptr);
      index++;
    }
    saturation_pub_->publish(saturation_states_msg_);
  }
}

void Fcat::PublishSchmittTriggerStates(){

  auto state_vec = device_type_vec_map_[fastcat::SCHMITT_TRIGGER_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      schmitt_trigger_states_msg_.names[index] = (*state_ptr)->name;
      schmitt_trigger_states_msg_.states[index] = SchmittTriggerStateToMsg(*state_ptr);
      index++;
    }
    schmitt_trigger_pub_->publish(schmitt_trigger_states_msg_);
  }
}

void Fcat::PublishSignalGeneratorStates(){

  auto state_vec = device_type_vec_map_[fastcat::SIGNAL_GENERATOR_STATE];
  if(state_vec.size() > 0){
    size_t index = 0;

    for(auto state_ptr = state_vec.begin(); 
        state_ptr != state_vec.end(); state_ptr++)
    {
      signal_generator_states_msg_.names[index] = (*state_ptr)->name;
      signal_generator_states_msg_.states[index] = SignalGeneratorStateToMsg(*state_ptr);
      index++;
    }
    signal_generator_pub_->publish(signal_generator_states_msg_);
  }
}
