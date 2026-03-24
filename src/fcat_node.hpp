// Copyright 2021 California Institute of Technology

#ifndef FCAT_NODE_HPP_
#define FCAT_NODE_HPP_

#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

class FcatNode : public rclcpp::Node {
 public:
  FcatNode(const std::string& node_name, const std::string& namespace_,
           const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node(node_name, namespace_, options) {}

  // Must be called before InitializeTimer() to take effect
  void InitializeTimerRate(double hz) { target_loop_rate_hz_ = hz; }
  double GetTimerRate() const { return target_loop_rate_hz_; }
  void SetTimerCallbackGroup(rclcpp::CallbackGroup::SharedPtr callback_group) {
    timer_callback_group_ = callback_group;
  }

  void SetActive() {}

  virtual void StartProcessTimer() {
    double period_usec = 1.0e6 / target_loop_rate_hz_;
    std::chrono::duration<double, std::micro> chrono_dur(period_usec);
    timer_ = this->create_wall_timer(chrono_dur, std::bind(&FcatNode::Process, this),
                                     timer_callback_group_);
  }

  virtual void Process() = 0;

 protected:
  double target_loop_rate_hz_ = 250.0;
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_ = nullptr;
};

#endif  // FCAT_NODE_HPP_
