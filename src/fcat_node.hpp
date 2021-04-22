#ifndef FCAT_NODE_HPP_
#define FCAT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

class FcatNode : public rclcpp::Node
{
  public:
    FcatNode(const std::string& node_name, const std::string& namespace_,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node(node_name, namespace_, options)
    {
      InitializeDefaultParameters();
    }

    void InitializeDefaultParameters()
    {
      // set up default parameters

    }

    // Must be called before InitializeTimer() to take effect
    void SetTimerRate(double hz){ target_loop_rate_hz_ = hz; }

    virtual void InitializeTimer()
    {
      double period_usec = 1.0e6 / target_loop_rate_hz_;
      std::chrono::duration<double, std::micro> chrono_dur(period_usec);
      timer_ = this->create_wall_timer(chrono_dur, 
          std::bind(&FcatNode::Process, this));
    }

    virtual void Process() = 0;

  protected:

    double target_loop_rate_hz_ = 250.0;
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

};

#endif
