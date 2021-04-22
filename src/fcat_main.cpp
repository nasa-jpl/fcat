#include "fcat/fcat.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fcat>());
  rclcpp::shutdown();
  return 0;
}
