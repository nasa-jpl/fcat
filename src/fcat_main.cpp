#include <sched.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "fcat/fcat.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

#ifdef ROS2_FOXY
  // support for adding specific callback groups to an executor was only added
  // after ROS2 foxy, so just use a standard MultiThreadedExecutor for ROS2 foxy
  // only
  //
  // A MultiThreadedExecutor consists of multiple SingleThreadedExecutors
  // running in different threads. SingleThreadedExecutors will check for new
  // subscriptions periodically at run-time, which may intermittently delay the
  // main process loop; therefore it is recommended to use ROS2 humble or later
  // when running at higher loop rates
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),  // Default options
    2                           // Number of threads
  );
  auto fcat_node = std::make_shared<Fcat>();
  executor.add_node(fcat_node);
  executor.spin();
  rclcpp::shutdown();
#else
  // Use a StaticSingleThreadedExecutor for main Process loop to avoid overhead
  // of checking for new subscriptions at run-time. ROS2 does not allow the
  // option of using a StaticSingleThreadedExecutor within a
  // MultiThreadedExecutor, so it is necessary to take more granular control of
  // spawning individual executors for different callback groups
  rclcpp::executors::StaticSingleThreadedExecutor process_loop_executor;
  rclcpp::executors::SingleThreadedExecutor topic_executor;

  auto fcat_node = std::make_shared<Fcat>();

  process_loop_executor.add_callback_group(
    fcat_node->get_process_loop_callback_group(),
    fcat_node->get_node_base_interface());

  topic_executor.add_callback_group(
    fcat_node->get_node_base_interface()->get_default_callback_group(),
    fcat_node->get_node_base_interface());
  topic_executor.add_callback_group(fcat_node->get_topic_callback_group(),
                                    fcat_node->get_node_base_interface());

  auto process_loop_thread =
    std::thread([&]() { process_loop_executor.spin(); });
  auto topic_thread = std::thread([&]() { topic_executor.spin(); });

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::shutdown();
  process_loop_thread.join();
  topic_thread.join();
#endif

  return 0;
}
