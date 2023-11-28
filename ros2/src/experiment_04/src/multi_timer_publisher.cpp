#include <chrono> // time literals
#include <vector>

#include "experiment_02/simple_node.hpp"
#include "basic_bench_msgs/msg/bench_minimal.hpp"

using namespace std::chrono_literals;
namespace messages = basic_bench_msgs::msg;

class SimplePublisher : public ParameterizedNode {

public:

  SimplePublisher()
  : ParameterizedNode("simple_publisher") {
    publisher_ = this->create_publisher<messages::BenchMinimal>("simple_publishing", 10);

// todo this could be an int64[] parameter
    std::vector<int> multipliers = {1, 3, 9};
    for (const auto& m : multipliers) {
      // assign a custom name to the timer for better clarity on which timer is which to influxdb/grafana.
      const auto * rcl_node_handle = rclcpp::node_interfaces::get_node_base_interface(*this)->get_rcl_node_handle();
      auto options = rclcpp::TimerOptions(rcl_node_handle, std::string("::x") + std::to_string(m));
      auto timer = this->create_wall_timer(
        std::chrono::milliseconds(m * PUBLISHER_COOLDOWN_PARAM),
        std::bind(&SimplePublisher::timer_callback, this),
        options
      );

      timers_.emplace_back(timer);
    }

    RCLCPP_INFO(this->get_logger(), "Multi-Timer (%llu) Publishing node is ready.", timers_.size());
  }

private:

  void timer_callback() {
    auto message = messages::BenchMinimal();
    message.junk.resize(MSG_SIZE_PARAM);

    BUSY_WAIT_MS(HOLD_CALLBACK_PARAM)

    publisher_->publish(message);
  }

  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<messages::BenchMinimal>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}