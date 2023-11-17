#include <chrono> // time literals

#include "experiment_02/simple_node.hpp"
#include "basic_bench_msgs/msg/bench_minimal.hpp"

using namespace std::chrono_literals;
namespace messages = basic_bench_msgs::msg;

class SimplePublisher : public ParameterizedNode {

public:

  SimplePublisher()
  : ParameterizedNode("simple_publisher") {
    publisher_ = this->create_publisher<messages::BenchMinimal>("simple_publishing", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(PUBLISHER_COOLDOWN_PARAM),
      std::bind(&SimplePublisher::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Publishing node is ready.");
  }

private:

  void timer_callback() {
    auto message = messages::BenchMinimal();
    message.junk.resize(MSG_SIZE_PARAM);

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<messages::BenchMinimal>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}