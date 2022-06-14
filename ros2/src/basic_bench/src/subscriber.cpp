#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "basic_bench/msg/bench.hpp"

#include "config.h"

using std::placeholders::_1;  // it's a bit spicy of the ROS tutorial to not even mention this. it's an indicator for the # of parameters the callback takes. https://en.cppreference.com/w/cpp/utility/functional/placeholders

class BenchSubscriber : public rclcpp::Node
{
public:
  BenchSubscriber()
  : Node("basic_bench_subscriber")
  {
    subscription_ = this->create_subscription<basic_bench::msg::Bench>(
      "bench", BASIC_BENCH_DATA_BUF, 
      std::bind(&BenchSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const basic_bench::msg::Bench::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->id);
  }
  rclcpp::Subscription<basic_bench::msg::Bench>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchSubscriber>());
  rclcpp::shutdown();
  return 0;
}
