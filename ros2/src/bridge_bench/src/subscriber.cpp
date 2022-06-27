#include "rclcpp/rclcpp.hpp"
#include "basic_bench/msg/bench.hpp"

using std::placeholders::_1;  // it's a bit spicy of the ROS tutorial to not even mention this. it's an indicator for the # of parameters the callback takes. https://en.cppreference.com/w/cpp/utility/functional/placeholders

class BenchSubscriber : public rclcpp::Node
{
public:
  BenchSubscriber()
  : Node("bridge_bench_subscriber")
  {
    subscription_ = this->create_subscription<basic_bench::msg::Bench>(
      "bridge_bench", 
      1000, 
      std::bind(&BenchSubscriber::topic_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "subscriber ready");
  }

private:
  void topic_callback(const basic_bench::msg::Bench::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "got %d", msg->id);
  }

  /** member declarations */
  rclcpp::Subscription<basic_bench::msg::Bench>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchSubscriber>());
  rclcpp::shutdown();
  return 0;
}
