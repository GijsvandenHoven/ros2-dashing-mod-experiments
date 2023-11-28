#include <functional>
#include <chrono>

#include "experiment_02/simple_node.hpp"
#include "basic_bench_msgs/msg/bench_minimal.hpp"

using std::placeholders::_1;
namespace messages = basic_bench_msgs::msg;

class SimpleSubscriber : public ParameterizedNode
{
public:
    SimpleSubscriber()
    : ParameterizedNode("simple_subscriber") {
        subscription_ = this->create_subscription<messages::BenchMinimal>(
            "simple_publishing", 
            100,
            std::bind(&SimpleSubscriber::topic_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribing node is ready.");
    }

private:

    void topic_callback(const messages::BenchMinimal::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "final recv from %u id %llu", msg->vandenhoven_publisher_hash, msg->vandenhoven_identifier);
        BUSY_WAIT_MS(HOLD_CALLBACK_PARAM)
        // RCLCPP_INFO(this->get_logger(), "yield");
    }

    rclcpp::Subscription<messages::BenchMinimal>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}