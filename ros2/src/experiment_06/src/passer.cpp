#include <chrono> // time literals
#include <functional>

#include "experiment_02/simple_node.hpp"
#include "basic_bench_msgs/msg/bench_minimal.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
namespace messages = basic_bench_msgs::msg;

//todo: create alternate publisher tracker in client library, to harden against interference.
// introduce interfering node to test this (optional, reduce chain to A->C to stay in 4 core budget. Interfere on B.)
// write influxdb query to extract the latency of A to D.
   // This should be some type of inner join on msg_id, discarding all but the most recent pairing based on _start (?)
class SimplePasser : public ParameterizedNode {

public:

  SimplePasser()
  : ParameterizedNode("simple_passer") {
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.message_tracker_opts.tracker_option = rclcpp::MessageTrackerEnum::TRACING_PUBLISHER;
    publisher_ = this->create_publisher<messages::BenchMinimal>("simple_passing_pub", 10, publisher_options); // pretty much expecting to be remapped.

    subscription_ = this->create_subscription<messages::BenchMinimal>(
          "simple_passing_sub", 
          100,
          std::bind(&SimplePasser::topic_callback, this, _1)
      );

    RCLCPP_INFO(this->get_logger(), "Passing node is ready.");
  }

private:

  void topic_callback(const messages::BenchMinimal::SharedPtr __msg) {
      RCLCPP_INFO(this->get_logger(), "recv from %u id %llu", __msg->vandenhoven_publisher_hash, __msg->vandenhoven_identifier);
      BUSY_WAIT_MS(HOLD_CALLBACK_PARAM)
      
      // intentionally create a new message, there is no guarantee a node would re-use a message, usually the opposite. Thus ID is not preserved unless done explicitly. This must be tested
      auto message = messages::BenchMinimal();
      message.junk.resize(MSG_SIZE_PARAM);
      message.vandenhoven_identifier = __msg->vandenhoven_identifier;
      publisher_->publish(message);
      //RCLCPP_INFO(this->get_logger(), "yield");
  }

  rclcpp::Subscription<messages::BenchMinimal>::SharedPtr subscription_;
  rclcpp::Publisher<messages::BenchMinimal>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePasser>());
  rclcpp::shutdown();
  return 0;
}