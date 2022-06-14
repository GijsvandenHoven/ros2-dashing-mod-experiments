#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "basic_bench/msg/bench.hpp"
#include "config.h"

using namespace std::chrono_literals;

class BenchPublisher : public rclcpp::Node {
public:
  BenchPublisher()
  : Node("basic_bench_publisher"), id_(0) {
    publisher_ = this->create_publisher<basic_bench::msg::Bench>("bench", BASIC_BENCH_DATA_BUF);
    timer_ = this->create_wall_timer(
      10ms, 
      std::bind(&BenchPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback() {
    auto message = basic_bench::msg::Bench();
    message.id = id_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.id);
    publisher_->publish(message);
  }

  /** member declarations */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<basic_bench::msg::Bench>::SharedPtr publisher_;
  size_t id_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
