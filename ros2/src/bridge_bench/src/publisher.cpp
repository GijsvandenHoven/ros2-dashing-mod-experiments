#include "rclcpp/rclcpp.hpp"
#include "basic_bench/msg/bench.hpp"

using namespace std::chrono_literals;



class BenchPublisher : public rclcpp::Node {
public:
  BenchPublisher()
  : Node("bridge_bench_publisher"), id_(0), rate_ms(10ms) {
    publisher_ = this->create_publisher<basic_bench::msg::Bench>("bridge_bench", 1000);
    timer_ = this->create_wall_timer(
      this->rate_ms, 
      std::bind(&BenchPublisher::timer_callback, this)
    );
    RCLCPP_INFO(this->get_logger(), "publisher ready");
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "publisher call back with id [%d]", id_);
    auto message = basic_bench::msg::Bench();

    message.id = id_;
    message.junk.resize(10);
    publisher_->publish(message);

    //<Notice me>> incrementing ID here.
    ++id_;
  }

  /** member declarations */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<basic_bench::msg::Bench>::SharedPtr publisher_;
  size_t id_;
  std::chrono::milliseconds rate_ms;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchPublisher>());
  rclcpp::shutdown();
  return 0;
}