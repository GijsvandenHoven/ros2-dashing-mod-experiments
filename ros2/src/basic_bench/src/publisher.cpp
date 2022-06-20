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

    message.id = id_;
    message.junk.resize(vectorSizeSchedule[sizeScheduleIndex]);
    auto now = std::chrono::steady_clock::now(); // chrono might be overengineered. This library is insane.
    auto now_64b = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    message.stamp = now_64b;
    publisher_->publish(message);

    ++id_;

    if (id_ <= BASIC_BENCH_PUBLISHER_EXTRA) { // synchronisation with publisher: the first (X) do not contribute to the schedule.
      return;
    }

    if ((id_ - BASIC_BENCH_PUBLISHER_EXTRA) % (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE) == 0) {
      sizeScheduleIndex++;
      int scheduledSize = vectorSizeSchedule[sizeScheduleIndex];
      RCLCPP_INFO(this->get_logger(), "ID [%d], switch to size [%d]", id_, scheduledSize);
      if (scheduledSize == -1) {
        RCLCPP_INFO(this->get_logger(), "Published all messages for all scheduled sizes.");
        
#ifdef USE_SLEEP
        std::chrono::nanoseconds sleepfor = 2s; // https://en.cppreference.com/w/cpp/chrono/operator%22%22ns
        // sometimes, the subscriber does not receive the last message on ROS2. suspected race condition with shutdown and sending?
        auto context = rclcpp::Context::get_rcl_context();        // error: this is a member function.
        context->sleep_for(sleepfor);
#else
        for (unsigned int i = 0; i < 4000000000; ++i) {
          if (i % 1000000000 == 0) RCLCPP_INFO(this->get_logger(), "%u", i);
        }
#endif
        RCLCPP_INFO(this->get_logger(), "Shutting down");
        exit(0);
      }
    }
  }

  /** member declarations */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<basic_bench::msg::Bench>::SharedPtr publisher_;
  size_t id_;

  static const int MESSAGE_BYTE_OFFSET = 16; // timestamp is 8 byte, id is 4 byte, serialized vector is preceded by a 4 byte length.
  int sizeTarget(int target) { return target - MESSAGE_BYTE_OFFSET; };

  int sizeScheduleIndex = 0;
  int vectorSizeSchedule[8] = {sizeTarget(16), sizeTarget(32), sizeTarget(64), sizeTarget(256), sizeTarget(1024), sizeTarget(4096), sizeTarget(65536), -1};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
