#include <chrono>
#include <memory>
#include <fstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "basic_bench/msg/bench.hpp"
#include "config.h"

using namespace std::chrono_literals;



class BenchPublisher : public rclcpp::Node {
public:
  BenchPublisher()
  : Node("basic_bench_publisher"), id_(0), rate_ms(10ms) {
    publisher_ = this->create_publisher<basic_bench::msg::Bench>("bench", BASIC_BENCH_DATA_BUF);
    timer_ = this->create_wall_timer(
      this->rate_ms, 
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

    if (prev_stamp != no_stamp) {
      int64_t observed_time_ns = static_cast<int64_t>(now_64b - prev_stamp);
      int64_t expected_time_ns = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(rate_ms).count());
      AJ_buf[AJ_buf_id] = std::abs(observed_time_ns - expected_time_ns);
      AJ_buf_id++;
    }
    prev_stamp = now_64b;

    if ((id_ - BASIC_BENCH_PUBLISHER_EXTRA) % (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE) == 0) {
      writeJitterBuffer(AJ_buf, AJ_buf_size, vectorSizeSchedule[sizeScheduleIndex] + MESSAGE_BYTE_OFFSET);
      AJ_buf_id = 0;

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

  /** helper functions */
  template <typename T>
  void writeJitterBuffer(T buf, uint32_t buf_size, int message_size) {
    static_assert(std::is_pointer<T>::value, "buffer must be a pointer");

    std::string suffix = std::string("jitter") + std::to_string(message_size);
    std::string path = std::string(BASIC_BENCH_JITTER_FOLDER) + suffix;
    
    std::ofstream file(path);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "could not open file");
      exit(-1);
    }
    // write a simple header for the file
    file << buf_size << " measurements\n";

    std::for_each(buf, buf + buf_size, [&file](auto measurement){
      file << measurement << '\n';
    });
  }

  // duplicated from subscriber.cpp
  template <typename T>
  void zeroBuffer(T buf, uint32_t siz) {
    static_assert(std::is_pointer<T>::value, "buffer must be a pointer");
    std::fill(buf, buf+siz, 0);
  }

  /** member declarations */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<basic_bench::msg::Bench>::SharedPtr publisher_;
  size_t id_;

  std::chrono::milliseconds rate_ms; // this is needed to remember the rate outside of the node constructor

  static const int MESSAGE_BYTE_OFFSET = 16; // timestamp is 8 byte, id is 4 byte, serialized vector is preceded by a 4 byte length.
  int sizeTarget(int target) { return target - MESSAGE_BYTE_OFFSET; };

  int sizeScheduleIndex = 0;
  int vectorSizeSchedule[8] = {sizeTarget(16), sizeTarget(32), sizeTarget(64), sizeTarget(256), sizeTarget(1024), sizeTarget(4096), sizeTarget(65536), -1};

  static constexpr int AJ_buf_size = (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE) - 1;
  int64_t AJ_buf[AJ_buf_size];
  int AJ_buf_id = 0;
  uint64_t no_stamp = ~(0U);
  uint64_t prev_stamp = no_stamp;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchPublisher>());
  rclcpp::shutdown();
  return 0;
}