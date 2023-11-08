#include <chrono> // used to measure elapsed time
#include <ctime> // formats time for the output data header
#include <memory> // std::fill
#include <fstream> // writing to files
#include <algorithm> // std::for_each

#include "rclcpp/rclcpp.hpp"
#include "basic_bench_msgs/msg/bench.hpp"
#include "config.h"

using namespace std::chrono_literals;

class BenchPublisher : public rclcpp::Node {
public:
  BenchPublisher()
  : Node("basic_bench_publisher"), id_(0), rate_ms(10ms), nanosecond_time_message_expected(0) {
    publisher_ = this->create_publisher<basic_bench_msgs::msg::Bench>("bench", BASIC_BENCH_DATA_BUF);
    timer_ = this->create_wall_timer(
      this->rate_ms, 
      std::bind(&BenchPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback() {
    auto message = basic_bench_msgs::msg::Bench();

    message.id = id_;
    message.junk.resize(vectorSizeSchedule[sizeScheduleIndex]);
    auto now = std::chrono::steady_clock::now();
    auto now_64b = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    message.stamp = now_64b;
    publisher_->publish(message);

    if (id_ == 0) {
      nanosecond_time_message_expected = now_64b;
    }
    nanosecond_time_message_expected += std::chrono::duration_cast<std::chrono::nanoseconds>(rate_ms).count();

    //<Notice me>> incrementing ID here.
    ++id_;

    if (id_ <= BASIC_BENCH_PUBLISHER_EXTRA) { // synchronisation with publisher: the first (X) do not contribute to the schedule.
      return;
    }

    int64_t nanosecond_time_message_jitter = static_cast<int64_t>(now_64b - nanosecond_time_message_expected);
    AJ_buf[AJ_buf_id] = nanosecond_time_message_jitter;
    AJ_buf_id++;

    // ROS_INFO("Expect: [%ld], Got: [%ld]. Diff: [%ld]", nanosecond_time_message_expected, now_64b, nanosecond_time_message_jitter);

    if ((id_ - BASIC_BENCH_PUBLISHER_EXTRA) % (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE) == 0) {
      writeJitterBuffer(AJ_buf, AJ_buf_size, vectorSizeSchedule[sizeScheduleIndex] + MESSAGE_BYTE_OFFSET);
      AJ_buf_id = 0;

      sizeScheduleIndex++;
      int scheduledSize = vectorSizeSchedule[sizeScheduleIndex];
      RCLCPP_INFO(this->get_logger(), "ID [%d], switch to size [%d]", id_, scheduledSize);
      if (scheduledSize == -1) {
        RCLCPP_INFO(this->get_logger(), "Published all messages for all scheduled sizes.");
        rclcpp::shutdown();
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
    std::time_t today = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    file << buf_size << " measurements, approximate date: " << std::ctime(&today); // beware: ctime inserts a newline, try keeping this last.

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
  rclcpp::Publisher<basic_bench_msgs::msg::Bench>::SharedPtr publisher_;
  size_t id_;

  std::chrono::milliseconds rate_ms; // this is needed to remember the rate outside of the node constructor

  static const int MESSAGE_BYTE_OFFSET = 16; // timestamp is 8 byte, id is 4 byte, serialized vector is preceded by a 4 byte length.
  int sizeTarget(int target) { return target - MESSAGE_BYTE_OFFSET; };

  int sizeScheduleIndex = 0;
  int vectorSizeSchedule[8] = {sizeTarget(16), sizeTarget(32), sizeTarget(64), sizeTarget(256), sizeTarget(1024), sizeTarget(4096), sizeTarget(65536), -1};

  static constexpr int AJ_buf_size = (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE);
  int64_t AJ_buf[AJ_buf_size];
  int AJ_buf_id = 0;
  
  uint64_t nanosecond_time_message_expected;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
