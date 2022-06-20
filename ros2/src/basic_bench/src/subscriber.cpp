#include <memory>
#include <algorithm>
#include <fstream>

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
  void topic_callback(const basic_bench::msg::Bench::SharedPtr msg) {
    int32_t id = msg->id;
    int64_t send_stamp = msg->stamp;
    size_t vector_size = msg->junk.size();
    auto now = std::chrono::steady_clock::now();
    auto now_64b = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    int64_t measurement = now_64b - send_stamp;

    if (prev_msg_id != -1 && prev_msg_id + 1 != id) {
      RCLCPP_ERROR(this->get_logger(), "Cannot keep up with the Publisher. Expected id [%d], got [%d]", prev_msg_id + 1, id);
      exit(-2);
    }

    const unsigned int expected_size = expected_vector_size[mode_id];
    if (expected_size != vector_size) {
      RCLCPP_ERROR(this->get_logger(), "Expected message size does not match received message Size. Expected size [%d], got [%lu]", expected_size, vector_size);
      exit(-2);
    }

    if (id < BASIC_BENCH_PUBLISHER_EXTRA) return; // synchronisation, the first (X) are not counted as measurements.

    // RCLCPP_INFO(this->get_logger(), "%d::%d", measure_id, id);

    measurements[measure_id++] = measurement;
    prev_msg_id = id;

    if (measure_id == BASIC_BENCH_DATA_BUF) {
      measure_id = 0;
      std::string result = write(measurements, BASIC_BENCH_DATA_BUF, BASIC_BENCH_WRITE_FOLDER);
      RCLCPP_INFO(this->get_logger(), "wrote [%u] measurements to file:\n[%s]", BASIC_BENCH_DATA_BUF, result.c_str());
      zeroBuffer(measurements, BASIC_BENCH_DATA_BUF);

      write_id++;
      if (write_id == BASIC_BENCH_BATCH_PER_SIZE) { // next data size incoming
        write_id = 0;
        mode_id++; // ensures the file written to is named appropriately.
        if (file_suffixes[mode_id] == nullptr) {
          RCLCPP_INFO(this->get_logger(), "Done with all scheduled measurements");
          exit(0);
        } else {
          RCLCPP_INFO(this->get_logger(), "ID [%d], expect size change to [%d] on the next message.", id, expected_vector_size[mode_id]);
        }
      }
    }
  }

  template <typename T>
  std::string write(T buf, uint32_t siz, const char* base_path) {
    static_assert(std::is_pointer<T>::value, "T must be of pointer type");

    std::string suffix = "batch" + std::string(file_suffixes[mode_id]) + std::to_string(write_id);
    std::string path = std::string(base_path) + suffix;
    
    std::ofstream file(path);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "could not open file\n[%s].", path);
      exit(-1);
    }
    // write a simple header for the file
    file << BASIC_BENCH_DATA_BUF << " measurements\n";

    std::for_each(buf, buf + siz, [&file](auto measurement){
      file << measurement << '\n';
    });

    return path;
  }

  template <typename T>
  void zeroBuffer(T buf, uint32_t siz) {
    static_assert(std::is_pointer<T>::value, "T must be of pointer type");
    std::fill(buf, buf+siz, 0);
  }

  /** member declarations */
  rclcpp::Subscription<basic_bench::msg::Bench>::SharedPtr subscription_;

  unsigned int expected_vector_size[7] = {0, 16, 48, 240, 1008, 4080, 65520};
  const char* file_suffixes[8] = {"_16b_", "_32b_", "_64b_", "_256b_", "_1024b_", "_4096b_", "_65536b_", nullptr};

  int prev_msg_id = -1; // used for asserting continuity of messages.
  int mode_id = 0;      // used for tracking what message size is being measured.
  int write_id = 0;     // used for file names 
  int measure_id = 0;   // used for indexing the data buffer
  int64_t measurements[BASIC_BENCH_DATA_BUF];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BenchSubscriber>());
  rclcpp::shutdown();
  return 0;
}
