#include "ros/ros.h"
#include "basic_bench/Bench.h"
#include "config.h"
#include <algorithm>
#include <type_traits>
#include <fstream>

#ifdef USE_CHRONO
#include <chrono>
#endif


int expected_vector_size[] = {0, 16, 48, 240, 1008, 4080, 65520};
const char* file_suffixes[] = {"_16b_", "_32b_", "_64b_", "_256b_", "_1024b_", "_4096b_", "_65536b_", nullptr};

int prev_msg_id = -1; // used for asserting continuity of messages.
int mode_id = 0;      // used for tracking what message size is being measured.
int write_id = 0;     // used for file names 
int measure_id = 0;   // used for indexing the data buffer
int64_t measurements[BASIC_BENCH_DATA_BUF];

template <typename T>
std::string write(T buf, uint32_t siz, const char* base_path) {
  static_assert(std::is_pointer<T>::value);

  std::string suffix = "batch" + std::string(file_suffixes[mode_id]) + std::to_string(write_id);
  std::string path = std::string(base_path) + suffix;
  
  std::ofstream file(path);
  if (!file) {
    ROS_ERROR("could not open file");
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
  static_assert(std::is_pointer<T>::value);
  std::fill(buf, buf+siz, 0);
}

void subCallback(const basic_bench::Bench::ConstPtr& msg) {
  int32_t id = msg->id;
  int64_t send_stamp = msg->stamp;
#ifdef USE_CHRONO
  auto now = std::chrono::steady_clock::now();
  auto now_64b = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  int64_t measurement = now_64b - send_stamp;
#else
  int64_t measurement = ros::Time::now().toNSec() - send_stamp;
#endif
  size_t vector_size = msg->junk.size();

  if (prev_msg_id != -1 && prev_msg_id + 1 != id) {
    ROS_ERROR("Cannot keep up with the Publisher. Expected id [%d], got [%d]", prev_msg_id + 1, id);
    exit(-2);
  }

  const int expected_size = expected_vector_size[mode_id];
  if (expected_size != vector_size) {
    ROS_ERROR("Expected message size does not match received message Size. Expected size [%d], got [%lu]", expected_size, vector_size);
    exit(-2);
  }

  if (id < BASIC_BENCH_PUBLISHER_EXTRA) return; // synchronisation, the first (X) are not counted as measurements.

  measurements[measure_id++] = measurement;
  prev_msg_id = id;

  if (measure_id == BASIC_BENCH_DATA_BUF) {
    measure_id = 0;
    std::string result = write(measurements, BASIC_BENCH_DATA_BUF, BASIC_BENCH_WRITE_FOLDER);
    ROS_INFO("wrote [%u] measurements to file:\n[%s]", BASIC_BENCH_DATA_BUF, result.c_str());
    zeroBuffer(measurements, BASIC_BENCH_DATA_BUF);

    write_id++;
    if (write_id == BASIC_BENCH_BATCH_PER_SIZE) { // next data size incoming
      write_id = 0;
      mode_id++; // ensures the file written to is named appropriately.
      if (file_suffixes[mode_id] == nullptr) {
        ROS_INFO("Done with all scheduled measurements");
        exit(0);
      } else {
        ROS_INFO("ID [%d], expect size change on the next message.", id);
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscriber");
  zeroBuffer(measurements, BASIC_BENCH_DATA_BUF);

  ros::NodeHandle n;
  
  // want enough buffer to fill one set of measurements, even if the publisher is going too fast.
  // This would be exposed in measurements where later data points "drift".
#ifdef USE_TCP_HINT
  ros::Subscriber sub = n.subscribe("bench", BASIC_BENCH_DATA_BUF, subCallback, ros::TransportHints().tcpNoDelay(true));
#else
  ros::Subscriber sub = n.subscribe("bench", BASIC_BENCH_DATA_BUF, subCallback);
#endif

  ROS_INFO("subscriber ready.\n\nDid you remember to empty the target folder? it is\n[%s].", BASIC_BENCH_WRITE_FOLDER);

  ros::spin();

  return 0;
}