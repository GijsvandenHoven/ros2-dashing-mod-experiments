#include "ros/ros.h"
#include "basic_bench/Bench.h"
#include <algorithm>
#include <type_traits>
#include <fstream>

#define BASIC_BENCH_DATA_BUF 10000U
#define BASIC_BENCH_WRITE_FOLDER "/home/ruben/june-measurements/results/basic_bench/"

int prev_msg_id = -1; // used for asserting continuity of messages.
int write_id = 0;     // used for file names 
int measure_id = 0;   // used for indexing the data buffer
int64_t measurements[BASIC_BENCH_DATA_BUF];

template <typename T>
std::string write(T buf, uint32_t siz, const char* base_path) {
  static_assert(std::is_pointer<T>::value);

  std::string suffix = "batch_" + std::to_string(write_id);
  std::string path = std::string(base_path) + suffix;
  write_id++;
  
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
  int64_t measurement = ros::Time::now().toNSec() - send_stamp;

  if (prev_msg_id != -1 && prev_msg_id + 1 != id) {
    ROS_ERROR("Cannot keep up with the Publisher. Expected id [%d], got [%d]", prev_msg_id + 1, id);
    exit(-2);
  }

  measurements[measure_id++] = measurement;
  prev_msg_id = id;

  if (measure_id == BASIC_BENCH_DATA_BUF) {
    measure_id = 0;
    ROS_INFO("buffer filled.");
    std::string result = write(measurements, BASIC_BENCH_DATA_BUF, BASIC_BENCH_WRITE_FOLDER);
    ROS_INFO("wrote to file [%s]", result.c_str());
    zeroBuffer(measurements, BASIC_BENCH_DATA_BUF);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscriber");
  zeroBuffer(measurements, BASIC_BENCH_DATA_BUF);

  ros::NodeHandle n;
  
  // want enough buffer to fill one set of measurements, even if the publisher is going too fast.
  // This would be exposed in measurements where later data points "drift".
  ros::Subscriber sub = n.subscribe("bench", BASIC_BENCH_DATA_BUF, subCallback);

  ROS_INFO("subscriber ready.\n\nDid you remember to empty the target folder? it is\n[%s].", BASIC_BENCH_WRITE_FOLDER);

  ros::spin();

  return 0;
}