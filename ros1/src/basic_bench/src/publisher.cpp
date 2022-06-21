#include "ros/ros.h"
#include "basic_bench/Bench.h"
#include "config.h"
#include <cmath>
#include <algorithm>
#include <type_traits>
#include <fstream>

#ifdef USE_CHRONO
#include <chrono>
#endif

static const int MESSAGE_BYTE_OFFSET = 16; // timestamp is 8 byte, id is 4 byte, serialized vector is preceded by a 4 byte length.
int sizeTarget(int target) { return target - MESSAGE_BYTE_OFFSET; };

int vectorSizeSchedule[] = {sizeTarget(16), sizeTarget(32), sizeTarget(64), sizeTarget(256), sizeTarget(1024), sizeTarget(4096), sizeTarget(65536), -1};
int sizeScheduleIndex = 0;

constexpr int AJ_buf_size = (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE);
int64_t AJ_buf[AJ_buf_size];
int AJ_buf_id = 0;

template <typename T>
void writeJitterBuffer(T buf, uint32_t buf_size, int message_size) {
  static_assert(std::is_pointer<T>::value);

  std::string suffix = std::string("jitter") + std::to_string(message_size);
  std::string path = std::string(BASIC_BENCH_JITTER_FOLDER) + suffix;
  
  std::ofstream file(path);
  if (!file) {
    ROS_ERROR("could not open file");
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
  static_assert(std::is_pointer<T>::value);
  std::fill(buf, buf+siz, 0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");

  ros::NodeHandle n;

  //note to self: second parameter is a buffer used for holding messages before they get sent.
  ros::Publisher pub = n.advertise<basic_bench::Bench>("bench", BASIC_BENCH_DATA_BUF);

  int rate_per_second = 100; // need to use this number later
  uint64_t nanosecond_time_between_message_expected = (1000000000 / rate_per_second);// assumes integer divisble. beware.
  uint64_t nanosecond_time_message_jitter;
  uint64_t nanosecond_time_message_expected; // first measurement + i * ns_time_between

  zeroBuffer(AJ_buf, AJ_buf_size);

  ros::Rate loop_rate(rate_per_second);
  int id = 0;
  while (ros::ok())
  {
    basic_bench::Bench msg; // note to self: It appears this default initializes everything, including the vector. 

    msg.id = id;
    msg.junk.resize(vectorSizeSchedule[sizeScheduleIndex]); // https://cplusplus.com/reference/vector/vector/resize/ -- "if val is specified (...) otherwise, they are value-initialized". In the case of chars, that's setting it all to zeroes.
    // there is ros::Time::now().toNSec() but we want to stay consistent with ROS2
#ifdef USE_CHRONO
    auto now = std::chrono::steady_clock::now();
    auto now_64b = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    msg.stamp = now_64b;
#else
    auto now_64b = ros::Time::now().toNSec(); // need this later for AJ measuring
    msg.stamp = now_64b;
#endif
    
    pub.publish(msg);

    if (id == 0) {
      nanosecond_time_message_expected = now_64b; 
    }
    nanosecond_time_message_expected += nanosecond_time_between_message_expected;

    // <Notice Me> increment of ID is done here.
    id++;
    if (id <= BASIC_BENCH_PUBLISHER_EXTRA) { // synchronisation with publisher: the first (X) do not contribute to the schedule.
      loop_rate.sleep();
      continue;
    };

    // record activation jitter by comparing with the expected stamp
    nanosecond_time_message_jitter = (static_cast<int64_t>(now_64b - nanosecond_time_message_expected));
    AJ_buf[AJ_buf_id] = nanosecond_time_message_jitter;
    AJ_buf_id++;

    // when this is the case, enough data for one message size has been written. Go to the next size.
    if ((id - BASIC_BENCH_PUBLISHER_EXTRA) % (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE) == 0) {
      // write recorded AJ data
      writeJitterBuffer(AJ_buf, AJ_buf_size, vectorSizeSchedule[sizeScheduleIndex] + MESSAGE_BYTE_OFFSET);
      zeroBuffer(AJ_buf, AJ_buf_size);
      AJ_buf_id = 0;
      // move to next size
      sizeScheduleIndex++;
      int scheduledSize = vectorSizeSchedule[sizeScheduleIndex];
      ROS_INFO("ID [%d], switch to size [%d]", id, scheduledSize);
      if (scheduledSize == -1) {
        ROS_INFO("Published all messages for all scheduled sizes.");
        exit(0);
      }
    }

    loop_rate.sleep();
  }


  return 0;
}