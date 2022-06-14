#include "ros/ros.h"
#include "basic_bench/Bench.h"
#include "config.h"


static const int MESSAGE_BYTE_OFFSET = 16; // timestamp is 8 byte, id is 4 byte, serialized vector is preceded by a 4 byte length.
int sizeTarget(int target) { return target - MESSAGE_BYTE_OFFSET; };

int vectorSizeSchedule[] = {sizeTarget(16), sizeTarget(32), sizeTarget(64), sizeTarget(256), sizeTarget(1024), sizeTarget(4096), sizeTarget(65536), -1};
int sizeScheduleIndex = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");

  ros::NodeHandle n;

  //note to self: second parameter is a buffer used for holding messages before they get sent.
  ros::Publisher pub = n.advertise<basic_bench::Bench>("bench", BASIC_BENCH_DATA_BUF);


  ros::Rate loop_rate(100);
  int id = 0;
  while (ros::ok())
  {
    basic_bench::Bench msg; // note to self: It appears this default initializes everything, including the vector. 

    msg.stamp = ros::Time::now().toNSec();
    msg.id = id;
    msg.junk.resize(vectorSizeSchedule[sizeScheduleIndex]); // https://cplusplus.com/reference/vector/vector/resize/ -- "if val is specified (...) otherwise, they are value-initialized". In the case of chars, that's setting it all to zeroes.

    pub.publish(msg);

    ++id;

    if (id <= BASIC_BENCH_PUBLISHER_EXTRA) { // synchronisation with publisher: the first (X) do not contribute to the schedule.
      loop_rate.sleep();
      continue;
    };

    // when this is the case, enough data for one message size has been written. Go to the next size.
    if ((id - BASIC_BENCH_PUBLISHER_EXTRA) % (BASIC_BENCH_DATA_BUF * BASIC_BENCH_BATCH_PER_SIZE) == 0) {
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