#include "ros/ros.h"
#include "basic_bench/Bench.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const basic_bench::Bench::ConstPtr& msg)
{
  ROS_INFO("I heard: [%lld]", msg->stamp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;
  // todo: include id in bench to see if messages are dropped.
  ros::Subscriber sub = n.subscribe("bench", 1000, chatterCallback);

  ros::spin();

  return 0;
}