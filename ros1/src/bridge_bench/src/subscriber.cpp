#include "ros/ros.h"
#include "basic_bench/Bench.h"

void subCallback(const basic_bench::Bench::ConstPtr& msg) {
    ROS_INFO("receive %d", msg->id);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bridge_subscriber");

  ros::NodeHandle n;

  //note to self: second parameter is a buffer used for holding messages before they get sent.
  ros::Subscriber sub = n.subscribe("bridge_bench", 1000, subCallback, ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("ready");

  ros::spin();


  return 0;
}