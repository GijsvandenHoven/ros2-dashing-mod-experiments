#include "ros/ros.h"
#include "basic_bench/Bench.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");

  ros::NodeHandle n;

  //note to self: second parameter is a buffer used for holding messages before they get sent.
  ros::Publisher pub = n.advertise<basic_bench::Bench>("bench", 1000);


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    basic_bench::Bench msg;

    msg.stamp = 0;

    pub.publish(msg);

    loop_rate.sleep();
  }


  return 0;
}