#include "ros/ros.h"
#include "basic_bench/Bench.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bridge_publisher");

  ros::NodeHandle n;

  //note to self: second parameter is a buffer used for holding messages before they get sent.
  ros::Publisher pub = n.advertise<basic_bench::Bench>("bridge_bench", 1000);

  ros::Rate loop_rate(100);
  int id = 0;
  while (ros::ok())
  {
    basic_bench::Bench msg; // note to self: It appears this default initializes everything, including the vector. 

    msg.id = id++;
    msg.junk.resize(10);

    pub.publish(msg);
    
    ROS_INFO("send %d", msg.id);

    loop_rate.sleep();
  }


  return 0;
}