#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");
  ROS_INFO("START SPIN");
  ros::spin();
  return 0;
}
