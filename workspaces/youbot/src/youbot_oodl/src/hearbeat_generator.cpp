#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv) {
  ros::init(argc,argv,"heartbeat_generator");
  ros::NodeHandle n;
  ros::Publisher hp;
  hp = n.advertise<std_msgs::Bool>("youbot_network_heartbeat",1);

  std_msgs::Bool b;
  b.data=true;
  
  ros::Rate rate(10); //Hz
  while (n.ok()) {
    hp.publish(b);

    ROS_INFO("Heartbeat");

    ros::spinOnce();
    rate.sleep();
  }
}
