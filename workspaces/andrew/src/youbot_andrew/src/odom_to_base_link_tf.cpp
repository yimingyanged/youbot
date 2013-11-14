#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>


void odomCallback(const nav_msgs::OdometryConstPtr& msg){
  geometry_msgs::Pose robot_pose = msg->pose.pose;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(robot_pose.position.x, robot_pose.position.y, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, robot_pose.orientation.z, robot_pose.orientation.w) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_to_base_link_tf");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/odom", 10, &odomCallback);

  ros::spin();
  return 0;
};
