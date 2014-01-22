#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_tf_publisher");

  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.5, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(1.0, 0.0, 0.0, 1.0) );
  while(ros::ok())
  {
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "dummy_prismatic_link"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy_prismatic_link", "dummy_revolute_link"));
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "xtion_link", "ar_marker"));
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  }
  return 0;
};





