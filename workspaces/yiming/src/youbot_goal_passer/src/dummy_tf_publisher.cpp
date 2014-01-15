#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_tf_publisher");

  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_dummy_revolute_joint"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_dummy_prismatic_x_link"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_dummy_prismatic_y_link"));

  ros::spin();
  return 0;
};





