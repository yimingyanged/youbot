#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/MoveGroupAction.h>

bool planning;
void plan_feedbackCb(const moveit_msgs::MoveGroupActionFeedback::ConstPtr & feedback){
	ROS_INFO("PLANNING INFO");
	if (feedback->feedback.state =="PLANNING")
	{
		ROS_INFO("PLANNING STARTED. STOP PUBLISHING TF");
		planning = true;
	}
	else
	{
		ROS_INFO("PLANNING FINISHED. PUBLISHING TF");
		planning = false;
	}
}
int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_tf_publisher");

  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(1.0, 0.0, 0.0, 1.0) );
  ros::Subscriber plan_sub = node.subscribe<moveit_msgs::MoveGroupActionFeedback>("move_group/feedback", 1, plan_feedbackCb);
  planning = false;
  while(ros::ok())
  {
	  if (!planning)
	  {
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "moveit_fixed_link"));
	  	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "moveit_fixed_link", "dummy_prismatic_link"));
	  	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy_prismatic_link", "dummy_revolute_link"));
	  	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy_revolute_link", "base_link"));

	  	  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "xtion_link", "ar_marker"));
	  }
	  ros::spinOnce();
	  ros::Duration(0.05).sleep();
  }
  return 0;
};





