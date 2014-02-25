/*
 * Youbot goal filter.
 * Since current moveit has a bug in move group service, we should not send to much goals to the server.
 * Just hold the goal, send it when ready. Could be removed when moveit's bug been fixed, or we could keep if for further applications.
 *
 * yiming yang
 * 21 Jan 2014
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string.h>
#include <boost/bind.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class goal_filter
{
public:
	goal_filter(ros::NodeHandle * nh_, std::string pub_ns_)
	{
		hasPose_ = false;
		filter_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(pub_ns_, 1, true);
		//boost::thread pid = boost::thread(boost::bind(&goal_filter::init, this));
		ROS_INFO("Goal filter initialized");
		convert();
	}

private:
	ros::Publisher	filter_pub_;
	geometry_msgs::PoseStamped pose_;
	bool hasPose_;
	static tf::TransformBroadcaster br;
	tf::TransformListener listener;
	tf::StampedTransform transform;

	void convert()
	{
		while (ros::ok()){
			try{
				listener.waitForTransform("/base_link", "ar_marker", ros::Time(0), ros::Duration(10.0) );
				listener.lookupTransform("/base_link", "ar_marker", ros::Time(0), transform);
				pose_.header.frame_id = "odom";
				pose_.header.stamp = ros::Time::now();
				pose_.pose.position.x = transform.getOrigin().x();
				pose_.pose.position.y = transform.getOrigin().y();
				pose_.pose.position.z = transform.getOrigin().z();
				pose_.pose.orientation.x = transform.getRotation().x();
				pose_.pose.orientation.y = transform.getRotation().y();
				pose_.pose.orientation.z = transform.getRotation().z();
				pose_.pose.orientation.w = transform.getRotation().w();
				//ROS_INFO("Publish target pose (%f, %f, %f) (%f, %f, %f, %f)", pose_.position.x, pose_.position.y, pose_.position.z,pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
				filter_pub_.publish(pose_);
				ros::Duration(0.1).sleep();
			}
			catch (tf::TransformException ex){
				ROS_WARN("Target Not Detected, keep searching");
			}
		}
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_filter");
	ros::NodeHandle nh_;
	goal_filter GoalFilter(&nh_, "target_goal_pose");
	ros::spin();
}
