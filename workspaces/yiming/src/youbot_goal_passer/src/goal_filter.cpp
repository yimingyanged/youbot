#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string.h>
#include <boost/bind.hpp>
class goal_filter
{
public:
	goal_filter(ros::NodeHandle * nh_, std::string sub_ns_, std::string pub_ns_):
		spinner_(1)
	{
		hasPose_ = false;
		filter_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(pub_ns_, 1, true);
		origin_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(sub_ns_, 1, boost::bind(&goal_filter::originCallback, this, _1));
		//boost::thread pid = boost::thread(boost::bind(&goal_filter::init, this));
		ROS_INFO("Goal filter initialized");
		spinner_.start();
		init();
	}
protected:
	bool start()
	{
		if (!hasPose_){
			ROS_INFO("Target pose not received");
			return false;
		}
		ROS_INFO("Publish target pose");
		filter_pub_.publish(pose_);
		return true;
	}

	void init()
	{
		char c;
		while(true)
		{
			ROS_INFO("Press (p) to publish, (q) to exit");
			c = std::getchar();
			if (c == 'p')
				bool res = start();
			else if (c == 'q')
				break;
		}
		ROS_INFO("Exit goal filter");
	}
private:
	ros::Subscriber	origin_sub_;
	ros::Publisher	filter_pub_;
	geometry_msgs::PoseStamped pose_;
	bool hasPose_;
	boost::thread pid;
	ros::AsyncSpinner spinner_;
	void originCallback(const geometry_msgs::PoseStamped::ConstPtr & origin)
	{
		pose_ = * origin;
		hasPose_ = true;
	}
};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_filter");
	ros::NodeHandle nh_;
	goal_filter GoalFilter(&nh_, "ar_pose_geometry", "target_goal_pose");
}
