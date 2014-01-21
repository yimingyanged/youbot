#include <ros/ros.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <string.h>
class DummyNav
{
public:
	DummyNav(ros::NodeHandle * nh_, std::string goal_ns, std::string result_ns)
	{
		res_pub = nh_->advertise<move_base_msgs::MoveBaseResult>(result_ns, 10);
		goal_sub = nh_->subscribe(goal_ns, 10, &goalCallback, this);
	}
private:
	ros::Publisher res_pub;
	ros::Subscriber goal_sub;

	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal)
	{
		ROS_INFO("Navigation goal received. Using dummy navigation.");
		ROS_WARN("Moving to goal position directly, make sure there is NOTHING in front of youbot.");

	}
};
int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_navigation");

  ros::NodeHandle n;
  ros::Publisher fb_pub = n.advertise<move_base_msgs::MoveBaseResult>("move_base/result", 10);
  ros::Subscriber goal_sub = n.subscribe("move_base_goal", 10, goalCallback)
  return 0;
};
