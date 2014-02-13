/*
 * Publishing dummy target goal
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dummy_goal_publisher_node");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("target_goal_pose", 1, true);
	geometry_msgs::PoseStamped goal;
	goal.pose.position.x = 0.5;
	goal.pose.position.y = 0.4;
	goal.pose.position.z = 0.3;
	goal.pose.orientation.x = 1.0;
	goal.pose.orientation.y = 0.0;
	goal.pose.orientation.z = 0.0;
	goal.pose.orientation.w = 1.0;

	char c;
	while (ros::ok())
	{
		ROS_INFO("Press (p) to publish dummy goal, press (q) to exit");
		c = std::getchar();
		if (c == 'p')
		{
			ROS_INFO("Dummy Goal Published");
			pub.publish(goal);
		}
		else if (c == 'q')
		{
			ROS_INFO("Exit");
			return 0;
		}
		std::getchar();
	}
	ros::spin();
	return 0;
}
