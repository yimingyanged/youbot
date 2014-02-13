/*
 * Publishing dummy target goal
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dummy_goal_publisher_node");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Pose>("target_goal_pose", 1, true);
	ros::Publisher stamped_pub = n.advertise<geometry_msgs::PoseStamped>("target_goal_pose_stamped", 1, true);
	geometry_msgs::Pose goal;
	geometry_msgs::PoseStamped gs;
	goal.position.x = -5.0;
	goal.position.y = 0.4;
	goal.position.z = 0.3;
	goal.orientation.x = 0.0;
	goal.orientation.y = 0.0;
	goal.orientation.z = 1.0;
	goal.orientation.w = 1.0;

	gs.header.frame_id = "odom";
	gs.header.stamp = ros::Time::now();
	gs.pose = goal;
	char c;
	while (ros::ok())
	{
		ROS_INFO("Press (p) to publish dummy goal, press (q) to exit");
		c = std::getchar();
		if (c == 'p')
		{
			ROS_INFO("Dummy Goal Published");
			pub.publish(goal);
			stamped_pub.publish(gs);
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
