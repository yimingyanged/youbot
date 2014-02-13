#include "youbot_manipulator/youbot_ik_manipulator.h"

namespace youbot_ik_manipulator{
YoubotIKManipulator::YoubotIKManipulator(ros::NodeHandle * nh_, std::string ik_group_ns, std::string plan_group_ns, std::string base_ns, std::string gripper_ns, std::string goal_ns, geometry_msgs::Pose pre_offset, bool has_base):
gripper_ac_(gripper_ns, true)
{

	target_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(goal_ns, 1, boost::bind(&YoubotManipulator::targetCallback, this, _1));
	base_pub_ = nh_->advertise<>
}
}
