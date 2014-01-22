#include "youbot_manipulator/youbot_manipulator.h"

namespace youbot_manipulator
{	YoubotManipulator::YoubotManipulator(ros::NodeHandle * nh, std::string group_ns, std::string target_pose_ns, std::string display_ns, std::string gripper_ns, double pre_offset, bool visual):
	group_(group_ns),
	visual_(visual),
	spinner_(1),
	target_pose_ns_(target_pose_ns),
	nh_(*nh),
	gripper_ac(gripper_ns, true),
	pregrap_offset_(pre_offset)
	{

		target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(target_pose_ns_, 1, boost::bind(&YoubotManipulator::targetCallback, this, _1));
		//plan_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(display_ns, 1, true);
		succeeded_ = false;
	}

	bool YoubotManipulator::planPose(const geometry_msgs::PoseStamped pose)
	{
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		ROS_INFO("Planning pose received (%f,%f,%f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
		geometry_msgs::Pose pregrap_pose = pose.pose;
		pregrap_pose.position.x = pregrap_pose.position.x - pregrap_offset_;
		group_.setStartStateToCurrentState();
		group_.allowReplanning(true);

		group_.setPoseTarget(pose.pose);
		moveit::planning_interface::MoveGroup::Plan target_plan;
		bool success = group_.plan(target_plan);
		if (success)
		{
			ROS_INFO("Planning SUCCESSED");
			succeeded_ = true;
			return true;
		}
		else
		{
			ROS_INFO("Planning FAILED");
			return false;
		}
	}
	void YoubotManipulator::targetCallback(const geometry_msgs::PoseStamped::ConstPtr & target)
	{
		if (!succeeded_)
		{
			ROS_INFO("Object Target Pose Received");
			spinner_.start();
			pose_= *(target);
			planPose(pose_);

			if (succeeded_)
			{
				ROS_INFO("Motion plan generated. press (m) to move");
				char m = std::getchar();

				if (m == 'm')
				{
					ROS_INFO("MOVING TO GOAL POSE");
					group_.move();
					ros::Duration(group_.getPlanningTime()).sleep();

					gripper_ac.sendGoal(open_gripper(), boost::bind(&YoubotManipulator::gripper_doneCallback, this, _1, _2));
				}
				else
				{
					ROS_INFO("Motion canceled");
				}
				spinner_.stop();
				nh_.shutdown();
			}
		}
		else
		{
			ROS_INFO("AsyncSpinner Stopped 2");
			spinner_.stop();
		}

	}
	void YoubotManipulator::gripper_doneCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::GripperCommandResult::ConstPtr & result)
	{
		if (result->reached_goal)
		{
			ROS_INFO("Gripper command succeeded");
		}
	}

	control_msgs::GripperCommandGoal YoubotManipulator::open_gripper()
	{
		control_msgs::GripperCommandGoal open_goal;
		open_goal.command.position = 0.01;	//!!!!!!!!!!!!!!!!!!!!!!!!!
		return open_goal;
	}

	control_msgs::GripperCommandGoal YoubotManipulator::grasp_gripper()
	{
		control_msgs::GripperCommandGoal grasp_goal;
		grasp_goal.command.position = 0.004;	//!!!!!!!!!!!!!!!!!!!!!!!!!
		return grasp_goal;
	}

	//void restartCallback()
	bool YoubotManipulator::startListener()
	{
		ROS_INFO("Start Listenner. Press (s) to start");
		char c = std::getchar();
		if (c == 's')
		{
			ROS_INFO("Subscribing to target pose");
			return true;
		}
		else
		{
			return false;
		}
	}
}


