#include "youbot_manipulator/youbot_manipulator.h"

namespace youbot_manipulator
{	YoubotManipulator::YoubotManipulator(ros::NodeHandle * nh, std::string group_ns, std::string target_pose_ns, std::string display_ns, std::string gripper_ns, geometry_msgs::Pose pre_offset, bool visual):
	group_(group_ns),
	visual_(visual),
	spinner_(1),
	target_pose_ns_(target_pose_ns),
	nh_(*nh),
	gripper_ac(gripper_ns, true),
	pregrasp_offset_(pre_offset)
	{

		target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(target_pose_ns_, 1, boost::bind(&YoubotManipulator::targetCallback, this, _1));
		//plan_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(display_ns, 1, true);
		succeeded_ = false;
		ROS_INFO("Wait for gripper server");
		hasGripper_ = gripper_ac.waitForServer(ros::Duration(1));
		if (!hasGripper_)
			ROS_INFO("Gripper server not connected, gripper command will not be executed");
	}

	bool YoubotManipulator::planPose(const geometry_msgs::PoseStamped pose)
	{
		succeeded_=false;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		ROS_INFO("Planning pose received (%f,%f,%f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
		group_.setStartStateToCurrentState();

		group_.setPoseTarget(pose.pose);
		moveit::planning_interface::MoveGroup::Plan target_plan;
		spinner_.start();
		succeeded_= group_.plan(target_plan);
		if (succeeded_)
		{
			ROS_INFO("Planning SUCCESSED");
			succeeded_ = true;
			return succeeded_;
		}
		else
		{
			ROS_INFO("Planning FAILED");
			return succeeded_;
		}
	}
	void YoubotManipulator::targetCallback(const geometry_msgs::PoseStamped::ConstPtr & target)
	{
		if (!succeeded_)
		{
			ROS_INFO("Object Target Pose Received");
			//spinner_.start();
			pose_= *(target);

			/** Plan to pre grasp pose */
			geometry_msgs::PoseStamped pre_grasp_pose_ = pose_;
			pre_grasp_pose_.pose.position.x = pre_grasp_pose_.pose.position.x + pregrasp_offset_.position.x;
			pre_grasp_pose_.pose.position.y = pre_grasp_pose_.pose.position.y + pregrasp_offset_.position.y;
			pre_grasp_pose_.pose.position.z = pre_grasp_pose_.pose.position.z + pregrasp_offset_.position.z;
			pre_grasp_pose_.pose.orientation.x = pre_grasp_pose_.pose.orientation.x + pregrasp_offset_.orientation.x;
			pre_grasp_pose_.pose.orientation.y = pre_grasp_pose_.pose.orientation.y + pregrasp_offset_.orientation.y;
			pre_grasp_pose_.pose.orientation.z = pre_grasp_pose_.pose.orientation.z + pregrasp_offset_.orientation.z;
			pre_grasp_pose_.pose.orientation.w = pre_grasp_pose_.pose.orientation.w + pregrasp_offset_.orientation.w;

			/** Check plan */
			if (planPose(pre_grasp_pose_))
			{
				/** Move to pre grasp pose */
				ROS_INFO("Motion plan generated. Move to Pre Grasp pose");
				group_.move();
				ros::Duration(group_.getPlanningTime()+5.0).sleep();

				/** Open gripper */
				if (!hasGripper_)
				{
					ROS_INFO("Gripper server not connected, gripper command will not be executed");
				}
				else
				{
					ROS_INFO("Opening gripper for grasp");
					gripper_ac.sendGoal(open_gripper(), boost::bind(&YoubotManipulator::gripper_doneCallback, this, _1, _2));
				}

				/** Move to grasp pose */
				geometry_msgs::PoseStamped grasp_pose_ = pose_;
				grasp_pose_.pose = pregrasp_offset_;
				if (planPose(pose_))
				{
					ROS_INFO("Motion plan generated. Move to Grasp pose");
					group_.move();
					ros::Duration(group_.getPlanningTime()).sleep();

					/** Open gripper */
					if (!hasGripper_)
					{
						ROS_INFO("Gripper server not connected, gripper command will not be executed");
					}
					else
					{
						ROS_INFO("Closing gripper to grasp");
						gripper_ac.sendGoal(grasp_gripper(), boost::bind(&YoubotManipulator::gripper_doneCallback, this, _1, _2));
					}
				}

			}
			/*
			if (succeeded_)
			{
				ROS_INFO("Motion plan generated. press (m) to move");
				char m = std::getchar();

				if (m == 'm')
				{
					ROS_INFO("MOVING TO GOAL POSE");
					group_.move();
					ros::Duration(group_.getPlanningTime()+5.0).sleep();
					if (!hasGripper_)
					{
						ROS_INFO("Gripper server not connected, gripper command will not be executed");
					}
					else
					{
						gripper_ac.sendGoal(open_gripper(), boost::bind(&YoubotManipulator::gripper_doneCallback, this, _1, _2));
					}
				}
				else
				{
					ROS_INFO("Motion cancelled");
				}
				succeeded_ = false;
				//spinner_.stop();
				//nh_.shutdown();
			}
			*/
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


