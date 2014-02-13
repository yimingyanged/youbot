/*
 * Youbot manipulator using IK.
 * Compute joint values using 8 DOF, but plan with 5 DOF arm
 *
 * yiming yang
 * 23 Jan 2014
 */
#ifndef YOUBOT_IK_MANIPULATOR_H
#define YOUBOT_IK_MANIPULATOR_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>
namespace youbot_ik_manipulator
{
	/**
	 * @brief Youbot manipulator using 8DOF IK and 5DOF plan
	 */
	class YoubotIKManipulator
	{
	public:
		/**
		 * @brief Youbot IK manipulator constructor
		 * @param nh_				ros nodehandle
		 * @param ik_group_ns		group name for IK
		 * @param plan_group_ns		group name for plan
		 * @param base_ns			topic of base navigation goal
		 * @param gripper_ns		gripper action server name
		 * @param goal_ns			goal topic
		 * @param pre_offset		offset for pre grasp
		 * @param use_base			true if move the base
		 */
		YoubotIKManipulator(ros::NodeHandle * nh_, std::string ik_group_ns, std::string plan_group_ns, std::string base_ns, std::string gripper_ns, std::string goal_ns, geometry_msgs::Pose pre_offset, bool use_base);

	private:
		/**
		 * @brief Planning to a target pose
		 * @param pose target pose
		 * @return true if plan found
		 */
		bool planPose(const geometry_msgs::PoseStamped pose);
		moveit::planning_interface::MoveGroup group_;	/** moveit planning group */

		/**
		 * @brief target pose call back
		 * @param target target pose
		 */
		void targetCallback(const geometry_msgs::PoseStamped::ConstPtr & target);

		/** Implementation of gripper action */
		void gripper_doneCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::GripperCommandResult::ConstPtr & result);

		/**
		 * @brief open gripper
		 * @return action goal for open gripper position
		 */
		control_msgs::GripperCommandGoal open_gripper();

		/**
		 * @brief grasp position
		 * @return action goal for gripper grasp position
		 */
		control_msgs::GripperCommandGoal grasp_gripper();

		/**
		 * @brief command line start listener
		 * @return true if start
		 */
		bool startListener();

		actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac;	/** Gripper Action Client */
		bool has_gripper_;
		ros::NodeHandle nh_;	/** NodeHandler */
		ros::Subscriber target_sub_;	/** target subscriber */
		ros::Publisher base_pub_;	/** base navigation goal publisher */
		std::string target_pose_ns_;	/** target pose topic name */
		moveit_msgs::DisplayTrajectory display_trajectory_;	/** trajectory */
		geometry_msgs::PoseStamped pose_;	/** target pose */
		bool visual_; /** true if want to publish visual state to rviz */
		bool succeeded_;
		geometry_msgs::Pose pregrasp_offset_;
		/**
		 * @brief there is a bug in current moveit move_group version, group_.plan is a dead loop
		 * so use asyncSpinner to get plan back and then move. REMOVE when the bug fixed
		 */
		ros::AsyncSpinner spinner_;
		moveit_msgs::GetPositionIK::Request ik_request_;
		moveit_msgs::GetPositionIK::Response ik_response_;
	};
}
#endif
