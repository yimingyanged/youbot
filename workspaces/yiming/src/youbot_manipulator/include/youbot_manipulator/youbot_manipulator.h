/*
 * Youbot manipulator.
 * Receives target pose and send out move_group request.
 * Currently using async spinner, Need to be removed when moveit bug fixed
 *
 * yiming yang
 * 19 Jan 2014
 */
#ifndef YOUBOT_MANIPULATOR_H
#define YOUBOT_MANIPULATOR_H

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
namespace youbot_manipulator
{
	/**
	 * @brief Youbot arm and base combined manipulator
	 */
	class YoubotManipulator
	{
	public:
		/**
		 * @brief Youbot manipulator constructor
		 * @param nh_				ros nodehandle
		 * @param group_ns			moveit planning group namespace
		 * @param target_pose_ns	target pose topic name space
		 * @param display_ns		display topic for rviz
		 * @param gripper_ns		gripper action topic
		 * @param visual			true if want to publish to rviz
		 */
		YoubotManipulator(ros::NodeHandle * nh_, std::string group_ns, std::string target_pose_ns, std::string display_ns, std::string gripper_ns, geometry_msgs::Pose pre_offset, bool visual);

		/**
		 * @brief Compute IK
		 */

		/**
		 * @brief Planning to a target pose
		 * @param pose target pose
		 * @return true if plan found
		 */
		bool planPose(const geometry_msgs::PoseStamped pose);
		moveit::planning_interface::MoveGroup group_;	/** moveit planning group */

	private:
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
		bool hasGripper_;
		ros::NodeHandle nh_;	/** NodeHandler */
		ros::Subscriber target_sub_;	/** target subscriber */
		ros::Publisher plan_pub_;	/** move plan publisher to rviz */
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

	};
}
#endif
