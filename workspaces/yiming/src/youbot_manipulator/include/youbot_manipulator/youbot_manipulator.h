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
#include <boost/thread/mutex.hpp>
#include <string.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/Twist.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>


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
		 * @param nh_			ros nodehandle
		 * @param ik_ns			group name for ik solution
		 * @param plan_ns		group name for planning
		 * @param gripper_ns	action server name of gripper action
		 * @param target_ns		target pose topic name
		 * @param base_ns		base navigation goal topic
		 * @param pre_offset	pre grasp offset
		 * @param has_base		true if need to perform base movement
		 */
		YoubotManipulator(ros::NodeHandle * nh_, std::string ik_ns, std::string plan_ns, std::string gripper_ns, std::string target_ns, std::string base_ns, geometry_msgs::Pose pre_offset, bool has_base);

	private:

		/**
		 * @brief Moveit Kinematics setup
		 */
		moveit::planning_interface::MoveGroup ik_group_;	/** moveit group for IK compute	*/
		moveit::planning_interface::MoveGroup plan_group_;	/** moveit group for FK planning	*/
		moveit::planning_interface::MoveGroup::Plan plan_;	/** motion plan	*/
		robot_model_loader::RobotModelLoader robot_model_loader;
		//robot_model::RobotModelPtr kinematic_model;	/**	Kinematic model for IK calculation and motion planning	*/
		//robot_state::RobotStatePtr kinematic_state;	/**	Kinematic state for IK calculation and motion planning	*/

//		const robot_state::JointModelGroup * ik_model_group_;	/**	Planning group for IK	*/
//		const robot_state::JointModelGroup * plan_group_;	/** Planning group for arm planning	*/
//		const std::vector<std::string> & ik_joint_names_;	/** Joint names for IK calculation	*/
//		const std::vector<std::string> & plan_joint_names_;	/** Joint names for planning	*/

		/**
		 * @brief get joint values of planning group
		 * @param	group planning group
		 * @return	joint values
		 */
		std::vector<double> getJointValues(const robot_state::JointModelGroup * group);
		/**
		 * @brief Implementation of target setting
		 * @param target target pose
		 */
		void setTarget(const geometry_msgs::PoseStamped & target);
		void targetCallback(const geometry_msgs::PoseStamped::ConstPtr & target);

		geometry_msgs::PoseStamped getGoal();
		/**
		 * @brief Implementation of gripper action
		 */
		void gripperDoneCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::GripperCommandResult::ConstPtr & result);

		/**
		 * @brief Manipulation start trigger
		 * @param start	0: direct grasp;	1: pre-grasp and then grasp
		 */
		void startCallback(const std_msgs::BoolConstPtr &  start);

		/**
		 * @brief Implementation of target searching
		 */
		bool searchTarget();

		/**
		 * @brief open gripper
		 * @return action goal for open gripper position
		 */
		control_msgs::GripperCommandGoal openGripper();

		/**
		 * @brief grasp position
		 * @return action goal for gripper grasp position
		 */
		control_msgs::GripperCommandGoal graspGripper();

		/**
		 * @brief Grasp target directly without pre-grasp manipulation
		 * @return true if grasp succeeded
		 */
		bool directGrasp();

		/**
		 * @brief move base to goal pose before perform arm manipulation
		 * @param	joint_values joint values with first three the base goal
		 * @return 	true if succeeded
		 */
		bool moveBase(const std::vector<double> &  joint_values, double dist_offset, geometry_msgs::Twist & final_move);

		bool goHome();
		/**
		 * @brief get ik solution for ik group
		 */
		bool getIK(std::vector<double> &  joint_values, geometry_msgs::PoseStamped goal);

		/**
		 * @brief get grasp pose from target centre pose
		 */
		geometry_msgs::PoseStamped setToGraspPose(const geometry_msgs::PoseStamped g, bool pre);
		actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac_;	/** Gripper Action Client */
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> base_ac_;	/**	Navigation Action Client	*/
		bool has_gripper_;	/** true if has gripper action server	*/
		bool has_base_;	/** true if allowed to perform base movement	*/
		ros::Subscriber target_sub_;	/** target subscriber */
		ros::Subscriber start_sub_;	/** start subscriber	*/
		ros::Publisher vel_pub_;	/** base cmd_vel publisher */
		ros::Publisher done_pub_;	/** done publisher */

		std::string ik_ns_;	/**	group name for IK calculation	*/
		std::string plan_ns_;	/** group name for planning */
		moveit_msgs::DisplayTrajectory display_trajectory_;	/** trajectory */
		geometry_msgs::PoseStamped goal_;	/** target pose */
		geometry_msgs::Pose offset_;
		bool has_goal_;	/** true after goal pose been received	*/
		boost::mutex lock_;	/** boost locker	*/
		ros::AsyncSpinner spinner_;	/** Async Spinner for multi threads	*/

	};
}
#endif
