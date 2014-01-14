#ifndef YOUBOT_RELAY_CONTROLLER_H
#define YOUBOT_RELAY_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cstdlib>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <cmath>

//Currently, the base joints must be the first 3 defined (although in any order) in the yaml file

#define BASE_X   1
#define BASE_Y 	 2
#define BASE_YAW 0

namespace youbot_relay_controller
{
	enum Relayer_State_t
	{
		RCS_IDLE = 0,
		RCS_BUSY = 1
	};

	class Controller_State_t
	{
		private:
			boost::mutex 		member_lock;
			Relayer_State_t 	state;
			bool 				base_done;
			bool				arm_done;
		
		public:
			Controller_State_t();	//Default Constructor
			Relayer_State_t getState();
			void reset();			//Reset the done fields and set the state to BUSY
			void setBaseDone();		//
			void setArmDone();		//
	};

	class YoubotRelayController
	{
		public:
			/** Constructor */
			YoubotRelayController(ros::NodeHandle * nh_, std::string input_ns, std::string arm_ns, std::string base_ns);
		
		private:
			/** Member Variables */
			Controller_State_t														 current_state;	//!< State Control for the FSM
			//!< 
			actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;			//!< Action Server (
			std::map<std::string, bool>											     joint_map;		//!< Maps the joint name to whether it relates to the base (true) or the arm (false)
			std::vector<std::string>												 base_jnts;		//!< The list of base joints
			actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_ac;		//!< The action client for the base goal message
			ros::Publisher															 base_pub;		//!< Alternatively the publisher for the base
			actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_ac;		//!< The action client for the arm goal message

			/** Implementation of the Action Server */
			void executeCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr & goal);

			/** Implementation of the action client */
			void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result, bool base);
	};

    class YoubotRelayPassive
	{
		public:
			/** Constructor */
			YoubotRelayPassive(ros::NodeHandle * nh_, std::string arm_in, std::string base_in, std::string arm_out, std::string base_out);
		
		private:
			/** Member Variables */
			Controller_State_t														 current_state;	//!< State Control for the FSM
			actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;			//!< Action Server
			ros::Subscriber															 prev_sub;		//!< Subscriber to the preview state
			ros::Publisher															 base_pub;		//!< The publisher for the base
			actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_ac;		//!< The action client for the arm goal message
			moveit_msgs::DisplayTrajectory											 current_preview;

			/** Implementation of the Action Server */
			void executeCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr & goal);

			/** Implementation of the action client(s) */
			void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result);
			void prevCb(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
	};

}
#endif
