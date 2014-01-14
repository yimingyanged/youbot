#ifndef YOUBOT_RELAY_CONTROLLER_H
#define YOUBOT_RELAY_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <move_base_msgs/MoveBaseActionResult.h>
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

#define USE_YB_NAV

namespace youbot_goal_passer
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

	class YoubotGoalPasser
	{
		public:
			/** Constructor */
			YoubotGoalPasser(ros::NodeHandle * nh_, std::string input_ns, std::string request_ns, std::string result_ns, std::string arm_ns, std::string base_ns);
		
		private:
			/** Member Variables */
			Controller_State_t														 current_state;	//!< State Control for the FSM
			bool																	 base_goal_retrieved;
			bool																	 base_succeeded;//!< Base movement flag
			actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;			//!< Action Server

		#ifndef USE_YB_NAV
			actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_ac;		//!< The action client for the base goal message
		#else
			ros::Publisher															 base_pub;		//!< Alternatively the publisher for the base
		#endif
			ros::Subscriber															 request_sub;	//!< Goal request subscriber
			ros::Subscriber															 result_sub;
		#ifndef USE_YB_NAV												//!< The Goal message for the base (different types)
			control_msgs::FollowJointTrajectoryGoal		base_goal;		
		#else
			geometry_msgs::PoseStamped					base_goal;
		#endif
			actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_ac;		//!< The action client for the arm goal message

			/** Implementation of the Action Server */
			void executeCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr & goal);

			/** Implementation of the action client */
			void doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result, bool base);

			/** Implementation of goal request retrireving */
			void goal_requestCallback(const moveit_msgs::MotionPlanRequest::ConstPtr & request);

			void base_doneCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr & result);
	};
}
#endif
