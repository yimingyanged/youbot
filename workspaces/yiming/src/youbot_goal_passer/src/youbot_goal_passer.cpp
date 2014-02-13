#include "youbot_goal_passer/youbot_goal_passer.h"

youbot_goal_passer::Controller_State_t::Controller_State_t()
{
	state = RCS_IDLE;
	base_done = arm_done = true;
}

youbot_goal_passer::Relayer_State_t youbot_goal_passer::Controller_State_t::getState()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	return state;
}

void youbot_goal_passer::Controller_State_t::reset()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	base_done = arm_done = false;
	state = RCS_BUSY;
}

void youbot_goal_passer::Controller_State_t::setBaseDone()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	base_done = true;
	if (arm_done) //!< If arm was previously done...
	{
		state = RCS_IDLE;
	}
}

void youbot_goal_passer::Controller_State_t::setArmDone()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	arm_done = true;
	if (base_done) //!< If base was previously done...
	{
		state = RCS_IDLE;
	}
}

youbot_goal_passer::YoubotGoalPasser::YoubotGoalPasser(ros::NodeHandle * nh_, std::string input_ns, std::string request_ns, std::string result_ns, std::string arm_ns, std::string base_ns, bool use_base):
		as_(*nh_, input_ns, boost::bind(&YoubotGoalPasser::executeCallback, this, _1), false),
#ifndef USE_YB_NAV
		base_ac(base_ns, true),
#endif
		arm_ac(arm_ns, true),
		use_base_(use_base)
{
	
	base_succeeded=false;
	base_goal_retrieved=false;
	/** Check that individual servers are up / or set up publisher...*/
	bool server_ok = false;
	ROS_INFO("Waiting for Servers to come online!");
	while (!server_ok)
	{
	#ifndef USE_YB_NAV
		server_ok = base_ac.waitForServer(ros::Duration(0.1)) && arm_ac.waitForServer(ros::Duration(0.1)); //!< If both are ok then will be true
	#endif 
		server_ok = arm_ac.waitForServer(ros::Duration(0.1));
	}
#ifdef USE_YB_NAV
	base_pub = nh_->advertise<geometry_msgs::PoseStamped>(base_ns, 1);
	//request_sub = nh_->subscribe(request_ns, 1, &youbot_goal_passer::YoubotGoalPasser::goal_requestCallback,this);
	//result_sub = nh_->subscribe(result_ns , 1, &youbot_goal_passer::YoubotGoalPasser::base_doneCallback,this);
#endif

	/** Start our action server */
	as_.start();
}

void youbot_goal_passer::YoubotGoalPasser::executeCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr & goal)
{
	ROS_INFO("Start Execute Callback");
	/** Data Variables */

	control_msgs::FollowJointTrajectoryGoal		arm_goal;		//!< The Goal message for the arm
	bool 										success = true;

	/** Prepare the headers for the trajectory*/
#ifndef USE_YB_NAV
	base_goal.trajectory.header = goal->trajectory.header;
#else
	base_goal.header = goal->trajectory.header;
#endif
	arm_goal.trajectory.header = goal->trajectory.header;
	
	/** Send Messages */
	current_state.reset();
//#ifndef USE_YB_NAV
//	base_ac.sendGoal(base_goal, boost::bind(&YoubotGoalPasser::doneCb, this, _1, _2, true));
//#else
//	base_pub.publish(base_goal);
//#endif

	if (use_base_)
	{
		int trails=0;
		for (int i = trails; i < 10; i++)	//Wait for 10 sec for base movement
		{
			ROS_INFO("Trails %d",i);
			if (base_succeeded)
			{
				break;
			}
			ros::Duration(1).sleep();
		}
	}
	if (base_succeeded || !use_base_)
	{
		arm_ac.sendGoal(*goal, boost::bind(&YoubotGoalPasser::doneCb, this, _1, _2, false));
		if (success)
		{
			while(current_state.getState() != RCS_IDLE) //!< If not yet idle
			{
				if (as_.isPreemptRequested()) { as_.setPreempted(); success = false; break; }
				else if (!as_.isActive())      { as_.setAborted(); 	success = false; break; }
				else if (!ros::ok())		  {	as_.setPreempted();	success = false; break; }
				else
				{
					ros::Duration(0.01).sleep();	//Sleep for some time
				}
			}
		}
	
		if (success)
		{
			ROS_INFO("Arm movement succeeded");
			as_.setSucceeded();
		}
	}
	else
	{
		ROS_INFO("Base movement not yet succeeded. Arm movement aborted");
		as_.setAborted();
	}
}				

void youbot_goal_passer::YoubotGoalPasser::doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result, bool base)
{
#ifndef USE_YB_NAV
	if (base)
	{
		current_state.setBaseDone();
	}
	else
	{
		current_state.setArmDone();
	}
#else
	current_state.setBaseDone();
	current_state.setArmDone();
#endif
}

void youbot_goal_passer::YoubotGoalPasser::goal_requestCallback(const moveit_msgs::MotionPlanRequest::ConstPtr & request)
{
	base_goal.header = request->workspace_parameters.header;
	//Rough implementation, needs to be improved
	for (int i = 0; i < request -> goal_constraints[0].joint_constraints.size(); i++)
	{
		if (request ->goal_constraints[0].joint_constraints[i].joint_name == "dummy_prismatic_x_joint")
			base_goal.pose.position.x = request->goal_constraints[0].joint_constraints[i].position;
		else if (request ->goal_constraints[0].joint_constraints[i].joint_name == "dummy_prismatic_y_joint")
			base_goal.pose.position.y = request->goal_constraints[0].joint_constraints[i].position;
		else if (request ->goal_constraints[0].joint_constraints[i].joint_name == "dummy_revolute_joint")
		{
			base_goal.pose.orientation.w = cos((request->goal_constraints[0].joint_constraints[i].position)/2);
			base_goal.pose.orientation.z = sin((request->goal_constraints[0].joint_constraints[i].position)/2);
			base_goal.pose.orientation.w /= (base_goal.pose.orientation.w*base_goal.pose.orientation.w + base_goal.pose.orientation.z*base_goal.pose.orientation.z);
			base_goal.pose.orientation.z /= (base_goal.pose.orientation.w*base_goal.pose.orientation.w + base_goal.pose.orientation.z*base_goal.pose.orientation.z);
		}
	}

	base_goal_retrieved=true;
	ROS_INFO("Goal Request Retrieved");
}
void youbot_goal_passer::YoubotGoalPasser::base_doneCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr & result)
{
	if (result->status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
	{
		ROS_INFO("Base Movement Has Been Succeeded! Now Performing Arm Movement");
		base_succeeded=true;
	}
	else
	{
		ROS_INFO("Base Movement Has Not Yet Been Succeeded! Arm Movement Is Not Going To Be Performed");
		base_succeeded=false;
	}
	ros::Duration(0.1).sleep();
}


