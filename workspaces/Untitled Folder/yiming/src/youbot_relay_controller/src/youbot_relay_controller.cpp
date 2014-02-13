#include "youbot_relay_controller/youbot_relay_controller.h"

youbot_relay_controller::Controller_State_t::Controller_State_t()
{
	state = RCS_IDLE;
	base_done = arm_done = true;
}

youbot_relay_controller::Relayer_State_t youbot_relay_controller::Controller_State_t::getState()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	return state;
}

void youbot_relay_controller::Controller_State_t::reset()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	base_done = arm_done = false;
	state = RCS_BUSY;
}

void youbot_relay_controller::Controller_State_t::setBaseDone()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	base_done = true;
	if (arm_done) //!< If arm was previously done...
	{
		state = RCS_IDLE;
	}
}

void youbot_relay_controller::Controller_State_t::setArmDone()
{
	boost::mutex::scoped_lock(member_lock);     //!< Lock Access
	arm_done = true;
	if (base_done) //!< If base was previously done...
	{
		state = RCS_IDLE;
	}
}

youbot_relay_controller::YoubotRelayController::YoubotRelayController(ros::NodeHandle * nh_, std::string input_ns, std::string arm_ns, std::string base_ns) : 
		as_(*nh_, input_ns, boost::bind(&YoubotRelayController::executeCallback, this, _1), false),
#ifndef USE_YB_NAV
		base_ac(base_ns, true),
#endif
		arm_ac(arm_ns, true)
{
	
	/** Joint Name Setup */
	ROS_INFO("Loading joint names from YAML");
	YAML::Node name_configs = YAML::LoadFile((std::string(ros::package::getPath("youbot_base_ik_weighted_moveit")).append("/config/controllers.yaml")));
	if (name_configs["controller_list"])	//!< If loaded correct file
	{
		ROS_INFO("YAML loaded");
		for (int i = 0; i < 3; i++)	//!< First the base joints
		{
			joint_map[name_configs["controller_list"][0]["joints"][i].as<std::string>()] = true;
		#ifdef USE_YB_NAV
			base_jnts.push_back(name_configs["controller_list"][0]["joints"][i].as<std::string>());
		#endif
		}
		for (int i = 3; i < name_configs["controller_list"][0]["joints"].size(); i++) //!< And now the arm joints
		{
			joint_map[name_configs["controller_list"][0]["joints"][i].as<std::string>()] = false;
		}
	}
	else
	{
		ROS_ERROR("File is inexistent or broken.");
	}
	ROS_INFO("Successfully Loaded File!");
	
	/** Check that individual servers are up / or set up publisher...*/
	bool server_ok = false;
	ROS_INFO("Waiting for Servers to come online!");
	int trails=0;
	while (!server_ok)
	{
	#ifndef USE_YB_NAV
		server_ok = base_ac.waitForServer(ros::Duration(0.1)) && arm_ac.waitForServer(ros::Duration(0.1)); //!< If both are ok then will be true
	#else 
		server_ok = arm_ac.waitForServer(ros::Duration(0.1));
	#endif
		trails++;
		if (trails > 20) //wait for 2 sec
		{
			ROS_INFO("Servers not connected......");
			break;
		}
	}
#ifdef USE_YB_NAV
	base_pub = nh_->advertise<geometry_msgs::PoseStamped>(base_ns, 1);
#endif

	/** Start our action server */
	as_.start();
}

void youbot_relay_controller::YoubotRelayController::executeCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr & goal)
{
	/** Data Variables */
#ifndef USE_YB_NAV												//!< The Goal message for the base (different types)
	control_msgs::FollowJointTrajectoryGoal		base_goal;		
#else
	geometry_msgs::PoseStamped					base_goal;
#endif
	control_msgs::FollowJointTrajectoryGoal		arm_goal;		//!< The Goal message for the arm
	bool 										success = true;

	/** Prepare the headers for the trajectory*/
#ifndef USE_YB_NAV
	base_goal.trajectory.header = goal->trajectory.header;
#else
	base_goal.header = goal->trajectory.header;
#endif
	arm_goal.trajectory.header = goal->trajectory.header;

	std::vector<int> base_order;
	std::vector<int> arm_order;

	/** First identify the indices */
	for (int i=0; i<goal->trajectory.joint_names.size(); i++)
	{
		if (joint_map.find(goal->trajectory.joint_names[i]) != joint_map.end()) //!< Ensure that the joint is actually registered
		{
			if (goal->trajectory.joint_names[i].compare(base_jnts[BASE_X]) == 0)
			{
				base_order.push_back(i);
			}
			else
			{
				arm_order.push_back(i);
			}
		}
		else
		{
			ROS_WARN("Found Unknown Joint '%s'", goal->trajectory.joint_names[i].c_str());
		}
	}

	for (int i=0; i<goal->trajectory.points.size(); i++)
	{
		//!< Allow preempting/aborting
		if (as_.isPreemptRequested()) { ROS_INFO("Pre-empt Request"); as_.setPreempted(); success = false; break; }
		else if (!as_.isActive())      { as_.setAborted(); 	success = false; break; }
		else if (!ros::ok())		  {	as_.setPreempted();	success = false; break; }
		else
		{
			trajectory_msgs::JointTrajectoryPoint base_point;
			trajectory_msgs::JointTrajectoryPoint arm_point;
			for (int j=0; j<arm_order.size(); j++)
			{
				arm_point.positions.push_back(goal->trajectory.points[i].positions[arm_order[j]]);
			}


//////////////////////// HERE!!!!




//	Need to set time from start field!!!




			ROS_INFO("Name: %s", goal->trajectory.joint_names[i].c_str());
			if (joint_map.find(goal->trajectory.joint_names[i]) != joint_map.end()) //!< Ensure that the joint is actually registered
			{
				if (joint_map[goal->trajectory.joint_names[i]]) //!< If true then goes into the base
				{
				#ifndef USE_YB_NAV
					base_goal.trajectory.joint_names.push_back(goal->trajectory.joint_names[i]);
					base_goal.trajectory.points.push_back(goal->trajectory.points[i]);
				#else
					if (goal->trajectory.joint_names[i].compare(base_jnts[BASE_X]) == 0)
					{
						base_goal.pose.position.x = goal->trajectory.points[i].positions.back();
					}
					else if (goal->trajectory.joint_names[i].compare(base_jnts[BASE_Y]) == 0)
					{
						base_goal.pose.position.y = goal->trajectory.points[i].positions.back();
					}
					else if (goal->trajectory.joint_names[i].compare(base_jnts[BASE_YAW]) == 0)
					{
						base_goal.pose.orientation.w = cos((goal->trajectory.points[i].positions.back())/2);
						base_goal.pose.orientation.z = sin((goal->trajectory.points[i].positions.back())/2);
						base_goal.pose.orientation.w /= (base_goal.pose.orientation.w*base_goal.pose.orientation.w + base_goal.pose.orientation.z*base_goal.pose.orientation.z);
						base_goal.pose.orientation.z /= (base_goal.pose.orientation.w*base_goal.pose.orientation.w + base_goal.pose.orientation.z*base_goal.pose.orientation.z);
					}
					else
					{
						ROS_WARN("Lookup Error in the indexing for the base navigation stack! Base pose is potentially invalid");
					}
				#endif
				}
				else
				{
					arm_goal.trajectory.joint_names.push_back(goal->trajectory.joint_names[i]);
					arm_goal.trajectory.points.push_back(goal->trajectory.points[i]);
				}
			}
		}
	}

	if (success) //!< If no failure yet...
	{
		/** Now separate the path_tolerance */
		for (int i=0; i<goal->path_tolerance.size(); i++)
		{
			//!< Allow preempting/aborting
			if (as_.isPreemptRequested()) { as_.setPreempted(); success = false; break; }
			else if (!as_.isActive())      { as_.setAborted(); 	success = false; break; }
			else if (!ros::ok())		  {	as_.setPreempted();	success = false; break; }
			else
			{
				if (joint_map.find(goal->path_tolerance[i].name) != joint_map.end()) //!< Ensure that the joint is actually registered
				{
	  				if (joint_map[goal->path_tolerance[i].name])
					{
					#ifndef USE_YB_NAV
						base_goal.path_tolerance.push_back(goal->path_tolerance[i]);
					#endif
					}
					else
					{
						arm_goal.path_tolerance.push_back(goal->path_tolerance[i]);
					}
				}
			}
		}
	}

	if (success) //!< If no critical failure still...
	{
		/** Finally separate the goal_tolerance */
		for (int i=0; i<goal->goal_tolerance.size(); i++)
		{
			//!< Allow preempting/aborting
			if (as_.isPreemptRequested()) { as_.setPreempted(); success = false; break; }
			else if (!as_.isActive())      { as_.setAborted(); 	success = false; break; }
			else if (!ros::ok())		  {	as_.setPreempted();	success = false; break; }
			else
			{
				if (joint_map.find(goal->goal_tolerance[i].name) != joint_map.end()) //!< Ensure that the joint is actually registered
				{
	  				if (joint_map[goal->goal_tolerance[i].name])
					{
					#ifndef USE_YB_NAV
						base_goal.goal_tolerance.push_back(goal->goal_tolerance[i]);
					#endif
					}
					else
					{
						arm_goal.goal_tolerance.push_back(goal->goal_tolerance[i]);
					}
				}
			}
		}
	}

	/** Send Messages */
	if (success)
	{
		current_state.reset();
	#ifndef USE_YB_NAV
		base_ac.sendGoal(base_goal, boost::bind(&YoubotRelayController::doneCb, this, _1, _2, true));
	#else
		base_pub.publish(base_goal);
	#endif
		arm_ac.sendGoal(arm_goal, boost::bind(&YoubotRelayController::doneCb, this, _1, _2, false));
	}

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
		as_.setSucceeded();
	}
}				

void youbot_relay_controller::YoubotRelayController::doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result, bool base)
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

youbot_relay_controller::YoubotRelayPassive::YoubotRelayPassive(ros::NodeHandle * nh_, std::string arm_in, std::string base_in, std::string arm_out, std::string base_out) : 
		as_(*nh_, arm_in, boost::bind(&YoubotRelayPassive::executeCallback, this, _1), false),
		arm_ac(arm_out, true)
{	
	/** Check that individual servers are up / or set up publisher...*/
	bool server_ok = false;
	ROS_INFO("Waiting for Servers to come online!");
	while (!server_ok)
	{
		server_ok = arm_ac.waitForServer(ros::Duration(0.1));
	}
	prev_sub = nh_->subscribe<moveit_msgs::DisplayTrajectory>(base_in, 1, boost::bind(&YoubotRelayPassive::prevCb, this, _1));
	
	/** Start our action server / publisher */
	as_.start();
    base_pub = nh_->advertise<geometry_msgs::PoseStamped>(base_out, 1);
}

void youbot_relay_controller::YoubotRelayPassive::prevCb(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
	current_preview = *(msg);	//!< Dereference and copy 
}

void youbot_relay_controller::YoubotRelayPassive::executeCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr & goal)
{
	/** Data Variables */
	bool 						success = true;
    geometry_msgs::PoseStamped	base_goal;
	
	/** Prepare the base goal from the current internal copy **/
	int index = current_preview.trajectory[0].joint_trajectory.points.size()-2;
	base_goal.header = current_preview.trajectory[0].joint_trajectory.header;
	base_goal.pose.position.x = current_preview.trajectory[0].joint_trajectory.points[index].positions[BASE_X];
    base_goal.pose.position.y = current_preview.trajectory[0].joint_trajectory.points[index].positions[BASE_Y];
    base_goal.pose.orientation.w = cos((goal->trajectory.points[index].positions[BASE_YAW])/2);
	base_goal.pose.orientation.z = sin((goal->trajectory.points[index].positions[BASE_YAW])/2);
	base_goal.pose.orientation.w /= (base_goal.pose.orientation.w*base_goal.pose.orientation.w + base_goal.pose.orientation.z*base_goal.pose.orientation.z);
	base_goal.pose.orientation.z /= (base_goal.pose.orientation.w*base_goal.pose.orientation.w + base_goal.pose.orientation.z*base_goal.pose.orientation.z);
	ROS_INFO("%d   %f %f %f %f ", index, base_goal.pose.position.x, base_goal.pose.position.y, base_goal.pose.orientation.w, base_goal.pose.orientation.z);

	/** Send Message(s) */	
    current_state.reset();
	base_pub.publish(base_goal);
    arm_ac.sendGoal(*goal, boost::bind(&YoubotRelayPassive::doneCb, this, _1, _2));

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

	if (success)
	{
		as_.setSucceeded();
	}
}				

void youbot_relay_controller::YoubotRelayPassive::doneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result)
{
	current_state.setBaseDone();
	current_state.setArmDone();
}
