/*
 * test.cpp
 *
 *  Created on: Oct 2, 2013
 *      Author: raha
 */
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm {
private:
	// Action client for the joint trajectory action
	// used to trigger the arm movement action
	TrajClient* traj_client_;
	bool _isReady;
public:

	//! Initialize the action client and wait for action server to come up
	RobotArm(bool localClient) {
		traj_client_ = NULL;
		if(localClient)
		{
			isReady(false);
			// tell the action client that we want to spin a thread by default
			traj_client_ = new TrajClient("arm_1/arm_controller/follow_joint_trajectory", true);

			// wait for action server to come up
			int count = 0;
			const int MAX_WAIT = 5;
			while (!traj_client_->waitForServer(ros::Duration(5.0))
					&& (count++ < MAX_WAIT)) {
				ROS_INFO(
						"Waiting for the joint_trajectory_action server, has waited for %d secs",
						count * 5);
			}

			if (count < MAX_WAIT)
			isReady(true);
		}
		else
		{
			isReady(true);
		}
	}

	//! Clean up the action client
	~RobotArm() {
		if(traj_client_ != NULL)
			delete traj_client_;
	}

	//! Sends the command to start a given trajectory
	void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
		// When to start the trajectory: 1s from now
		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(10.0);
		traj_client_->sendGoal(goal);
	}

	control_msgs::FollowJointTrajectoryGoal joint5test(int poseId)
	{
		control_msgs::FollowJointTrajectoryGoal goal;

		// First, the joint names, which apply to all waypoints
		goal.trajectory.joint_names.push_back("arm_joint_1");
		goal.trajectory.joint_names.push_back("arm_joint_2");
		goal.trajectory.joint_names.push_back("arm_joint_3");
		goal.trajectory.joint_names.push_back("arm_joint_4");
		goal.trajectory.joint_names.push_back("arm_joint_5");

		// We will have two waypoints in this goal trajectory
		goal.trajectory.points.resize(1);

		int ind  = 0;
		if(poseId == 0)
		{
			// First point
			setFirstPoint(goal, ind, 2.0);
		}
		else
		{
			// Second trajectory point
//			ind++;
			setSecondPoint(goal, ind, 2.0);
//			goal.trajectory.points[ind].positions[4] = 2.0;
		}

//		ROS_INFO("Joint 5 set-point is: %4.2f", goal.trajectory.points[ind].positions[4]);

		//we are done; return the goal
		return goal;
	}
	//! Generates a simple trajectory with two waypoints, used as an example
	/*! Note that this trajectory contains two waypoints, joined together
	 as a single trajectory. Alternatively, each of these waypoints could
	 be in its own trajectory - a trajectory can have one or more waypoints
	 depending on the desired application.
	 */
	control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory() {
		//our goal variable
		control_msgs::FollowJointTrajectoryGoal goal;

		// First, the joint names, which apply to all waypoints
		goal.trajectory.joint_names.push_back("arm_joint_1");
		goal.trajectory.joint_names.push_back("arm_joint_2");
		goal.trajectory.joint_names.push_back("arm_joint_3");
		goal.trajectory.joint_names.push_back("arm_joint_4");
		goal.trajectory.joint_names.push_back("arm_joint_5");

		// We will have two waypoints in this goal trajectory
		goal.trajectory.points.resize(3);

		int ind  = 0;
		// First point
		setFirstPoint(goal, ind, 2.0);


		// Second trajectory point
		ind++;
		setSecondPoint(goal, ind, 8.0);

		// First point
		ind++;
		setFirstPoint(goal, ind, 12.0);

		//we are done; return the goal
		return goal;
	}

	void setFirstPoint(control_msgs::FollowJointTrajectoryGoal &goal, int ind, double timeToReach)
	{

		// First trajectory point
					// Positions
//		ROS_INFO("creating 1st point");
		goal.trajectory.points[ind].positions.resize(5);
		goal.trajectory.points[ind].positions[0] = 0.25;
		goal.trajectory.points[ind].positions[1] = 0.30;
		goal.trajectory.points[ind].positions[2] =-0.45;
		goal.trajectory.points[ind].positions[3] = 0.14;
		goal.trajectory.points[ind].positions[4] = 0.80;
		// Velocities
//		ROS_INFO("1: vels");
		goal.trajectory.points[ind].velocities.resize(5);
		for (size_t j = 0; j < 5; ++j) {
			goal.trajectory.points[ind].velocities[j] = 0.00001;
		}

		// Accelerations
		goal.trajectory.points[ind].accelerations.resize(5);
		for (size_t j = 0; j < 5; ++j) {
			goal.trajectory.points[ind].accelerations[j] = 0.00001;
		}
		// To be reached 1 second after starting along the trajectory
		goal.trajectory.points[ind].time_from_start = ros::Duration(timeToReach);
	}

	void setSecondPoint(control_msgs::FollowJointTrajectoryGoal &goal, int ind, double timeToReach)
	{

		// Positions
//		ROS_INFO("creating 2nd point");
		goal.trajectory.points[ind].positions.resize(5);
		goal.trajectory.points[ind].positions[0] = 1.5;
		goal.trajectory.points[ind].positions[1] = 0.30;
		goal.trajectory.points[ind].positions[2] =-0.45;
		goal.trajectory.points[ind].positions[3] = 0.34;
		goal.trajectory.points[ind].positions[4] = 1.80;
		// Velocities
		goal.trajectory.points[ind].velocities.resize(5);
		for (size_t j = 0; j < 5; ++j) {
			goal.trajectory.points[ind].velocities[j] = 0.00001;
		}

		// Accelerations
		goal.trajectory.points[ind].accelerations.resize(5);
		for (size_t j = 0; j < 5; ++j) {
			goal.trajectory.points[ind].accelerations[j] = 0.00001;
		}

		// To be reached 2 seconds after starting along the trajectory
		goal.trajectory.points[ind].time_from_start = ros::Duration(timeToReach);
	}

	//! Returns the current state of the action
	actionlib::SimpleClientGoalState getState() {
		return traj_client_->getState();
	}

	bool isReady() {
		return _isReady;
	}

	void isReady(bool val) {
		_isReady = val;
	}

};

class RobotGripper {
public:
	RobotGripper() {
		// max travel distance in meters
		gripper_max_travel = 0.0115;
	}

	// returns goal for open and close states
	control_msgs::GripperCommandGoal getCloseGripperGoal()
	{
		//goal variable
		control_msgs::GripperCommandGoal goal;
		// setting data
		goal.command.position = 0.0;
		goal.command.max_effort = 0.5;

		return goal;
	}

	control_msgs::GripperCommandGoal getOpenGripperGoal()
	{
		//goal variable
		control_msgs::GripperCommandGoal goal;
		// setting data
		goal.command.position = gripper_max_travel;
		goal.command.max_effort = 0.5;

		return goal;
	}

private:
	// max travel distance in meters
	double gripper_max_travel;

};
void spinThread() {
	ros::spin();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_crtl_action_client_2");

	int poseId = 0;
	float setpoint = -1.0;
	if(argc>1)
		poseId= atoi(argv[1]);
	ROS_INFO("Using pose id %d",poseId);

	if(argc>2)
	{
		setpoint = atof(argv[2]);
		ROS_INFO("Using setpoint id %f",setpoint);
	}

	// Acces to the trajoxtory generatror
	RobotArm arm(false);
	RobotGripper gripper;
	// create the action client for the armcontroller
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> 
		acArm("/arm_1/arm_controller/follow_joint_trajectory", false);
	// create the action client for the armcontroller
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
			acGripper("/arm_1/gripper_controller/gripper_command", false);
	boost::thread spin_thread(&spinThread);

	// waiting for servers
	ROS_DEBUG("Waiting for action servers to start.");
	acArm.waitForServer();
	ROS_DEBUG("FollowJointTrajectoryAction server started.");
	acGripper.waitForServer();
	ROS_DEBUG("GripperCommandAction server started.");
	ROS_DEBUG("All action servers started, sending goals	.");

	// send a goal to the arm action
	control_msgs::FollowJointTrajectoryGoal armGoal;
//	armGoal = arm.armExtensionTrajectory();
	armGoal = arm.joint5test(poseId);
	if(setpoint > 0.0)
		armGoal.trajectory.points[0].positions[4] = setpoint;
	ROS_INFO("Arm goal created, now sending...");
	acArm.sendGoal(armGoal);

	//wait for the action to return
	bool finished_before_timeout = acArm.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = acArm.getState();
		ROS_INFO("Arm action finished: %s", state.toString().c_str());
	} else
	{
		ROS_INFO("Arm action did not finish before the time out -> cancelling all goals.");
		acArm.cancelAllGoals();
	}


//	// send a goal to the arm action
//	control_msgs::GripperCommandGoal gripperGoal;
//	gripperGoal = gripper.getOpenGripperGoal();
//	ROS_INFO("Open gripper goal created, now sending");
//	acGripper.sendGoal(gripperGoal);
//	//wait for the action to return
//	finished_before_timeout = acGripper.waitForResult(ros::Duration(10.0));
//	if (finished_before_timeout) {
//		actionlib::SimpleClientGoalState state = acGripper.getState();
//		ROS_INFO("Open gripper action finished: %s", state.toString().c_str());
//	} else
//	{
//		ROS_INFO("Open gripper action did not finish before the time out -> cancelling all goals.");
//		acGripper.cancelAllGoals();
//	}
//
//	gripperGoal = gripper.getCloseGripperGoal();
//	ROS_INFO("Close gripper goal created, now sending");
//	acGripper.sendGoal(gripperGoal);
//	//wait for the action to return
//	finished_before_timeout = acGripper.waitForResult(ros::Duration(10.0));
//	if (finished_before_timeout) {
//		actionlib::SimpleClientGoalState state = acGripper.getState();
//		ROS_INFO("Close gripper action finished: %s", state.toString().c_str());
//	} else
//	{
//		ROS_INFO("Close gripper action did not finish before the time out -> canceling all goals.");
//		acGripper.cancelAllGoals();
//	}

	// shutdown the node and join the thread back before exiting
	ROS_INFO("Shutting down..");
	ros::shutdown();
	spin_thread.join();

	//exit
	return 0;
}

int main_(int argc, char** argv) {
	// Init the ROS node
	ros::init(argc, argv, "trajectory_test");

	RobotArm arm(true);
	if (!arm.isReady()) {
		ROS_INFO("Failed to establish contact to server. Shutting down!");
		return 0;
	}
	// Start the trajectory
	arm.startTrajectory(arm.armExtensionTrajectory());
	// Wait for trajectory completion
	while (!arm.getState().isDone() && ros::ok()) {
		usleep(50000);
	}

	// Wait for trajectory completion
	while (!arm.getState().isDone() && ros::ok()) {
		usleep(50000);
	}
}
