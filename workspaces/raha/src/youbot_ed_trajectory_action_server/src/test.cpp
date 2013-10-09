/*
 * test.cpp
 *
 *  Created on: Oct 2, 2013
 *      Author: raha
 */
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
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
	if(localClient)
	{
		isReady(false);
		// tell the action client that we want to spin a thread by default
		traj_client_ = new TrajClient("arm_2/arm_controller/follow_joint_trajectory", true);

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
	delete traj_client_;
}

//! Sends the command to start a given trajectory
void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
	// When to start the trajectory: 1s from now
	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(10.0);
	traj_client_->sendGoal(goal);
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
	ROS_INFO("entry");
	// First, the joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back("arm_2_joint_1");
	goal.trajectory.joint_names.push_back("arm_2_joint_2");
	goal.trajectory.joint_names.push_back("arm_2_joint_3");
	goal.trajectory.joint_names.push_back("arm_2_joint_4");
	goal.trajectory.joint_names.push_back("arm_2_joint_5");

	// We will have two waypoints in this goal trajectory
	goal.trajectory.points.resize(2);

	// First trajectory point
	// Positions
	ROS_INFO("creating 1st point");
	int ind = 0;
	goal.trajectory.points[ind].positions.resize(5);
	goal.trajectory.points[ind].positions[0] = 0.2500000;
	goal.trajectory.points[ind].positions[1] = 0.2256610;
	goal.trajectory.points[ind].positions[2] =-0.2486099;
	goal.trajectory.points[ind].positions[3] = 0.0069911;
	goal.trajectory.points[ind].positions[4] = 0.9257740;
	// Velocities
	ROS_INFO("1: vels");
	goal.trajectory.points[ind].velocities.resize(5);
	for (size_t j = 0; j < 5; ++j) {
		goal.trajectory.points[ind].velocities[j] = 0.0;
	}

	// Accelerations
	goal.trajectory.points[ind].accelerations.resize(5);
	for (size_t j = 0; j < 5; ++j) {
		goal.trajectory.points[ind].accelerations[j] = 0.0;
	}
	// To be reached 1 second after starting along the trajectory
	goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);


	// Second trajectory point
	// Positions
	ROS_INFO("creating 2nd point");
	ind += 1;
	goal.trajectory.points[ind].positions.resize(5);
	goal.trajectory.points[ind].positions[0] = 0.2500000;
	goal.trajectory.points[ind].positions[1] = 0.2256610;
	goal.trajectory.points[ind].positions[2] =-0.2486099;
	goal.trajectory.points[ind].positions[3] = 0.0069911;
	goal.trajectory.points[ind].positions[4] = 1.9257740;
	// Velocities
	ROS_INFO("2: vels");
	goal.trajectory.points[ind].velocities.resize(5);
	for (size_t j = 0; j < 5; ++j) {
		goal.trajectory.points[ind].velocities[j] = 0.0;
	}

	// Accelerations
	ROS_INFO("2: acc");
	goal.trajectory.points[ind].accelerations.resize(5);
	for (size_t j = 0; j < 5; ++j) {
		goal.trajectory.points[ind].accelerations[j] = 0.0;
	}

	// To be reached 2 seconds after starting along the trajectory
	ROS_INFO("2: time");
	goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

	//we are done; return the goal
	return goal;
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

void spinThread() {
	ros::spin();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_crtl_action_client_2");
	// Acces to the trajoxtory generatror
	RobotArm arm(false);

	// create the action client
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> 
		ac("/arm_2/arm_controller/follow_joint_trajectory", false);
	boost::thread spin_thread(&spinThread);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	control_msgs::FollowJointTrajectoryGoal goal;
	goal = arm.armExtensionTrajectory();
	ROS_INFO("Goal created!");
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");

	// shutdown the node and join the thread back before exiting
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
//		return 0;
	}
	// Start the trajectory
	arm.startTrajectory(arm.armExtensionTrajectory());
	// Wait for trajectory completion
	while (!arm.getState().isDone() && ros::ok()) {
		usleep(50000);
	}
}
