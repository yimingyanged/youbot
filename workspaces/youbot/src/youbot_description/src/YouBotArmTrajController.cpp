#include <YouBotArmTrajController.h>
#include <joint_state_observer.h>

#include <youbot_trajectory_action_server/joint_trajectory_action.h>

#include <sstream>

namespace youBot
{

YouBotArmTrajController::YouBotArmTrajController()
{

}

YouBotArmTrajController::YouBotArmTrajController(ros::NodeHandle n) :
    		node(n)
{

	youBotChildFrameID = "base_link"; //holds true for both: base and arm
	armJointStateMessages.clear();

	//n.param("trajectoryActionServerEnable", trajectoryActionServerEnable, false);
	//n.param("trajectoryVelocityGain", trajectoryVelocityGain, 0.0);
	//n.param("trajectoryPositionGain", trajectoryPositionGain, 5.0);
	gripperCycleCounter = 0;
	diagnosticNameArms = "platform_Arms";
	diagnosticNameBase = "platform_Base";
	dashboardMessagePublisher = n.advertise < youbot_common::PowerBoardState > ("/dashboard/platform_state", 1);
	diagnosticArrayPublisher = n.advertise < diagnostic_msgs::DiagnosticArray > ("/diagnostics", 1);
}

YouBotArmTrajController::~YouBotArmTrajController()
{

	this->stop();
	dashboardMessagePublisher.shutdown();
	diagnosticArrayPublisher.shutdown();

}

void YouBotArmTrajController::initializeArmTraj(std::string armName, bool enableStandardGripper)
{
	int armIndex;
	youbot::JointName jointNameParameter;
	std::string jointName;
	stringstream topicName;
	stringstream serviceName;

	armJointTrajectoryAction = new actionlib::ActionServer<
			control_msgs::FollowJointTrajectoryAction>(
					node, topicName.str(), boost::bind(&YouBotArmTrajController::armJointTrajectoryGoalCallback, this, _1, 1),
					boost::bind(&YouBotArmTrajController::armJointTrajectoryCancelCallback, this, _1, armIndex), false);

	topicName.str("");
	topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "joint_states";


	/* initialize message vector for arm joint states */
	sensor_msgs::JointState dummyMessage;

	/* setup frame_ids */
	youBotArmFrameID = "arm"; //TODO find default topic name
	youBotConfiguration.hasArms = true;

	// currently no action is running

	if(armHasActiveJointTrajectoryGoals.size() <= armIndex)
		armHasActiveJointTrajectoryGoals.push_back(false);
	else
		armHasActiveJointTrajectoryGoals[armIndex] = false;


	//  these has to be fixed..
	armActiveJointTrajectoryGoals;

	//tracejoint = 4;
	//myTrace = new youbot::DataTrace(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(tracejoint), "Joint4TrajectoryTrace");

	// we can handle actionlib requests only after the complete initialization has been performed
	armJointTrajectoryAction->start();
	gripperCommandAction->start();

	ROS_INFO("Arm \"%s\" is initialized.", armName.c_str());
	ROS_INFO("System has 1 initialized arm(s).");
}

void YouBotArmTrajController::armJointTrajectoryGoalCallback(
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex)
{
	boost::mutex::scoped_lock lock(_mutex);
	ROS_DEBUG("Goal for arm%i received", armIndex + 1);
	ROS_ASSERT(armIndex < 1);

	trajectory_msgs::JointTrajectory trajectory = youbotArmGoal.getGoal()->trajectory;

	// validate that the correct number of joints is provided in the goal
	if (trajectory.joint_names.size() != static_cast<unsigned int>(youBotArmDoF))
	{
		ROS_ERROR("Trajectory is malformed! Goal has %i joint names, but only %i joints are supported",
				static_cast<int>(trajectory.joint_names.size()), youBotArmDoF);
		youbotArmGoal.setRejected();
		return;
	}

	std::vector < youbot::JointTrajectory > jointTrajectories(youBotArmDoF);

	youbot::TrajectorySegment segment;
	for (unsigned int i = 0; i < trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
		// validate the trajectory point
		if ((point.positions.size() != static_cast<unsigned int>(youBotArmDoF)
				|| point.velocities.size() != static_cast<unsigned int>(youBotArmDoF)
				|| point.accelerations.size() != static_cast<unsigned int>(youBotArmDoF)))
		{
			ROS_ERROR("A trajectory point is malformed! %i positions, velocities and accelerations must be provided",
					youBotArmDoF);
			youbotArmGoal.setRejected();
			return;
		}

		for (int j = 0; j < youBotArmDoF; j++)
		{
			segment.positions = point.positions[j] * radian;
			segment.velocities = point.velocities[j] * radian_per_second;
			segment.accelerations = point.accelerations[j] * radian_per_second / second;
			segment.time_from_start = boost::posix_time::microsec(point.time_from_start.toNSec() / 1000);
			jointTrajectories[j].segments.push_back(segment);
		}
	}
	for (int j = 0; j < youBotArmDoF; j++)
	{
		jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
	}


	// replace the old goal with the new one
	youbotArmGoal.setAccepted();
	if(armActiveJointTrajectoryGoals.size() <= armIndex)
		armActiveJointTrajectoryGoals.push_back(youbotArmGoal);
	else
		armActiveJointTrajectoryGoals[armIndex] = youbotArmGoal;
	armHasActiveJointTrajectoryGoals[armIndex] = true;

	// myTrace->startTrace();


	for (int i = 0; i < youBotArmDoF; ++i)
	{
		try
		{
			// youBot joints start with 1 not with 0 -> i + 1

			ROS_DEBUG("set trajectories %d", i);
		}
		catch (std::exception& e)
		{
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set trajectory for joint %i: %s", i + 1, errorMessage.c_str());
			//      youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		}
	}
	ROS_INFO("set all trajectories");
}

void YouBotArmTrajController::armJointTrajectoryCancelCallback(
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex)
{
	/*boost::mutex::scoped_lock lock(_mutex);
	ROS_DEBUG("Cancel for arm%i received", armIndex + 1);
	ROS_ASSERT(armIndex < youBotConfiguration.youBotArmConfigurations.size());

	// stop the controller
	for (int i = 0; i < youBotArmDoF; ++i)
	{
		try
		{
			// youBot joints start with 1 not with 0 -> i + 1
			//TODO cancel trajectory
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.cancelCurrentTrajectory();
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).stopJoint();
		}
		catch (std::exception& e)
		{
			std::string errorMessage = e.what();
			ROS_WARN("Cannot stop joint %i: %s", i + 1, errorMessage.c_str());
		}
	}

	if (armActiveJointTrajectoryGoals[armIndex] == youbotArmGoal)
	{
		// Marks the current goal as canceled.
		youbotArmGoal.setCanceled();
		armHasActiveJointTrajectoryGoals[armIndex] = false;
	}
	*/
	ROS_INFO("CANCEL GOAL>>>>>>>>>>>>>>>>>>>");
}

/* EOF */
