/******************************************************************************
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef YOUBOTOODLWRAPPER_H_
#define YOUBOTOODLWRAPPER_H_

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

//System includes
#include <boost/thread.hpp>

/* ROS includes */
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include <diagnostic_msgs/DiagnosticArray.h>

#include "youbot_common/PowerBoardState.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/server/action_server.h>
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"

/* OODL includes */
#include "YouBotConfiguration.h"
#include "youbot_driver/youbot/JointTrajectoryController.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"


namespace youBot
{

/**
 * @brief Wrapper class to map ROS messages to OODL method calls for the youBot platform.
 */
class YouBotArmTrajController
{
public:

    /**
     * @brief Constructor with a ROS handle.
     * @param n ROS handle
     */
    YouBotArmTrajController(ros::NodeHandle n);

    /**
     * @brief DEfault constructor.
     */
    virtual ~YouBotArmTrajController();


    /**
     * @brief Initializes a youBot base.
     * @param armName Name of the base. Used to open the configuration file e.g. youbot-manipulator.cfg
     * @param enableStandardGripper If set to true, then the default gripper of the youBot will be initialized.
     */
    void initializeArmTraj(std::string armName, bool enableStandardGripper = true);

    /**
	 * @brief Callback that is executed when an action goal to perform a joint trajectory with the arm comes in.
	 * @param youbotArmGoal Actionlib goal that contains the trajectory.
	 * @param armIndex Index that identifies the arm
	 */
	void armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex);

	/**
	 * @brief Callback that is executed when an action goal of a joint trajectory is canceled.
	 * @param youbotArmGoal Actionlib goal that contains the trajectory.
	 * @param armIndex Index that identifies the arm
	 */
	void armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex);
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> *armJointTrajectoryAction;

private:

    YouBotArmTrajController(); //forbid default constructor
    

    /// Degrees of freedom for the youBot manipulator
    static const int youBotArmDoF = 5;

    /// Number of finger mounted on the gripper.
    static const int youBotNumberOfFingers = 2;

    /// Number of wheels attached to the base.
    static const int youBotNumberOfWheels = 4;


    std::string youBotChildFrameID;
    std::string youBotOdometryFrameID;
    std::string youBotOdometryChildFrameID;
    std::string youBotArmFrameID;


    /// The ROS node handle
    ros::NodeHandle node;

    /// ROS timestamp
    ros::Time currentTime;


    /// The published odometry message with distances in [m], angles in [RAD] and velocities in [m/s] and [RAD/s]
    nav_msgs::Odometry odometryMessage;

    /// The published odometry tf frame with distances in [m]
    geometry_msgs::TransformStamped odometryTransform;

    /// The quaternion inside the tf odometry frame with distances in [m]
    geometry_msgs::Quaternion odometryQuaternion;

    /// A tf transform by which the odometry is offset. This is used to prevent the odometry from zeroing upon reconnect.
    tf::Transform odometryOffset;
    /// The previously-recorded positions. These are needed if ethercat is dead when a reconnect is called.
    double last_x,last_y,last_theta;

    /// The published joint state of the base (wheels) with angles in [RAD] and velocities in [RAD/s]
    sensor_msgs::JointState baseJointStateMessage;

    /// Vector of the published joint states of per arm with angles in [RAD]
    vector<sensor_msgs::JointState> armJointStateMessages;

    /// The complete joint state
    sensor_msgs::JointState completeJointStateMessage;

    /// The joint trajectory goal that is currently active pr. arm.
	vector<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> armActiveJointTrajectoryGoals;

	/// Tell if a goal is currently active.
	vector<bool> armHasActiveJointTrajectoryGoals;

	/// The GripperCommand goal that is currently active.
	vector<actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle> gripperActiveGripperCommandGoals;

	/// Tell if a goal is currently active.
	vector<bool> gripperHasGripperCommandGoals;

  /// Gripper positions
	vector<youbot::GripperSensedBarPosition> gripperBar1Positions;
	vector<youbot::GripperSensedBarPosition> gripperBar2Positions;
	vector<youbot::GripperSensedBarSpacing> gripperBarSpacings;

	/// Gripper velocities
	vector<youbot::GripperSensedVelocity> gripperBar1Velocities;
  	vector<youbot::GripperSensedVelocity> gripperBar2Velocities;
	int gripperCycleCounter;

    //void executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal,  int armIndex);
    
    //bool trajectoryActionServerEnable;
    //double trajectoryVelocityGain;
    //double trajectoryPositionGain;
    double youBotDriverCycleFrequencyInHz;
        
    /// diagnostic msgs
    ros::Time lastDiagnosticPublishTime;

    ros::Publisher dashboardMessagePublisher;

    ros::Publisher diagnosticArrayPublisher;
    diagnostic_msgs::DiagnosticArray diagnosticArrayMessage;
    diagnostic_msgs::DiagnosticStatus diagnosticStatusMessage;
    std::string diagnosticNameArms;
    std::string diagnosticNameBase;


    // Mutex for controlling threaded access
    boost::mutex _mutex;
};

} // namespace youBot

#endif /* YOUBOTOODLWRAPPER_H_ */

/* EOF */
