/*****************************************************************************
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


#include <YouBotOODLWrapper.h>
#include <stdlib.h>

#include <iostream>
#include <math.h>
#include <unistd.h>

#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

bool has_received_heartbeat=false;
int heartbeat_timeout=0;
const int heartbeat_timeout_max=50;
bool bad=false;

ros::Time last_heartbeat_time;

void heartbeatCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();
  
  if (!has_received_heartbeat) {
    ROS_INFO("Got first heartbeat");
  } 
  else {
    //ROS_INFO_STREAM("Got heartbeat " << heartbeat_timeout << " ticks after last one");
    double secs = (now-last_heartbeat_time).toSec();
    if (secs >= heartbeat_timeout_max*0.01)
    {
      //ROS_WARN_STREAM("Got heartbeat " << (now-last_heartbeat_time).toSec() << " seconds after last, reconnecting...");
      printf("Got heartbeat %2.5f seconds after last, reconnecting...", (now-last_heartbeat_time).toSec());
      //heartbeat is late so we have timed out already
      //call reconnect and reset state
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response res;
      //ros::service::call("/reconnect",req,res);
      //youBot->switchOnBaseMotorsCallback(req, res);
      printf("Reconnected\r\n");
      
      bad = false;
      now = ros::Time::now();
    }
    else {
      //ROS_INFO_STREAM("Got heartbeat " << (now-last_heartbeat_time).toSec() << " seconds after last");
      //printf("Got heartbeat " << (now-last_heartbeat_time).toSec() << " seconds after last");
      printf("\rGot heartbeat %2.5f seconds after last", (now-last_heartbeat_time).toSec());
      
    }
  }

  has_received_heartbeat = true;
  heartbeat_timeout = 0;
  last_heartbeat_time = now;
}

void checkHeartbeat(youBot::YouBotOODLWrapper *youBot) {
  if (bad) {
    //ROS_INFO("Stopping motors...");
    //printf("Stopping motors...");
    // Kill base motors
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    //youBot->switchOffBaseMotorsCallback(req, res);

    // Stop arm without powering down motors
    //youBot->youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmJoint(1).stopJoint();
    //youBot->youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmJoint(2).stopJoint();
      // stop the controller
    for (int armIndex = 0; armIndex < 2; armIndex++) {
  for (int i = 0; i < 5; ++i)
  {
    try
    {
      // youBot joints start with 1 not with 0 -> i + 1
      //TODO cancel trajectory
      youBot->youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).trajectoryController.cancelCurrentTrajectory();
      youBot->youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).stopJoint();
    }
    catch (std::exception& e)
    {
      std::string errorMessage = e.what();
      ROS_WARN("Cannot stop joint %i: %s", i + 1, errorMessage.c_str());
    }
  }
}

    // TODO: get current goal
    //actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal;
    //youBot->armJointTrajectoryCancelCallback(goal, 0);
    //youBot->armJointTrajectoryCancelCallback(goal, 1);
    
    //youBot->switchOffArmMotorsCallback(req, res, 0);
    //youBot->switchOffArmMotorsCallback(req, res, 1);

    //ROS_INFO("Stopped motors");
    //printf("Stopped motors");
  }

  if (has_received_heartbeat && !bad) {
    heartbeat_timeout++;
      
    if (heartbeat_timeout > heartbeat_timeout_max)
    {
      //ROS_INFO("HEARTBEAT TIMED OUT!");
      printf("HEARTBEAT TIMED OUT! Stopping motors...");
      bad = true;
    }
  }
}


int main(int argc, char **argv)
{
	int debugWait = 0;
	if(argc > 1)
	{
		debugWait = atoi(argv[1]);
		ROS_INFO("Debug wait time set to %d", debugWait);
	}
  youbot::Logger::toConsole = false;
  youbot::Logger::toFile = false;
  youbot::Logger::toROS = true;
  ros::init(argc, argv, "youbot_wrapper");
  ros::NodeHandle n;
  youBot::YouBotOODLWrapper youBot(n);
  std::vector<std::string> armNames;

  /* configuration */
  bool youBotHasBase;
  bool youBotHasArms;
  bool runEstopNetworkMonitor;
  double youBotDriverCycleFrequencyInHz;  //the driver recives commands and publishes them with a fixed frequency
  n.param("youBotHasBase", youBotHasBase, true);
  n.param("youBotHasArms", youBotHasArms, true);
  n.param("runNetworkEstopper", runEstopNetworkMonitor, false);
  n.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 50.0);

  //get the config file path from an environment variable
  char* configLocation = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (configLocation == NULL)
    throw std::runtime_error("youbot_wrapper.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

	n.param<std::string>("youBotConfigurationFilePath",
			youBot.youBotConfiguration.configurationFilePath,
			configLocation);

	n.param<std::string>("youBotBaseName",
			youBot.youBotConfiguration.baseConfiguration.baseID,
			"youbot-base");

  // Retrieve all defined arm names from the launch file params
  int i = 1;
  std::stringstream armNameParam;
  armNameParam << "youBotArmName" << i; // youBotArmName1 is first checked param... then youBotArmName2, etc.
  while (n.hasParam(armNameParam.str()))
  {
    std::string armName;
    n.getParam(armNameParam.str(), armName);
    armNames.push_back(armName);
    ROS_DEBUG("Added arm %s to list of armsNames", armNameParam.str().c_str());
    armNameParam.str("");
    armNameParam << "youBotArmName" << (++i);

  }

  ros::ServiceServer reconnectService = n.advertiseService("reconnect", &youBot::YouBotOODLWrapper::reconnectCallback,
                                                           &youBot);

  ros::Subscriber heartbeatSubscriber = n.subscribe("youbot_network_heartbeat", 1, heartbeatCallback);

  // wait for debugger
  //  ros::spinOnce();
  if(debugWait > 0)
  {
	  ROS_INFO("Waiting for remote debugger..");
	  ros::Duration(debugWait).sleep();
  }



  ROS_ASSERT((youBotHasBase == true) || (youBotHasArms == true)); // At least one should be true, otherwise nothing to be started.
  if (youBotHasBase == true)
  {
    youBot.initializeBase(youBot.youBotConfiguration.baseConfiguration.baseID);
  }

  if (youBotHasArms == true)
  {
    std::vector<std::string>::iterator armNameIter;
    for (armNameIter = armNames.begin(); armNameIter != armNames.end(); ++armNameIter)
    {
      youBot.initializeArm(*armNameIter);
      ROS_DEBUG("Added arm \"%s\" to list of armsNames", armNameParam.str().c_str());
    }
  }

  /* coordination */
  ros::Rate rate(youBotDriverCycleFrequencyInHz); //Input and output at the same time... (in Hz)
  while (n.ok())
  {
    ros::spinOnce();

    //if (runNetworkEstopper)
      checkHeartbeat(&youBot);

    youBot.computeOODLSensorReadings();
    youBot.publishOODLSensorReadings();
    youBot.publishArmAndBaseDiagnostics(2.0);    //publish only every 2 seconds
    rate.sleep();

    //automatically attempt a reconnect if ethercat dies
    if (!youBot.youBotConfiguration.isEtherCATOkay()) {
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response res;
      youBot.reconnectCallback(req,res);
    }

  }

  youBot.stop();

  return 0;
}
