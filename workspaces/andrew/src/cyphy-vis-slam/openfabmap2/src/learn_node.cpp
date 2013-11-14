//
//  learn_node.cpp
//  
//
//  Created by Timothy Morris on 14/04/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "ros/ros.h"
#include "openfabmap2_ros.h"

using openfabmap2_ros::FABMapLearn;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_node");
	
	// Learning
	ros::NodeHandle priv_nh("~");	
	double sampleRate_;
	priv_nh.param<double>("sampleRate", sampleRate_, 100); 
	ros::Rate r(sampleRate_);
	
	ros::NodeHandle nh_learn;
	FABMapLearn oFABMap2_learn(nh_learn);
	
	ROS_INFO_STREAM("Node sampling rate set to: " << sampleRate_ << "Hz");
	while (nh_learn.ok() && oFABMap2_learn.isWorking())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	ROS_INFO("Node closed.........");
	
	return 0;
}