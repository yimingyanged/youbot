//
//  run_node.cpp
//  
//
//  Created by Timothy Morris on 14/04/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#include "ros/ros.h"
#include "openfabmap2_ros.h"

using openfabmap2_ros::FABMapRun;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "run_node");
	
	// Running
	ros::NodeHandle priv_nh("~");
	double sampleRate_;
	priv_nh.param<double>("sampleRate", sampleRate_, 100);
	ros::Rate r(sampleRate_);
	
	ros::NodeHandle nh_run;
	FABMapRun oFABMap2_run(nh_run);
	
	ROS_INFO_STREAM("Node sampling rate set to: " << sampleRate_ << "Hz");
	while (nh_run.ok() && oFABMap2_run.isWorking())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}