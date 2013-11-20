/*
 * YoubotCtrl.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */
#include <string>
#include "youbot_ctrl_config.h"

#include "YoubotCtrl.h"

#include <moveit_msgs/GetPositionIK.h>

namespace youbot_ctrl
{

YoubotCtrl::YoubotCtrl()
{
  init();
}

YoubotCtrl::~YoubotCtrl()
{
  // TODO Auto-generated destructor stub
}

bool
YoubotCtrl::init()
{
  // create node
  _n.reset(new ros::NodeHandle());

  // setup ik service
  _ik_client   = _n->serviceClient<moveit_msgs::GetPositionIK> (config::ik_service_name);
  int count = 0;
  while(!_ik_client.exists())
  {
    if (++count % 3 == 3)
      ROS_INFO_NAMED("YoubotCtrl","Waiting for IK service");
    sleep(0.5);
  }
  ROS_INFO_NAMED("YoubotCtrl","IK service available");

  //setup frames
  _frame_ik_end_name = config::frame_ik_end_name;
  _frame_tcp_name = config::frame_tcp_name;

  // wait for tf-tree
  ROS_INFO_NAMED("YoubotCtrl", "Waiting for TF tree to be available");
  _tf_available = _listener.waitForTransform(_frame_tcp_name,
                                             _group->getPoseReferenceFrame(),
                                             ros::Time(0),
                                             ros::Duration(10.0));
  if(!_tf_available)
  {
    ROS_INFO_NAMED("YoubotCtrl", "TF tree not available");
  }
  else
  {
    geometry_msgs::PoseStamped  pose_tcp_target,
                                pose_link_5,
                                pose_link_5_to_tcp;
    // setup pose = 0 in link_5
    pose_link_5.header.stamp    = ros::Time(0);
    pose_link_5.header.frame_id = _frame_ik_end_name;
    pose_link_5.pose.orientation.w = 1;
    // get transform from link_5 to tcp
    listener.transformPose(_frame_tcp_name,
                           pose_link_5,
                           pose_link_5_to_tcp);
    tf::poseMsgToKDL(pose_link_5_to_tcp.pose, _frame_link_5_to_tcp);
  }



  ROS_INFO("planning frame        : %s", _group->getPlanningFrame().c_str());
  ROS_INFO("getEndEffectorLink    : %s", _group->getEndEffectorLink().c_str());
  ROS_INFO("getEndEffector        : %s", _group->getEndEffector().c_str());
  ROS_INFO("getPoseReferenceFrame : %s", _group->getPoseReferenceFrame().c_str());
}


bool YoubotCtrl::isReachable(const geometry_msgs::Pose &location, bool avoid_collisions)
{
  bool res = false;

  //
  // Convert tcp_goal into link_5 goal for IK service
  //
  KDL::Frame frame_goal_tcp, frame_goal_link_5;
  geometry_msgs::Pose location_link_5;
  tf::poseMsgToKDL(location, frame_goal_tcp);
  frame_goal_link_5 = frame_goal_tcp * _frame_link_5_to_tcp;
  tf::poseKDLToMsg(frame_goal_link_5, location_link_5);

  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;
  service_request.ik_request.group_name = config::moveit_group_name;
  service_request.ik_request.pose_stamped.header.frame_id = _frame_tcp_name;
  service_request.ik_request.pose_stamped.pose = location_link_5;
  service_request.ik_request.avoid_collisions = avoid_collisions;
  if(_ik_client.call(service_request, service_response))
  {
    // process response..
  }


  return res;
}

} /* namespace youbot_ctrl */

