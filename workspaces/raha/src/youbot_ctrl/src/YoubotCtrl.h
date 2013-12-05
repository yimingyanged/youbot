/*
 * YoubotCtrl.h
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */

#ifndef YOUBOTCTRL_H_
#define YOUBOTCTRL_H_

#include <string>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include "ObjectInfo.h"

namespace youbot_arm_ctrl
{

class YoubotCtrl
{
public:
  YoubotCtrl();
  virtual
  ~YoubotCtrl();

  void
  rosThread(int threads);

  bool
  init();

  bool
  isGraspable(const ObjectInfo &obj_id, const geometry_msgs::Pose location);

  bool
  isReachable(const geometry_msgs::Pose &location, bool avoid_collisions = true);

  bool
  moveToPose(const geometry_msgs::Pose &location);

  bool
  pickObj(const ObjectInfo &objId, const geometry_msgs::Pose &location);

  bool
  placeObj(const ObjectInfo &objId, const geometry_msgs::Pose &location);

  void
  trackObjPose(const ObjectInfo &objId, const geometry_msgs::Pose &location);

  void
  searchForObj(const ObjectInfo &objId);

  // start
  void
  start();

  void
  stop();

  bool
  run();

private:


  //
  // Variables
  //

  // Ros node
  boost::shared_ptr<ros::NodeHandle> _n;
  // publishers
  //  ros::Publisher    _prim_exe_pub;
  // Service handling
  ros::ServiceClient  _ik_client;

  // Connect to a running instance of the move_group node
  boost::shared_ptr<moveit::planning_interface::MoveGroup> _group;

  // TF listener
  tf::TransformListener _tf_listener;
  bool _tf_available;

  // frame names
  std::string _frame_ik_end_name,
              _frame_tcp_name;

  // Transform between link_5 and tcp
  KDL::Frame _frame_link_5_to_tcp;

  // Stop indicated
  bool _do_run;


  void doRun(bool val);
  bool doRun();

};

} /* namespace youbot_ctrl */

#endif /* YOUBOTCTRL_H_ */
