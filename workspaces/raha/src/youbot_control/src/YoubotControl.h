/*
 * YoubotControl.h
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */

#ifndef YOUBOTCONTROL_H_
#define YOUBOTCONTROL_H_

#include <string>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ar_tools_msgs/ARMarker.h>
#include <ar_tools_msgs/ARMarkers.h>
#include <youbot_control/YoubotControlConfig.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>

#include <boost/thread/mutex.hpp>

#include "ObjectInfo.h"


namespace youbot_arm_control
{

void callback();
class YoubotControl
{
public:
  YoubotControl();
  virtual
  ~YoubotControl();

  /***
   * Should be called before init!
   * @param threads
   */
  void
  rosThread(int threads);

  bool
  init();

  bool
  isGraspable(const ObjectInfo &obj_id, const geometry_msgs::Pose location);


  bool isReachable(int objId);

  bool
  isReachable(const geometry_msgs::PoseStamped &location, bool avoid_collisions = true);

  bool
  moveTCPToPose(const geometry_msgs::PoseStamped &location);

  bool
  moveTCPToPosition(const geometry_msgs::Point &location);

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

  bool
  isRunning();
private:


  //
  // Variables
  //

  // Ros node
  boost::shared_ptr<ros::NodeHandle> _n;
  // publishers
  ros::Publisher    _status_pub;
  ros::Publisher    _cam_pose_pub,
                    _ik_pose_pub,
                    _tcp_pose_pub,
                    _ik_offset_pose_pub;

  // the action client for the gripper controller
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *_acGripper;
  // subscribers
  ros::Subscriber   _marker_pose_sub;
  ros::Subscriber   _config_sub;
  // Service handling
  ros::ServiceClient  _ik_client;

  // Connect to a running instance of the move_group node
  boost::shared_ptr<moveit::planning_interface::MoveGroup> _group;
  moveit::planning_interface::MoveGroup::Plan _plan;
  bool _is_plan_valid;
  // TF listener
  tf::TransformListener _tf_listener;
  bool _tf_available;

  // frame names
  std::string _frame_ik_end_name,
              _frame_tcp_name,
              _frame_cam_name,
              _frame_ref_name;

  // Transform between link_5 and tcp
  KDL::Frame _frame_link_5_to_tcp;

  // Cached transform between cam and tcp
  KDL::Frame _frame_cam_to_tcp;

  double _position_tolerance,
         _orientation_tolerance;
  double _plan_time;

  // Stop indicated
  bool _do_run;

  //is running flag
  bool _is_running;

  std::map<int, geometry_msgs::PoseStamped> _obj_map;


  // Has new cmd?
  bool _has_new_cmd,
       _has_new_obj_pose;

  double _max_gripper_travel;
  // stored command
  youbot_control::YoubotControlConfig _command;

  // mutex for thread control
  boost::mutex _mutex,
               _mutexRun,
               _mutexPose;

  //
  // Control functions
  //
  void
  doRun(bool val);

  bool
  doRun();

  void
  cleanUp();

  //
  // getters and setters
  //

  double
  getMaxGripperTravelDistance();

  void
  setMaxGripperTravelDistance(double dist = 0.0115);

  control_msgs::GripperCommandGoal
  getOpenGripperGoal();

  control_msgs::GripperCommandGoal
  getCloseGripperGoal();

  bool openGripper();
  bool closeGripper();

  void
  setObjPose(geometry_msgs::PoseStamped camPose, int objId);

  bool
  getObjPose(int objId,
             geometry_msgs::PoseStamped *poseOut,
             bool doDeleteIfFound = true);

  std::vector<int> getAvailableObjs();

  bool
  getPlan(moveit::planning_interface::MoveGroup::Plan *plan);

  void
  updateTolerace();

  bool
  plan(int objId);

  bool
  execute();

  void
  setPlan(moveit::planning_interface::MoveGroup::Plan &plan);

  bool
  hasNewObjPose();

  void
  setNewCommand(const youbot_control::YoubotControlConfig &msg);

  youbot_control::YoubotControlConfig
  getNewCommand();

  bool
  hasNewCommand();

  void
  setIsRunning(bool val);

  void
  setPositionTolerance(double tol);

  void
  setOrientationTolerance(double tol);

  double
  getPositionTolerance();

  double
  getOrientationTolerance();


  double
  getPlanTime();

  void
  setPlanTime(double time);

  KDL::Frame
  getLink5ToTCPFrame();

  void
  setLink5ToTCPFrame(KDL::Frame &frame);

  KDL::Frame
  getCAMToTCPFrame();

  void
  setCAMToTCPFrame(KDL::Frame &frame);

  std::string getFrameTCPName();
  void setFrameTCPName(std::string name);
  std::string getFrameIKName();
  void setFrameIKName(std::string name);
  std::string getFrameCamName();
  void setFrameCamName(std::string name);
  void setFrameREFName(std::string name);
  std::string getFrameREFName();
  /**
   * Convert tcp_goal into link_5 goal for IK
   * (using cached transform between link5 and tcp)
   * @param location
   * @return
   */
  geometry_msgs::PoseStamped
  convertTCPPoseToIKPose(const geometry_msgs::Pose& location);

  /**
   * See convertTCPPoseToIKPose(const geometry_msgs::PoseStamped& location);
   * This one taking a stamped pose, asserting the ref_frame is correct.
   * @param location
   * @return
   */

  geometry_msgs::PoseStamped
  convertTCPPoseToIKPose(const geometry_msgs::PoseStamped& location);

//  KDL::Frame
//  convertTCPPoseToIKPose(const geometry_msgs::PoseStamped& location);

  geometry_msgs::PoseStamped
  convertCAMPoseToTCPPose(const geometry_msgs::PoseStamped& location);

  /**
   * General purpose transform of arbitrary pose to IKpose
   * @param location
   * @return
   */

  geometry_msgs::PoseStamped
  convertPoseToIKPose(const geometry_msgs::PoseStamped& location);

  geometry_msgs::PoseStamped
  addOffsetToPose(const geometry_msgs::PoseStamped& pose, double offset = -0.1);

  void ARCallback(const ar_tools_msgs::ARMarkersConstPtr &msg);
  void configCallback(const youbot_control::YoubotControlConfigConstPtr &msg);

  bool
  moveToCalib();

  bool
  moveToObjSearch();
};


} /* namespace youbot_arm_control */

#endif /* YOUBOTCONTROL_H_ */
