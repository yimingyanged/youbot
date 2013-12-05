/*
 * YoubotControl.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */
#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <kdl_conversions/kdl_msg.h>
#include <moveit_msgs/GetPositionIK.h>
#include <kdl/frames.hpp>

#include <ar_tools_msgs/ARMarker.h>
#include <ar_tools_msgs/ARMarkers.h>


#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread.hpp>

#include "config.h"
#include "YoubotControl.h"

namespace youbot_arm_control
{

YoubotControl::YoubotControl()
{
  stop();
//  init();
}

YoubotControl::~YoubotControl()
{
  cleanUp();
  // TODO Auto-generated destructor stub
}

void
YoubotControl::rosThread(int threads)
{
  ros::AsyncSpinner spinner(threads); // Use threads
  spinner.start();
  ros::waitForShutdown();

//  std::cout << "rosThread stopped" << std::endl;
//  quit();
}

bool
YoubotControl::init()
{
  setIsRunning(false);
  // create node
  _n.reset(new ros::NodeHandle());

  // setup ik service
  ROS_INFO_STREAM_NAMED("YoubotControl", "Setting up IK service client @ "
                        << config::default_ik_service_name);
  _ik_client = _n->serviceClient<moveit_msgs::GetPositionIK>(
      config::default_ik_service_name);

  // subscribe to _marker_pose (objec pose)
  _marker_pose_sub = _n->subscribe<ar_tools_msgs::ARMarker>(
      config::default_ar_marker_topic, 1, &YoubotControl::ARCallback, this);

  //subscribe to configuration cmds
  _config_sub = _n->subscribe<std_msgs::Int32>(
      config::default_config_topic, 1, &YoubotControl::configCallback, this);

  _tcp_pose_pub = _n->advertise<geometry_msgs::PoseStamped>("tcp_pose_stamped", 1);
  _ik_pose_pub  = _n->advertise<geometry_msgs::PoseStamped>("ik_pose_stamped", 1);
  _cam_pose_pub = _n->advertise<geometry_msgs::PoseStamped>("cam_pose_stamped", 1);
  _ik_offset_pose_pub = _n->advertise<geometry_msgs::PoseStamped>("ik_offset_pose_stamped", 1);

  // Connects to a running instance of the move_group node
  ROS_INFO_NAMED("YoubotControl", "Connecting to moveit..");
  try {
    _group.reset(new move_group_interface::MoveGroup(
          config::default_moveit_group_name));
  } catch (std::exception &e) {
    ROS_ERROR("Failed to contact moveit, shutting down!");
    stop();
    return false;
  }
  double tol = 0.05;
  double planTime = 1.5;
  ROS_INFO_STREAM("Setting tolerance to " << tol);
  setPoseTolerance(tol);
  setPlanTime(planTime);
   _group->setPlanningTime(planTime);
   _group->setGoalTolerance(tol);
  //setup frame names
  setFrameIKName(config::default_frame_ik_end_name);
  setFrameTCPName(config::default_frame_tcp_name);
  setFrameCamName(config::default_frame_cam_name);
  setFrameREFName(_group->getPoseReferenceFrame());

  // wait for tf-tree
  ROS_INFO_NAMED("YoubotControl", "Waiting for TF tree to be available.");
  _tf_available = _tf_listener.waitForTransform(getFrameTCPName(),
                                                getFrameREFName(),
                                                ros::Time(0),
                                                ros::Duration(10.0));
  if (!_tf_available)
  {
    ROS_INFO_NAMED("YoubotControl", "TF tree not available");
  }
  else
  {
    KDL::Frame tmp;
    geometry_msgs::PoseStamped  pose_link_5,
                                pose_link_5_to_tcp,
                                pose_cam,
                                pose_cam_to_tcp;
    // setup pose = 0 in link_5
    pose_link_5.header.stamp = ros::Time(0);
    pose_link_5.header.frame_id = getFrameIKName();
    pose_link_5.pose.orientation.w = 1;
    // get transform from link_5 to tcp
    _tf_listener.transformPose(getFrameTCPName(),
                               pose_link_5,
                               pose_link_5_to_tcp);
    tf::poseMsgToKDL(pose_link_5_to_tcp.pose, tmp);
    setLink5ToTCPFrame(tmp);

    pose_cam.header.stamp = ros::Time(0);
    pose_cam.header.frame_id = getFrameTCPName();
    pose_cam.pose.orientation.w = 1;

    // get transform from cam to tcp
    _tf_listener.transformPose(getFrameTCPName(),
                               pose_cam,
                               pose_cam_to_tcp);
    tf::poseMsgToKDL(pose_cam_to_tcp.pose, tmp);
    setCAMToTCPFrame(tmp);
    // Cam to tcp cache
  }

  ROS_INFO("planning frame        : %s", _group->getPlanningFrame().c_str());
  ROS_INFO("getEndEffectorLink    : %s", _group->getEndEffectorLink().c_str());
  ROS_INFO("getEndEffector        : %s", _group->getEndEffector().c_str());
  ROS_INFO("getPoseReferenceFrame : %s",
           _group->getPoseReferenceFrame().c_str());


  // wait for ik service
//  int count = 0;
//  while (!_ik_client.exists())
//  {
//    if (++count % 3 == 3)
//      ROS_INFO_NAMED("YoubotControl",
//                     "Waiting for IK service, is MoveIt running?");
//    sleep(0.5);
//  }
//  ROS_INFO_NAMED("YoubotControl", "IK service available");


//  start();
  return true;
}


/* ***************************************
 *              CALLBACKS
 * ***************************************/

void
YoubotControl::ARCallback(const ar_tools_msgs::ARMarkerConstPtr &msg)
{
  if(!isRunning()) return;
//  ROS_INFO_STREAM("---------------------------");
  ROS_DEBUG_STREAM("---- ARmarker received ----");
                  //<< "\nPose:\n" << msg->pose.pose);
  geometry_msgs::PoseStamped goal_cam;
  goal_cam.pose = msg->pose.pose;
  goal_cam.header = msg->header;
  goal_cam.header.stamp = ros::Time(0);

  setCamPose(goal_cam);

  return;
}

std::string
YoubotControl::getFrameCamName()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _frame_cam_name;
}

void
YoubotControl::setFrameCamName(std::string name)
{
  boost::mutex::scoped_lock lock(_mutex);
  _frame_cam_name = name;
}

std::string
YoubotControl::getFrameREFName()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _frame_ref_name;
}

void
YoubotControl::setFrameREFName(std::string name)
{
  boost::mutex::scoped_lock lock(_mutex);
  _frame_ref_name = name;
}

std::string
YoubotControl::getFrameTCPName()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _frame_tcp_name;
}

void
YoubotControl::setFrameTCPName(std::string name)
{
  boost::mutex::scoped_lock lock(_mutex);
  _frame_tcp_name = name;
}

std::string
YoubotControl::getFrameIKName()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _frame_ik_end_name;
}

void
YoubotControl::setFrameIKName(std::string name)
{
  boost::mutex::scoped_lock lock(_mutex);
  _frame_ik_end_name = name;
}

void
YoubotControl::configCallback(const std_msgs::Int32ConstPtr &msg)
{

  ROS_INFO("In config callback");
}

/* ***************************************
 *             Helper functions
 * ***************************************/
geometry_msgs::PoseStamped
YoubotControl::convertTCPPoseToIKPose(
    const geometry_msgs::Pose& TCPPose)
{
  KDL::Frame frame_goal_tcp, frame_goal_link_5;
  geometry_msgs::PoseStamped location_link_5;
  tf::poseMsgToKDL(TCPPose, frame_goal_tcp);
  frame_goal_link_5 = frame_goal_tcp * getLink5ToTCPFrame();
  tf::poseKDLToMsg(frame_goal_link_5, location_link_5.pose);
  //  location_link_5.header = location.header;
//  location_link_5.header.frame_id = config::default_frame_ik_end_name;
  return location_link_5;
}

geometry_msgs::PoseStamped
YoubotControl::addOffsetToPose(
    const geometry_msgs::PoseStamped& pose, double offset)
{
  KDL::Frame frame_pose, frame_pose_offset, frame_offset;
  frame_offset.p.z(offset);
  geometry_msgs::PoseStamped pose_offset;
  tf::poseMsgToKDL(pose.pose, frame_pose);
  frame_pose_offset = frame_pose * frame_offset;
  tf::poseKDLToMsg(frame_pose_offset, pose_offset.pose);
  pose_offset.header = pose.header;
  return pose_offset;
}
//KDL::Frame
//YoubotControl::convertTCPPoseToIKPose(
//    const geometry_msgs::PoseStamped& TCPPose)
//{
//  KDL::Frame frame_goal_tcp, frame_goal_link_5;
//  tf::poseMsgToKDL(TCPPose.pose, frame_goal_tcp);
//  frame_goal_link_5 = frame_goal_tcp * getLink5ToTCPFrame();
//  //  location_link_5.header = location.header;
//  return frame_goal_link_5;
//}

geometry_msgs::PoseStamped
YoubotControl::convertTCPPoseToIKPose(
    const geometry_msgs::PoseStamped& pose_tcp)
{
  geometry_msgs::PoseStamped location_link_5;
  if(pose_tcp.header.frame_id == getFrameREFName())
  {
    location_link_5 = convertTCPPoseToIKPose(pose_tcp.pose);
    location_link_5.header = pose_tcp.header;
  }
  else
    ROS_WARN_STREAM("Wrong pose passed to " << BOOST_CURRENT_FUNCTION << " returning invalid pose");
  return location_link_5;
}

geometry_msgs::PoseStamped
YoubotControl::convertCAMPoseToTCPPose(
    const geometry_msgs::PoseStamped& CAMPose)
{
  KDL::Frame frame_goal_cam, frame_goal_tcp;
  geometry_msgs::PoseStamped location_tcp;
  KDL::Frame frame(frame_goal_cam.M.RotY(3.14));

  tf::poseMsgToKDL(CAMPose.pose, frame_goal_cam);
//  frame_goal_link_5 = frame_goal_cam * getCAMToTCPFrame();
  frame_goal_tcp = frame_goal_cam * frame; // rotate around Y (180 degrees)
  tf::poseKDLToMsg(frame_goal_tcp, location_tcp.pose);

  location_tcp.header = CAMPose.header;
//  location_tcp.header.frame_id = getFrameREFName();//config::default_frame_ik_end_name;
  return location_tcp;
}

geometry_msgs::PoseStamped
YoubotControl::convertPoseToIKPose(const geometry_msgs::PoseStamped& location)
{
  // setup pose = 0 in link_5
  geometry_msgs::PoseStamped  pose_link_5;
//  pose_link_5.header.stamp = ros::Time(0);
//  pose_link_5.header.frame_id = _frame_ik_end_name;
//  pose_link_5.pose.orientation.w = 1;
  // get transform from link_5 to tcp
  _tf_listener.transformPose(getFrameTCPName(),
                             location,
                             pose_link_5);
  return pose_link_5;
}

bool
YoubotControl::isReachable(const geometry_msgs::PoseStamped &location,
                           bool avoid_collisions)
{
  bool res = false;

  // Convert tcp_goal into link_5 goal for IK service
//  geometry_msgs::PoseStamped location_link_5 = convertTCPPoseToIKPose(location);
  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;
  service_request.ik_request.group_name = config::default_moveit_group_name;
//  service_request.ik_request.pose_stamped.header.frame_id = _frame_tcp_name;
//  service_request.ik_request.pose_stamped = location_link_5;
  service_request.ik_request.pose_stamped = location;
  service_request.ik_request.avoid_collisions = avoid_collisions;
  if (_ik_client.call(service_request, service_response))
  {
    res = false;
    switch (service_response.error_code.val) {
      case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        ROS_INFO_STREAM("No ik found!");
        break;
      case moveit_msgs::MoveItErrorCodes::SUCCESS:
        ROS_INFO_STREAM("Ik solution found!");
        res = true;
        break;
      default:
        ROS_INFO_STREAM_NAMED("YoubotControl", "Call to IK: " << ((res) ? "Succeeded" : "Failed")
                                  << " " << service_response.error_code.val);
        break;
    }

  }

  return res;
}

/* ***************************************
 *             ARM control functions
 * ***************************************/
bool
YoubotControl::moveTCPToPose(const geometry_msgs::PoseStamped &location)
{
//  if(!isReachable(location, true))
//    return false;
//  ROS_INFO_STREAM("---- Pose is reachable ----");

  bool res = false;
  boost::mutex::scoped_lock lock(_mutex);
  move_group_interface::MoveGroup::Plan plan;

  _group->stop();
  _group->setStartStateToCurrentState();
  _group->setPoseTarget(location);

  // plan
 ROS_DEBUG("Now planning!");
 res = _group->plan(plan);

 //Execute
 if(res)
 {
   ROS_INFO_STREAM_NAMED("YoubotControl", " Plan succeeded, now moving");
   _group->execute(plan);
   ROS_INFO_STREAM_NAMED("YoubotControl", " Move finished succeeded");

 } else
   ROS_INFO_STREAM_NAMED("YoubotControl", "Plan failed");
 return res;
}

bool
YoubotControl::moveTCPToPosition(const geometry_msgs::Point &position)
{
  boost::mutex::scoped_lock lock(_mutex);
  move_group_interface::MoveGroup::Plan plan;
  bool res = false;
  _group->setPositionTarget(position.x, position.y, position.z);
  // plan
 ROS_INFO("Now planning!");
 res = _group->plan(plan);
 if(res)
 {
   ROS_INFO_STREAM_NAMED("YoubotControl", "Plan succeeded, could move but waiting!");
//   _group->execute(plan);
 } else
   ROS_INFO_STREAM_NAMED("YoubotControl", "Plan failed");

 return res;
}

/* ***************************************
 *             Thread control
 * ***************************************/

void
YoubotControl::start()
{
  doRun(true);
}

void
YoubotControl::stop()
{
  doRun(false);
  boost::mutex::scoped_lock lock(_mutexRun);
  if(_group.use_count() > 0)
    _group->stop();
}

void
YoubotControl::doRun(bool val)
{
  boost::mutex::scoped_lock lock(_mutexRun);
  _do_run = val;
}

bool
YoubotControl::doRun()
{
  boost::mutex::scoped_lock lock(_mutexRun);
  return _do_run;
}

bool
YoubotControl::isRunning()
{
  boost::mutex::scoped_lock lock(_mutexRun);
  return _is_running;
}

void
YoubotControl::setIsRunning(bool val)
{
  boost::mutex::scoped_lock lock(_mutexRun);
  _is_running = val;
}

void
YoubotControl::setCamPose(geometry_msgs::PoseStamped cam_pose)
{
  boost::mutex::scoped_lock lock(_mutexPose);
  _has_new_cam_pose = true;
  _cam_pose = cam_pose;
}

geometry_msgs::PoseStamped
YoubotControl::getCamPose()
{
  boost::mutex::scoped_lock lock(_mutexPose);
  _has_new_cam_pose = false;
  return _cam_pose;
}

bool YoubotControl::hasNewCamPose()
{
  boost::mutex::scoped_lock lock(_mutexPose);
  return _has_new_cam_pose;
}
double
YoubotControl::getPoseTolerance()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _pose_tolerance;
}

void
YoubotControl::setPoseTolerance(double tol)
{
  boost::mutex::scoped_lock lock(_mutex);
  _pose_tolerance = tol;
}

void
YoubotControl::setPlanTime(double time)
{
  boost::mutex::scoped_lock lock(_mutex);
  _plan_time = time;
}

double
YoubotControl::getPlanTime()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _plan_time;
}

KDL::Frame
YoubotControl::getLink5ToTCPFrame()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _frame_link_5_to_tcp;
}

void
YoubotControl::setLink5ToTCPFrame(KDL::Frame &frame)
{
  boost::mutex::scoped_lock lock(_mutex);
  _frame_link_5_to_tcp = frame;
}

KDL::Frame
YoubotControl::getCAMToTCPFrame()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _frame_cam_to_tcp;
}

void
YoubotControl::setCAMToTCPFrame(KDL::Frame &frame)
{
  boost::mutex::scoped_lock lock(_mutex);
  _frame_cam_to_tcp = frame;
}

bool
YoubotControl::run()
{
  setIsRunning(true);
  int count = 0;
  while (doRun() && ros::ok())
  {
    if(count++ % 20 == 0)
    {
      std::cout << ".";
      std::cout.flush();
    }

    if(hasNewCamPose())
    {
      geometry_msgs::PoseStamped goal_tcp, goal_ik, goal_ik_offset, goal_cam_tmp;
      std::string refFrameName = getFrameREFName();
      boost::mutex::scoped_lock lock(_mutex);
      _tf_listener.transformPose(refFrameName, getCamPose(), goal_cam_tmp);
      lock.unlock();

      // rotate to match
      goal_tcp = convertCAMPoseToTCPPose(goal_cam_tmp);
      // Add ik offset
      goal_ik = convertTCPPoseToIKPose(goal_tcp);
      // Add ofsfet to marker
      goal_ik_offset = addOffsetToPose(goal_ik);

      //  const geometry_msgs::PoseStamped &ikPose = convertCAMPoseToIKPose(location);
      //  ROS_INFO_STREAM("Ikpose \n" << ikPose);


      //  ROS_INFO_STREAM("Pose received \n" << goal_cam);
      //  ROS_INFO_STREAM("TCPGoal \n" << goal_tcp);
      //  ROS_INFO_STREAM("IkGoal \n" << goal_ik);
      //  if(isReachable(location, true))
      lock.lock();
      _cam_pose_pub.publish(goal_cam_tmp);
      _ik_pose_pub.publish(goal_ik);
      _ik_offset_pose_pub.publish(goal_ik_offset);
      _tcp_pose_pub.publish(goal_tcp);
      lock.unlock();
      if(!moveTCPToPose(goal_ik_offset))
        //    ;
        //  if(!moveTCPToPosition(goal_tcp.pose.position))
        ROS_INFO_STREAM("Pose not reachable");
    }
    // sleep
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  setIsRunning(false);
  return true;
}

void
YoubotControl::cleanUp()
{
  boost::mutex::scoped_lock lock(_mutex);
  _group.reset();
  _config_sub.shutdown();
  this->_ik_client.shutdown();
  this->_marker_pose_sub.shutdown();
}

} /* namespace youbot_arm_control */

