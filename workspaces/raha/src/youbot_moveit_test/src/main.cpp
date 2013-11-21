#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>

#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  using std::cout;
  using std::endl;

  // Configure the ros node
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();



  // this connects to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm_joints");
  //setup frames
  std::string frame_ik_end_name("arm_link_5");
  std::string frame_tcp_name("gripper_tcp_link");
  // specify our target
  geometry_msgs::PoseStamped pose_tcp_target, pose_link_5_target, pose_link_5_to_tcp;

  ROS_INFO("planning frame        : %s", group.getPlanningFrame().c_str());
  ROS_INFO("getEndEffectorLink    : %s", group.getEndEffectorLink().c_str());
  ROS_INFO("getEndEffector        : %s", group.getEndEffector().c_str());
  ROS_INFO("getPoseReferenceFrame : %s", group.getPoseReferenceFrame().c_str());
  //
  // Start
  //
  // orientation
  pose_tcp_target.header.stamp = ros::Time(0);
//  goalState.header.frame_id = frame_ik_end_name;
  pose_tcp_target.header.frame_id = group.getPoseReferenceFrame();
  pose_tcp_target.pose.orientation.x = 0.012;
  pose_tcp_target.pose.orientation.y =  -0.760;
  pose_tcp_target.pose.orientation.z = 0.014;
  pose_tcp_target.pose.orientation.w = 0.650;
  // position
  pose_tcp_target.pose.position.x = -0.148;
  pose_tcp_target.pose.position.y = -0.012;
  pose_tcp_target.pose.position.z = 0.347;
//  link_5_to_tcp.header.stamp = ros::Time(0);
//  link_5_to_tcp.header.frame_id = frame_ik_end_name;

  pose_link_5_target.header.stamp    = ros::Time(0);
  pose_link_5_target.header.frame_id = frame_ik_end_name;
  pose_link_5_target.pose.orientation.w = 1;

  tf::TransformListener listener;
  tf::StampedTransform transform;


//  make IK calculations for link5;
//  if succes -> success = plan.;
//  if succes -> success = execute.;

  //
  // Target
  //
  // orientation
//  target.orientation.x = 0.0274181;
//  target.orientation.y = 0.0549;
//  target.orientation.z = 0.121021;
//  target.orientation.w = 0.990751;
  // position
//  target.position.x = -0.0;
//  target.position.y = 0.0579093;
//  target.position.z = 0.383419;

//  std::cout << "sending pose: " << startState << std::endl;
////  group.setPoseReferenceFrame("arm_link_0");
////  group.setPlanningTime(2);
////  group.setStartState(startState);
//  group.setPoseTarget(startState, "arm_link_5");
  // plan the motion and then move the group to the sampled target
//  group.move();

  ros::Rate d(10);

  bool stat = false;
  int count = 0;
  int tmp = 0;
  std::vector<geometry_msgs::PoseStamped> validPoses;
  move_group_interface::MoveGroup::Plan plan;

  stat = listener.waitForTransform(frame_tcp_name,
                                   group.getPoseReferenceFrame(),
                                   ros::Time(0),
                                   ros::Duration(10.0));
  if (!stat)
    ROS_ERROR("TF can't transform between %s and %s",
              frame_tcp_name.c_str(),
              group.getPoseReferenceFrame().c_str());


  KDL::Frame frame_goal_link_5, frame_tcp_to_link_5, frame_goal_tcp;
  while(ros::ok() && stat)
  {

    try
    {
      // transform to correct frame (from tcp to tcp)
//      listener.transformPose(frame_tcp, pose_link_5_target, target);
      //  pose_link_5_target is the actual link_5 frame (eg. x, y, z = 0, q.w = 1)

      listener.transformPose(frame_tcp_name, pose_link_5_target, pose_link_5_to_tcp);
//      listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
      cout << "Original pose     : " << pose_link_5_target << "\n";
      cout << "pose_link_5_to_tcp: " << pose_link_5_to_tcp    << "\n";
      cout << "pose_tcp_target   : " << pose_tcp_target    << "\n";

      tf::poseMsgToKDL(pose_tcp_target.pose, frame_goal_tcp);
      tf::poseMsgToKDL(pose_link_5_to_tcp.pose, frame_tcp_to_link_5);
      frame_goal_link_5 = frame_goal_tcp * frame_tcp_to_link_5;
      // set target pose
      tf::poseKDLToMsg(frame_goal_link_5, pose_link_5_target.pose);
      pose_link_5_target.header.frame_id = group.getPoseReferenceFrame();
      cout << "pose_link_5_target: " << pose_link_5_target << "\n";
      cout << "pose_tcp_target: " << pose_tcp_target << "\n";
      group.setPoseTarget(pose_link_5_target);
      // plan
      stat = group.plan(plan);
      if (stat)
        // execute plan
        stat = group.execute(plan);
      cout << "Moving to target num " << count << " "
          << (stat ? "succeeded" : "failed") << endl;
      if (stat) validPoses.push_back(pose_tcp_target);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    // update pose for next iteration
    count++;
    int mod = count % 1;
    //    cout << "modulus: " << mod << " <== " << count << " % 3" << endl;
    switch (mod)
    {
      case 0:
        pose_link_5_target.pose.position.x += 0.01;
        break;
      case 1:
        //        startState.position.y += 0.01;

        break;
      case 2:
        //        startState.position.z += 0.01;
        if(pose_link_5_target.pose.position.z>1)
          pose_link_5_target.pose.position.z = 0.3;

        break;
      default:
        cout << "invalid modules.." << endl;
        break;
    }

    //    d.sleep();
    break;
  }

  cout << "Valid poses found in this run " << validPoses.size() << endl;
  for(int i = 0; i<validPoses.size();i++)
  {
    cout << "<---- " << i << " :" << validPoses.at(i) << "-->\n";
  }

  return 0;
}
