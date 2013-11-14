#include <vector>
#include <iostream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  using std::cout;
  using std::endl;

  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // this connects to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm_joints");
  // specify that our target will be a random one
  geometry_msgs::Pose target, startState;

  cout << "planning frame        : " << group.getPlanningFrame() << endl;
  cout << "getEndEffectorLink    : " << group.getEndEffectorLink() << endl;
  cout << "getEndEffector        : " << group.getEndEffector() << endl;
  cout << "getPoseReferenceFrame : " << group.getPoseReferenceFrame() << endl;
  //
  // Start
  //
  // orientation
//  startState.orientation.x = 0.00913563;
//  startState.orientation.y = -0.0696261;
//  startState.orientation.z = -0.045372;
  startState.orientation.w = 1;
  // position
  startState.position.x = 0.1;
  startState.position.y = 0.1;
  startState.position.z = 0.5;


  //
  // Target
  //
  // orientation
  target.orientation.x = 0.0274181;
  target.orientation.y = 0.0549;
  target.orientation.z = 0.121021;
  target.orientation.w = 0.990751;
  // position
  target.position.x = -0.0;
  target.position.y = 0.0579093;
  target.position.z = 0.383419;

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
  std::vector<geometry_msgs::Pose> validPoses;

  move_group_interface::MoveGroup::Plan plan;
  while(ros::ok())
  {

    group.setPoseTarget(startState);
    stat = group.plan(plan);
    if(stat)
    stat = group.execute(plan);
    cout << "Moving to target num " << count << " " << (stat? "succeeded" : "failed") << endl;
    if(stat) validPoses.push_back(startState);
//    d.sleep();
//    do{
//      group.setRandomTarget();
//      stat = group.move();
//    } while(!stat && ros::ok());
//    cout << "Tried to move randomly. The move " << (stat? "succeeded" : "failed") << endl;

    // update pose for next iteration
    count++;
    int mod = count % 3;
//    cout << "modulus: " << mod << " <== " << count << " % 3" << endl;
    switch (mod)
    {
      case 0:
        startState.position.x += 0.01;
      break;
      case 1:
        startState.position.y += 0.01;
      break;
      case 2:
        startState.position.z += 0.01;

      break;
      default:
        cout << "invalid modules.." << endl;
      break;
    }

//    d.sleep();
  }

  cout << "Valid poses found in this run " << validPoses.size() << endl;
  for(int i = 0; i<validPoses.size();i++)
  {
    cout << "<---- " << i << " :" << validPoses.at(i) << "-->\n";
  }

}
