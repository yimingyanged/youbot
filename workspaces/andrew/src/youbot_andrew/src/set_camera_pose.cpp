#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <brics_actuator/JointPositions.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "set_camera_pose");

  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 10);

  /*
  rostopic pub /arm_1/arm_controller/position_command brics_actuator/JointPositions -1 '[None, None, 0.5]' '[[now, arm_joint_1, rad, 2.925]]' 
  rostopic pub /arm_1/arm_controller/position_command brics_actuator/JointPositions -1 '[None, None, 0.5]' '[[now, arm_joint_2, rad, 0.2]]'
  rostopic pub /arm_1/arm_controller/position_command brics_actuator/JointPositions -1 '[None, None, 0.5]' '[[now, arm_joint_3, rad, -1.6]]'
  rostopic pub /arm_1/arm_controller/position_command brics_actuator/JointPositions -1 '[None, None, 0.5]' '[[now, arm_joint_4, rad, 3.4]]'
  rostopic pub /arm_1/arm_controller/position_command brics_actuator/JointPositions -1 '[None, None, 0.5]' '[[now, arm_joint_5, rad, 2.93]]'
  */

  double positions[] = {2.925, 0.2, -1.6, 3.4, 2.93};
  int joints_number = sizeof(positions) / sizeof(positions[0]);

  brics_actuator::JointPositions command;
  command.poisonStamp.originator = "origin";
  command.poisonStamp.description = "desc";
  command.poisonStamp.qos = 1.0;

  vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(joints_number);
  
  std::stringstream jointName;
  for (int i = 0; i < joints_number; i++) {
    jointName.str("");
    jointName << "arm_joint_" << (i + 1); // Assamble "arm_joint_1", "arm_joint_2", ...

    armJointPositions[i].joint_uri = jointName.str();
    armJointPositions[i].value = positions[i]; // Here we set the new value.
    armJointPositions[i].unit = "rad"; // Set unit.
    armJointPositions[i].timeStamp = ros::Time::now(); // Set unit.

  };
  command.positions = armJointPositions;

  ROS_INFO("Latching for 4 seconds...");
  int count = 0;
  float freq = 100;

  ros::Rate rate(freq);
  while (node.ok() && count < int(4 / (1 / freq))) {
    pub.publish(command);
    rate.sleep();
    count++;
  }

  return 0;
}
