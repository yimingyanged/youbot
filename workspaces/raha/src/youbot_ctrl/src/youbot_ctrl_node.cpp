/*
 * youbot_ctrl.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */

#include "YoubotCtrl.h"

#include <iostream>
using std::cout;
using std::endl;
int main(int argc, char **argv) {

  // setup ROS;
  ros::init(argc, argv, "youbot_arm_control");

  youbot_ctrl::YoubotCtrl ctrl;

  return 0;
}




