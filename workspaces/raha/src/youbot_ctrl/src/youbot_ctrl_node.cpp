/*
 * youbot_ctrl.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "YoubotCtrl.h"


boost::shared_ptr<youbot_arm_ctrl::YoubotCtrl> youbotNode;
boost::shared_ptr<boost::thread> spinThread;
boost::shared_ptr<boost::thread> logicThread;

void quit(int sig); // Forward declare

int main(int argc, char **argv) {
  using std::cout;
  using std::endl;

  // setup ROS;
  ros::init(argc, argv, "youbot_arm_control");

  int threads = 3;
  // Node creation
  cout << "creating ctrl node" << endl;
  youbotNode.reset(new youbot_arm_ctrl::YoubotCtrl());
  // Ros spinner thread(s)
  cout << "creating ros thread(s)" << endl;
  spinThread.reset(new boost::thread(boost::bind(&youbot_arm_ctrl::YoubotCtrl::rosThread, youbotNode, threads)));

  // start actual node
  cout << "starting node" << endl;
  youbotNode->start();
  youbotNode->run();



  cout << "creating ctrl object" << endl;

  return 0;
}




void quit(int sig)
{
//  youbotNode->stop();
//  std::cout << "\rQuitting!!" << std::endl;
  // wait for node to shutdown..
  youbotNode->stop();
  ros::shutdown();
//  std::cout << "Waiting for ros::ok() to finish" << std::endl;
  while(ros::ok())
    ;

//  std::cout << "Joining spinThread... id = " <<  spinThread->get_id();
//  std::flush(std::cout);
  spinThread->join();
//  std::cout << "   Done." << std::endl;
//  std::cout << "Joining logicThread... id = " <<  logicThread->get_id();
//  std::flush(std::cout);
//  logicThread->join();
//  std::cout << "   Done." << std::endl;

  // TODO: delete/stop!! boost threads!
  exit(sig);
}
