/*
 * youbot_ctrl_node.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */


#include <signal.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "YoubotControl.h"


boost::shared_ptr<youbot_arm_control::YoubotControl> youbotNode;
boost::shared_ptr<boost::thread> spinThread;
boost::shared_ptr<boost::thread> logicThread;

void quit(int sig); // Forward declare
void stop(int sig); // forward declare

int main(int argc, char **argv) {
  using std::cout;
  using std::endl;

  // setup ROS;
  ros::init(argc, argv, "youbot_control", ros::init_options::NoSigintHandler);
  int threads = 3;

  signal(SIGINT,stop);

  // Node creation
  ROS_INFO("creating control node");
  youbotNode.reset(new youbot_arm_control::YoubotControl());

  // ROS spinner thread(s)
  ROS_INFO("Creating %d ros thread(s)", threads);
  spinThread.reset(new boost::thread(boost::bind(&youbot_arm_control::YoubotControl::rosThread,
                                                 youbotNode,
                                                 threads)));

  ROS_INFO("Initializing youbot control");
  youbotNode->init();

  // Start actual node
  cout << "starting node" << endl;
  youbotNode->start();
  youbotNode->run();

  cout << "\nQuitting node" << endl;

  quit(0);
  return 0;
}


void stop(int sig)
{
//  std::cout << "Stopping node from sig" << std::endl;
  youbotNode->stop();
//  std::cout << "Now stopped" << std::endl;
}

void quit(int sig)
{
  //wait for node to exit;
  youbotNode->stop();
  while(youbotNode->isRunning())
    ;
  youbotNode.reset();
  //  std::cout << "Quitting!!" << std::endl;
  // wait for node to shutdown..
  ros::shutdown();
  std::cout << "Waiting for ros::ok() to finish" << std::endl;
  while(ros::ok())
    ;

  std::cout << "Joining spinThread... id = " <<  spinThread->get_id();
//  std::flush(std::cout);
//  spinThread->join();

  std::cout << " ... ->   Done." << std::endl;
//  std::cout << "Joining logicThread... id = " <<  logicThread->get_id();
//  std::flush(std::cout);
//  logicThread->join();
//  std::cout << "   Done." << std::endl;

  // TODO: delete/stop!! boost threads!
//  exit(sig);
}
