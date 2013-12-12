/*
 * youbot_config_node.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: raha
 */


#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <youbot_control/YoubotControlConfig.h>
#include "YoubotConfigNode.h"
#include "config.h"



namespace youbot_arm_control
{
// Init static vars common vars
int YoubotConfigNode::kfd = 0;
extern struct termios YoubotConfigNode::raw;
extern struct termios YoubotConfigNode::cooked;

using std::string;

YoubotConfigNode::YoubotConfigNode()
{
  init(youbot_arm_control::config::default_config_topic);
}


YoubotConfigNode::YoubotConfigNode(std::string topicCrtl)
{
  init(topicCrtl);
}


YoubotConfigNode::~YoubotConfigNode()
{
  std::cout << "\nDeleting YoubotConfigNode";
  tcsetattr(kfd, TCSANOW, &cooked);
  crtl_pub.shutdown();
  n.shutdown();
  std::cout << " --> Finished!" << std::endl;
}


void YoubotConfigNode::init(std::string topicCrtl)
{
  using std::stringstream;
  //make topics
  stringstream crtl, skillCrtlSrv;
  crtl << topicCrtl;
  _topicCrtl = crtl.str();
  std::cout << "topicCrtl: " <<_topicCrtl << std::endl;
  crtl_pub = n.advertise<youbot_control::YoubotControlConfig>(_topicCrtl, 1000);
}

void YoubotConfigNode::keyLoop()
{

  int8_t c; // key container

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
//  puts("Use arrow keys to navigate the skill(s).");
//  puts("Use ENTER to activate entire skill");
//  puts("Use space to _select_ a skills, or _activate_ a primitive");


  youbot_arm_control::config::CtrlOpCode cmd;
  const int bufSize = 4;
  int8_t valBuf [bufSize];
  int bufIdx = 0;
  clearBuffer(valBuf, bufSize, bufIdx);

  bool doCont = true;
  bool doSetLocal = false;
  std::string setType;
  while(doCont)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
//    printf("value: 0x%02X\n", c);
//    std::cout << "c_value: " << c << std::endl;

    cmd = youbot_arm_control::config::ERROR_CRTL;
    std::cout  << "\r\033[2K";
    switch(c)
    {
      case KEYCODE__BACK_SPACE:
        removeFromBuffer(valBuf, bufSize, bufIdx);
        std::cout   << "Current obj id: " << printBuf(valBuf, bufIdx);
      break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        addToBuffer(valBuf, c-'0', bufSize, bufIdx);
        std::cout   << "Current obj id: " << printBuf(valBuf, bufIdx);
        break;

      case'P':
      case 'p':
        std::cout   << "Planning to move to obj: " << printBuf(valBuf, bufIdx);
        cmd = youbot_arm_control::config::PLAN;
        break;

      case'E':
      case 'e':
        std::cout   << "Execute plan for obj: " << printBuf(valBuf, bufIdx);
        cmd = youbot_arm_control::config::EXECUTE_PLAN;
        break;

      case'M':
      case 'm':
        std::cout   << "Moving to obj: " << printBuf(valBuf, bufIdx);
        break;

      case'C':
      case 'c':
        std::cout   << "Moving to calibration position";
        cmd = youbot_arm_control::config::MOVE_TO_CALIB;
        break;

      case'S':
      case 's':
        std::cout   << "Show all available obj ids";
        cmd = youbot_arm_control::config::SHOW_AVAILABLE_OBJS;
        break;


      case'R':
      case 'r':
        std::cout   << "Is obj reachable?";
        cmd = youbot_arm_control::config::IS_REACHABLE_OBJ;
      break;

      case KEYCODE_ESC:
        cmd = youbot_arm_control::config::ABORT;
        std::cout << "Abort / Cancel / Reset";
      break;


      case 't':
        std::cout   << "Decrease tolerance (id 1: position, id:2 orientation)";
        cmd = youbot_arm_control::config::DECREASE_TOL;
      break;

      case 'T':
        std::cout   << "Increase tolerance (id 1: position, id:2 orientation)";
        cmd = youbot_arm_control::config::INCREASE_TOL;
      break;

      case 'O':
        std::cout   << "Search for obj";
        cmd = youbot_arm_control::config::MOVE_TO_OBJ_SEARCH;
      break;

//      case 'Z':
//        std::cout   << "Pick obj";
//        cmd = youbot_arm_control::config::PICK_OBJ;
//        break;

      case KEYCODE_ENTER:
        // do nothing on enter
      break;


      default:
        std::cout << "Unknown cmd! ";
        printf("key value: (hex) 0x%02X", c);
        std::cout << "Val: " << c;
        cmd = youbot_arm_control::config::ERROR_CRTL;
    }



    youbot_control::YoubotControlConfig ctrl;
    if (cmd != youbot_arm_control::config::ERROR_CRTL)
    {
      ctrl.command = cmd;
//      ctrl.id = printBuf(valBuf, bufIdx);
      ctrl.id = bufferToInt(valBuf, bufIdx);
      std::cout << "\tSending: cmd: " << ctrl.command << " id: " << ctrl.id;
      crtl_pub.publish(ctrl);
    }

    std::cout.flush();
    ros::spinOnce();
    usleep(100);
  }
}


}  // namespace youbot_arm_config




/******
 * Instantiation of the node
 */
youbot_arm_control::YoubotConfigNode* crtl_node;
void quit(int sig)
{
  delete crtl_node;
  ros::shutdown();
  // wait for node to shutdown..
  while(ros::ok())
    ;
  exit(sig);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SkillCrtl");
  crtl_node = new youbot_arm_control::YoubotConfigNode;

  signal(SIGINT,quit);


  crtl_node->keyLoop();

  quit(0);
//  delete crtl_node;

  return(0);
}
