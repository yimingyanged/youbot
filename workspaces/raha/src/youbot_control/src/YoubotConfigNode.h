/*
 * YoubotConfigNode.h
 *
 *  Created on: 6 Dec 2013
 *      Author: raha
 */

#ifndef YOUBOTCONFIGNODE_H_
#define YOUBOTCONFIGNODE_H_

#include <ros/ros.h>

#include <string>
#include <sstream>
#include <termios.h>

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT  0x44
#define KEYCODE_ENTER 0x0A
#define KEYCODE_SPACE 0x20
#define KEYCODE_ESC   0x1B

#define KEYCODE__BACK_SPACE 0x7f

namespace youbot_arm_control
{
class YoubotConfigNode
{
public:
  YoubotConfigNode();

  YoubotConfigNode(std::string topicCrtl);

  virtual ~YoubotConfigNode();
  void init(std::string topicCrtl);
  void keyLoop();
  void quit();
  static int kfd;
  static struct termios cooked;
  static struct termios raw;

private:

  ros::NodeHandle n;
  ros::Publisher crtl_pub;
  std::string _topicCrtl;
//  ros::ServiceClient crtl_srv_client;
//  int cmd;


  void clearBuffer(int8_t buffer[], int size, int& bufIdx)
  {
    for (int i = 0; i < size; i++) buffer[i] = 0;
    bufIdx = 0;
  }

  int addToBuffer(int8_t buffer[], int8_t val, int size, int& bufIdx)
  {
    int res = -1;
    if(bufIdx < 0)
    {
      // Buffer idx invalid, clearing buffer
      clearBuffer(buffer, size, bufIdx);
    }

    if(bufIdx >= 0 && bufIdx < size-1 && val <= 9 && val >= 0)
    {
      // add val to
      buffer[bufIdx++] = val;
      res = 1;
    }

    return res;
  }

  int removeFromBuffer(int8_t buffer[],int size,  int& bufIdx)
  {
    int res;
    if(bufIdx > 0 && bufIdx < size)
    {
      // add val to
      buffer[bufIdx--] = 0;
      res = 1;
    }

    return res;
  }

  std::string printBuf(int8_t buffer[], int idx)
  {
    std::stringstream ss;
    if(idx > 0)
    {
      for (int i = 0; i < idx; i++)
        ss << (int) buffer[i];
    }
    else
      ss << 0;
    return ss.str();
  }

  int bufferToInt(int8_t buffer[], int idx)
  {
    int res = 0;
    int count = 0;
    for (int i = idx-1; i >= 0; i--)
    {
      res += pow(10, count++)*buffer[i];
    }
    return res;
  }


};

}; // namespace youbot_arm_control
#endif /* YOUBOTCONFIGNODE_H_ */
