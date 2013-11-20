/*
 * youbot_ctrl_config.h
 *
 *  Created on: Nov 20, 2013
 *      Author: raha
 */

#ifndef YOUBOT_CTRL_CONFIG_H_
#define YOUBOT_CTRL_CONFIG_H_

#include <string>

namespace youbot_ctrl
{
namespace config
{

  const std::string ik_service_name   = "compute_ik";
  const std::string frame_ik_end_name = "arm_link_5";
  const std::string frame_tcp_name    = "gripper_tcp_link";
  const std::string moveit_group_name = "arm_joints";



} /* namespace config */
} /* namespace youbot_ctrl */


#endif /* YOUBOT_CTRL_CONFIG_H_ */

