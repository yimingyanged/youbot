/*
 * youbot_ctrl_config.h
 *
 *  Created on: Nov 20, 2013
 *      Author: raha
 */

#ifndef YOUBOT_CTRL_CONFIG_H_
#define YOUBOT_CTRL_CONFIG_H_

#include <string>

namespace youbot_arm_control
{
namespace config
{

// default values
extern const std::string default_ik_service_name;
extern const std::string default_frame_ik_end_name;
extern const std::string default_frame_tcp_name;
extern const std::string default_frame_cam_name;
extern const std::string default_moveit_group_name;
extern const std::string default_ar_marker_topic;
extern const std::string default_config_topic;
extern const std::string default_namespace_topic;


//// control OpCodes
enum CtrlOpCode
{
  // commands
  IGNORE_CRTL = 0,
  PLAN = 1,
  EXECUTE_PLAN = 2,
  MOVE = 3,
  MOVE_TO_CALIB = 4,
  PICK_OBJ = 5,
  PLACE_OBJ = 6,
  INCREASE_TOL = 7,
  DECREASE_TOL = 8,
  MOVE_TO_OBJ_SEARCH = 9,
  IS_REACHABLE_OBJ = 10,
  SHOW_AVAILABLE_OBJS=11,


  ABORT = 99,
  ERROR_CRTL = 100
};

enum StatusOpCode
{
  IGNORE_RESP = 0,

  ID_LIST = 1,

  ERROR_RESP = 100
};

} /* namespace config */
} /* namespace youbot_ctrl */

#endif /* YOUBOT_CTRL_CONFIG_H_ */

