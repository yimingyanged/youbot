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


//// control OpCodes
//enum CrtlOpCode: int32_t
//    {
//      IGNORE_CRTL               = 0,
//      START_OBJ_TRACKER         = 1,
//      STOP_OBJ_TRACKER          = 2,
//
//      ERROR_CRTL                = 100
//    };
//
//    enum StatusOpCode: int32_t
//    {
//      IGNORE_RESP = 0,
//
//      ERROR_RESP = 100
//    };

} /* namespace config */
} /* namespace youbot_ctrl */

#endif /* YOUBOT_CTRL_CONFIG_H_ */

