/*
 * config.cpp
 *
 *  Created on: Nov 28, 2013
 *      Author: raha
 */

#include "config.h"

namespace youbot_arm_control
{

namespace config
{
const std::string default_ik_service_name   = "compute_ik";
const std::string default_frame_ik_end_name = "arm_link_5";
const std::string default_frame_tcp_name    = "gripper_tcp_link";
const std::string default_frame_cam_name    = "xtion_rgb_optical_frame";
const std::string default_moveit_group_name = "arm_1_joints";
const std::string default_ar_marker_topic   = "ar_pose_obj";
const std::string default_config_topic      = "config";
const std::string default_namespace_topic   = "youbot";

}
}
