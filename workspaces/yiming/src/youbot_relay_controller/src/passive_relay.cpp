#include "youbot_relay_controller/youbot_relay_controller.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "YoubotRelayPassive");
    ros::NodeHandle n;
    ROS_INFO("Starting (Passive) Relay Node");
    youbot_relay_controller::YoubotRelayPassive relay_controller(&n, "arm_and_base_controller/follow_joint_trajectory", "/move_group/display_planned_path", "/arm_1/arm_controller/follow_joint_trajectory", "/move_base_simple/goal");
    ROS_INFO("Done Initializing Interface");
    ros::spin();
    return 0;
}
