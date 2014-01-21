#include "youbot_manipulator/youbot_manipulator.h"
int main(int argc, char** argv) {

    ros::init(argc, argv, "youbot_manipulator");
    ros::NodeHandle n;
    ROS_INFO("Starting Youbot Manipulator Node");
    youbot_manipulator::YoubotManipulator manipulator(&n, "ArmAndBase", "target_goal_pose", "/move_group/display_planned_path", true);
    ROS_INFO("Youbot Manipulator Initialized");
    ros::spin();
    return 0;
}
