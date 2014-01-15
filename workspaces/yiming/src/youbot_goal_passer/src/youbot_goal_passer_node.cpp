#include "youbot_goal_passer/youbot_goal_passer.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "youbot_goal_passer");
    ros::NodeHandle n;
    ROS_INFO("Starting Goal Passer Node");
    youbot_goal_passer::YoubotGoalPasser goal_passer(&n, "arm_and_base_controller/follow_joint_trajectory", "/move_group/motion_plan_request", "/move_base/result", "/arm_1/arm_controller/follow_joint_trajectory", "/move_base_simple/goal");
    ROS_INFO("Done Initializing Interface");
    ros::spin();
    return 0;
}
