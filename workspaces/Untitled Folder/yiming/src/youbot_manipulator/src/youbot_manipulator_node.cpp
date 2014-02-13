#include "youbot_manipulator/youbot_manipulator.h"
int main(int argc, char** argv) {

    ros::init(argc, argv, "youbot_manipulator");
    //ros::AsyncSpinner spinner(2);

    ros::NodeHandle n;
    ROS_INFO("Starting Youbot Manipulator Node");
    geometry_msgs::Pose pre_grasp_offset;
    pre_grasp_offset.position.x = 0.1;
    pre_grasp_offset.position.y = 0.0;
    pre_grasp_offset.position.z = 0.03;
    pre_grasp_offset.orientation.x = 1.0;
    pre_grasp_offset.orientation.y = 0.0;
    pre_grasp_offset.orientation.z = 0.0;
    pre_grasp_offset.orientation.w = 1.0;
    youbot_manipulator::YoubotManipulator manipulator(&n, "ArmAndBase", "target_goal_pose", "/move_group/display_planned_path", "/arm_1/gripper_controller/gripper_command", pre_grasp_offset, true);
    ROS_INFO("Youbot Manipulator Initialized");
    ros::spin();
    //spinner.start();
    return 0;
}
