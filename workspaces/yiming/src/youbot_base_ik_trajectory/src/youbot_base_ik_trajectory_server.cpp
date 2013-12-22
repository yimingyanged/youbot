#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

class YoubotFollowJointTrajectoryAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;

public:

  YoubotFollowJointTrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&YoubotFollowJointTrajectoryAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~YoubotFollowJointTrajectoryAction(void)
  {
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_follow_joint_trajectory");

  YoubotFollowJointTrajectoryAction youbotFollowJointTrajectoryAction(ros::this_node::getName());
  ros::spin();

  return 0;
}
