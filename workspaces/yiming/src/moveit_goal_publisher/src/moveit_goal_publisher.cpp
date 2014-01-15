#include "ros/ros.h"
#include <visualization_msgs/InteractiveMarker.h>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_goal_publisher");
  ros::NodeHandle n;
  ros::Publisher goal_pub = n.advertise<visualization_msgs::InteractiveMarker>("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    visualization_msgs::InteractiveMarker goal;
    goal.markers.header.stamp=ros::Time::now();
    goal.markers.header.frame_id="odom";

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    break;
  }


  return 0;
}
