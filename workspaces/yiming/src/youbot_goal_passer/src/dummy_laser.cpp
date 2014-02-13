#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("base_scan", 50);

  unsigned int num_readings = 422;
  bool flag = true;
  while(n.ok()){
    //generate some fake data for our laser scan
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser";
    scan.angle_min = -1.30081570148;
    scan.angle_max = 1.29467976093;

    scan.angle_increment = 0.00613592332229;

    scan.time_increment = 9.76562514552e-05;

    scan.range_min = 0.019999999553;
    scan.range_max = 5.59999990463;


    scan.ranges.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
    	if (flag)
    	{
    		scan.ranges[i] = 4.0;
    	}
    	else
    	{
    		scan.ranges[i] = 4.1;
    	}
    }
    flag = !flag;
    scan_pub.publish(scan);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
}
