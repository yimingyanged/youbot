#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>

tf::Transform transform_gripper2camera;
std::string urdf_location=""; //location of the urdf file, read from a param
std::vector<std::string> lines; //every line in the urdf file
int whichline = 0; //which line to change

using std::cout;
using std::endl;

bool read_urdf(std::string location) {
  //open the file for read and write
  ROS_INFO_STREAM("Will open urdf at location " << location);
  std::ifstream myfile;
  myfile.open(urdf_location.c_str());
  if (myfile.is_open()) {
    ROS_INFO("Urdf is open, reading...");
    lines.clear();
    std::string line;
    int i=0;
    whichline=0;
    bool last_line_was_flag=false;

    while (myfile.good())
    {
      //push each line into the vector
      getline(myfile,line);
      lines.push_back(line);

      //if the last was the flag, check that this line has an origin tag
      if (last_line_was_flag
           && (line.find("origin"))!=std::string::npos)
      {
        whichline=i;
        ROS_INFO("Found line to edit: line %d: '%s'",i,line.c_str());
      }

      //see if the current line is the flag
      if ( !last_line_was_flag
           && (line.find("AR_CALIBRATOR:EDIT_HERE"))!=std::string::npos)
      {
        last_line_was_flag=true;
        ROS_INFO("Found calibrator start flag in urdf");
      }
      else last_line_was_flag=false;
      i++;
    } //end of file
    myfile.close();
    if (whichline==0)
    {
      ROS_ERROR("Desired line not found in urdf");
      return false;
    }
    return true;
  } //end of 'if (myfile.is_open())'
  else {
    ROS_ERROR("Error openning urdf file");
    return false;
  }
}

bool write_urdf() {
  //replace the urdf's camera pose with the calibrated pose
  double x,y,z,r,p,yaw;
  x=transform_gripper2camera.getOrigin().getX();
  y=transform_gripper2camera.getOrigin().getY();
  z=transform_gripper2camera.getOrigin().getZ();
  tf::Matrix3x3(transform_gripper2camera.getRotation()).getRPY(r,p,yaw);
  std::stringstream updated_line;
  updated_line << "                  <origin xyz=\"";
  updated_line << x << " " << y << " " << z;
  updated_line << "\" rpy=\"" << r << " " << p << " " << yaw << "\" />";
  ROS_INFO_STREAM("replacing old origin with: '" << updated_line.str() << "'");
  lines.at(whichline)=updated_line.str();

  //copy old file before writing in case something goes wrong
  ROS_INFO("Creating backup of urdf");
  std::ifstream src(urdf_location.c_str(), std::ios::binary);
  std::stringstream temp;
  temp << urdf_location << ".temp";
  std::ofstream dst(temp.str().c_str(),std::ios::binary);
  if (src.is_open() && dst.is_open()) dst << src.rdbuf();
  else {
    ROS_ERROR("Error creating backup of urdf");
    return false;
  }

  //write new urdf file with modified camera origin
  std::ofstream myfile;
  //open the file for writing, truncate because we're writing the whole thing
  myfile.open(urdf_location.c_str(), std::ios::out | std::ios::trunc);
  if (myfile.is_open()) {
    ROS_INFO("Urdf is open, writing...");
    for (std::vector<std::string>::iterator it = lines.begin(); it != lines.end(); ++it)
    {
      //write each line to the file
      if (myfile.good()) myfile<<(*it)<<std::endl;
      else
      {
        ROS_ERROR("Something went wrong with writing the urdf");
        return false;
      }
    } //end iterator
    myfile.close();
    ROS_INFO("Success");
    return true;
  } //end of 'if (myfile.is_open())'
  else {
    ROS_ERROR("Error opening the urdf file for writing");
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"ar_camera_calibrator");
  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  //get the urdf location parameter
  if (n.getParam("/ar_camera_calibrator/urdf_location", urdf_location)) {
    if (read_urdf(urdf_location))
    {
      //set the transform to the one that was read
      double x,y,z,r,p,yaw;
      char garbage1[100],garbage2[100];
      int parsed = sscanf(lines.at(whichline).c_str(),
            "%[^=]=\"%lf %lf %lf\"%[^=]=\"%lf %lf %lf",
            garbage1,&x,&y,&z,garbage2,&r,&p,&yaw);
      if (parsed<6)
      {
        ROS_ERROR_STREAM("Error parsing origin tag. Got " << parsed <<", expected 6. shutting down");
        return 0;
      }
      else {
        ROS_INFO("Read transform xyz=%lf %lf %lf  rpy=%lf %lf %lf",x,y,z,r,p,yaw);
        transform_gripper2camera.setOrigin(tf::Vector3(x,y,z));
        tf::Quaternion rot = tf::createQuaternionFromRPY(r,p,yaw);
        transform_gripper2camera.setRotation(rot.normalized());
      }
    }
    else {
      ROS_ERROR("Error reading urdf, shutting down");
      return 0;
    }
  }
  else {
    ROS_ERROR_STREAM("Error getting urdf location, quitting");
//    transform_gripper2camera.setOrigin(tf::Vector3(0.04,0,0));
//    transform_gripper2camera.setRotation(tf::createQuaternionFromRPY(0,1.57,0));
    return 0;
  }

  ros::Rate rate(10.0);

  if (n.ok()) for (int i=0; i<10; i++) rate.sleep();


  while (n.ok()) {

    //tranform from base_footprint to gripper_palm_link
    tf::Transform transform_marker2camera;
    //do this a bunch of times because something goes wrong otherwise
    //TODO: consider interpolating the transforms instead
    for (int i=0; i<30; i++) {
      bool ok = true;
      //get transform from ar marker to camera
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/ar_marker",
  		"/xtion_link",
  		ros::Time(0), //latest
  		transform);
        tf::Quaternion q;
      //q.setRPY(0,0,1.57);
        q.setRPY(0,0,0);
        tf::Transform rot;
        rot.setRotation(q);
        transform_marker2camera=rot*transform;

      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: couldn't get gripper orientation: %s",ex.what());
        ok=false;
      }

      if (ok) {
        tf_broadcaster.sendTransform(
          tf::StampedTransform(transform_marker2camera,ros::Time::now(),
          "ar_marker_fixed", "xtion_calibrated"));

        ros::spinOnce();
        rate.sleep();
      }
    }


std::string cam_mount_frame("/arm_sensor_angle_part2_link");
std::string cam_calib_frame("/xtion_calibrated");
    try {
//  cout << "Using new frame " << cam_mount_frame << " as parent for sensor mount" << endl;
      tf::StampedTransform transform;
      tf_listener.lookupTransform(
	cam_mount_frame,
	cam_calib_frame,
	ros::Time(0), //latest
	transform);
      transform_gripper2camera.setOrigin(transform.getOrigin());
      transform_gripper2camera.setRotation(transform.getRotation());
      if (!write_urdf()) {
        ROS_ERROR("Couldn't write URDF, shutting down");
      }
      else
        ROS_INFO("URDF updated with new calibration values. Please restart publisher!");
      return 0;
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("ARCameraCalibrator.cpp: couldn't get gripper to camera transform: %s",ex.what());
      ROS_INFO("The ar_marker may not have been in view, retrying");
    }
  }
  ros::spinOnce();
  rate.sleep();

}
