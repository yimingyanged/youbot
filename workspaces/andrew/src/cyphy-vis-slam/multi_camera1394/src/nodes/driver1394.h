/* -*- mode: C++ -*- */
/* $Id: driver1394.h 38809 2012-02-07 19:28:15Z joq $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <multi_camera1394/RelativeTimestamp.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include "dev_camera1394.h"
#include "multi_camera1394/MultiCamera1394Config.h"
typedef multi_camera1394::MultiCamera1394Config Config;
typedef boost::shared_ptr<camera_info_manager::CameraInfoManager> CameraInfoManagerPtr;
typedef boost::shared_ptr<image_transport::ImageTransport> ImageTransportPtr;
typedef boost::shared_ptr<ros::Time> TimePtr;

/** @file

    @brief ROS driver interface for IIDC-compatible IEEE 1394 digital cameras.

*/

namespace camera1394_driver
{

class Camera1394Driver
{
public:

  // public methods
  Camera1394Driver(ros::NodeHandle node, ros::NodeHandle priv_nh);
  ~Camera1394Driver();
  void poll(void);
  void setup(void);
  void shutdown(void);

private:

  // private methods
  void closeCamera();
  bool openCamera();
  void publish(std::vector<sensor_msgs::ImagePtr> &image);
  bool read(std::vector<sensor_msgs::ImagePtr> &image);
  void reconfig(Config &newconfig, uint32_t level);
  void extractConfig(Config &newconfig);

  // Some important timing variables
  ros::Time start_;
  bool globFirstStamp_;
  int resetTimestampLoop_;
  int enableEmbeddedTimestamp_;


  /** Non-recursive mutex for serializing callbacks with device polling. */
  boost::mutex mutex_;

  /** driver state variables */
  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;         // true when reconfig() running
	bool initialising_;										// true for driver startup
	ros::NodeHandle node_;								// node handle for new relative handles
  ros::NodeHandle priv_nh_;             // private node handle
  std::vector<ros::NodeHandle> camera_nh_; // camera/s name space handle/s
  std::string camera_name_;             // camera name
  ros::Rate cycle_;                     // polling rate when closed
  uint32_t retries_;                    // count of openCamera() retries

  /** libdc1394 camera device interface */
  boost::shared_ptr<multiCamera1394::MultiCamera1394> dev_;

  /** dynamic parameter configuration */
  Config config_;
  std::vector<Config> configVec_;
  dynamic_reconfigure::Server<Config> srv_;

  /** camera calibration information */
  std::vector<CameraInfoManagerPtr> cinfo_;
  std::vector<bool> calibration_matches_;            // CameraInfo matches video mode

  /** image transport interfaces */
	std::vector<ImageTransportPtr> it_;
  std::vector<image_transport::CameraPublisher> image_pub_;
  std::vector<ros::Publisher> relative_time_pub_;//timestamp_pub_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double topic_diagnostics_min_freq_;
  double topic_diagnostics_max_freq_;
  diagnostic_updater::TopicDiagnostic topic_diagnostics_;

}; // end class Camera1394Driver

}; // end namespace camera1394_driver
