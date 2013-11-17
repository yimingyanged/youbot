// $Id: driver1394.cpp 38809 2012-02-07 19:28:15Z joq $

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
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

#include <boost/format.hpp>

#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>
#include <vector>

#include "driver1394.h"
#include "multi_camera1394/MultiCamera1394Config.h"
#include "features.h"

/** @file

@brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

This is a ROS driver for 1394 cameras, using libdc1394.  It can be
instantiated as either a node or a nodelet.  It is written with with
minimal dependencies, intended to fill a role in the ROS image
pipeline similar to the other ROS camera drivers.

@par Advertises

 - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each image.

*/

namespace camera1394_driver
{
  // some convenience typedefs
  typedef multi_camera1394::MultiCamera1394Config Config;
  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  Camera1394Driver::Camera1394Driver(ros::NodeHandle node, ros::NodeHandle priv_nh):
    state_(Driver::CLOSED),
    reconfiguring_(false),
		initialising_(true),
		node_(node),
    priv_nh_(priv_nh),
    //camera_nh_(camera_nh),
    //relative_time_pub_(1, ros::Publisher),
    camera_name_("camera"), // TODO ?
    cycle_(1.0),                        // slow poll when closed
    retries_(3),
    dev_(new multiCamera1394::MultiCamera1394()),
    srv_(priv_nh),
		//cinfo_(1, CameraInfoManagerPtr), // TODO
    calibration_matches_(1, true), // TODO
    //it_(1, ImageTransportPtr), // TODO
    diagnostics_(),
    topic_diagnostics_min_freq_(0.),
    topic_diagnostics_max_freq_(1000.),
    topic_diagnostics_("image_raw", diagnostics_, 
		       diagnostic_updater::FrequencyStatusParam
		       (&topic_diagnostics_min_freq_,
			&topic_diagnostics_max_freq_, 0.1, 10), // TODO: update which topic (cam: 0 ?)
		       diagnostic_updater::TimeStampStatusParam
		       ()),
    start_(ros::Time::now()),
    globFirstStamp_(true),
    resetTimestampLoop_(0),
    enableEmbeddedTimestamp_(0)
  {}

  Camera1394Driver::~Camera1394Driver()
  {}

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void Camera1394Driver::closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * @post diagnostics frequency parameters set
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   *   GUID configuration parameter updated
   */
  bool Camera1394Driver::openCamera()
  {
    bool success = false;

    try
      {
        if (0 == dev_->open(configVec_))
          {
					ROS_INFO_STREAM("Camera opened");
					unsigned int cameraIdx = 0;
					cinfo_.resize(configVec_.size());
					image_pub_.resize(configVec_.size());
					calibration_matches_.resize(configVec_.size(), false);
					
					// 'configVec_' has been sized to match cameras open
					for (std::vector<Config>::iterator configIter = configVec_.begin();
							 configIter != configVec_.end();
							 ++configIter)
						{
						cameraIdx = distance(configVec_.begin(), configIter);
						ROS_INFO_STREAM("Setting publishers for camera " << cameraIdx
														<< " Currently " << camera_nh_.size() << " node handles and "
														<< it_.size() << " image transport objects and "
														<< relative_time_pub_.size() << " relative time stamp objects");
						//if (cameraIdx > camera_nh_.size()-1)
						//	{
							ROS_INFO("Expanding number of node handles and publishers");
							camera_nh_.push_back(ros::NodeHandle(node_, (*configIter).frame_id));
							it_.push_back((ImageTransportPtr)new image_transport::ImageTransport(camera_nh_.at(cameraIdx)));
							relative_time_pub_.push_back((camera_nh_.at(cameraIdx)).advertise<multi_camera1394::RelativeTimestamp>("relative_timestamp", 10));
							//}
						//else if (cameraIdx == 0)
						//	{
							//ROS_INFO("Assigning to the first node handle and publishers");
							//camera_nh_.at(0) = ros::NodeHandle(node_, (*configIter).frame_id);
							//it_.at(0) = (ImageTransportPtr)new image_transport::ImageTransport(camera_nh_.at(0));
							//relative_time_pub_.at(0) = (camera_nh_.at(0)).advertise<multiCamera1394::RelativeTimestamp>("relative_timestamp", 10);
							//}
						
						cinfo_.at(cameraIdx) = (CameraInfoManagerPtr)new camera_info_manager::CameraInfoManager(camera_nh_.at(cameraIdx));
						image_pub_.at(cameraIdx) = (it_.at(cameraIdx))->advertiseCamera("image_raw", 1);
						
						camera_name_ = (*configIter).guid;
						ROS_INFO_STREAM("Updating camera " << cameraIdx 
														<< " with GUID: " << camera_name_);
						
                if (!(cinfo_.at(cameraIdx))->setCameraName((*configIter).guid))
                  {
                    // GUID is 16 hex digits, which should be valid.
                    // If not, use it for log messages anyway.
                    ROS_WARN_STREAM("[" << camera_name_
                                    << "] name not valid"
                                    << " for camera_info_manger");
                  }
            ROS_INFO_STREAM("[" << camera_name_ << "] opened: "
                            << (*configIter).video_mode << ", "
                            << (*configIter).frame_rate << " fps, "
                            << (*configIter).iso_speed << " Mb/s");
            calibration_matches_.at(cameraIdx) = true;
						}
					state_ = Driver::OPENED;
					retries_ = 0;
					success = true;
          }
      }
    catch (multiCamera1394::Exception& e)
      {
        state_ = Driver::CLOSED;    // since the open() failed
        if (retries_-- > 0)
		{
			ROS_DEBUG_STREAM("[" << camera_name_
							 << "] exception opening device (retrying): "
							 << e.what());
			return false;
		}
        else
		{
			ROS_ERROR_STREAM("[" << camera_name_
							 << "] device open failed: " << e.what());
			throw;
		}		  
      }

		// TODO: diagnostics on multiple cameras
    // update diagnostics parameters
    diagnostics_.setHardwareID(camera_name_);
    double delta = (configVec_.at(0)).frame_rate * 0.1; // allow 10% error margin of camera_0
    topic_diagnostics_min_freq_ = (configVec_.at(0)).frame_rate - delta;
    topic_diagnostics_max_freq_ = (configVec_.at(0)).frame_rate + delta;

    return success;
  }


  /** device poll */
  void Camera1394Driver::poll(void)
  {
    // Do not run concurrently with reconfig().
    //
    // The mutex lock should be sufficient, but the Linux pthreads
    // implementation does not guarantee fairness, and the reconfig()
    // callback thread generally suffers from lock starvation for many
    // seconds before getting to run.  So, we avoid acquiring the lock
    // if there is a reconfig() pending.
    bool do_sleep = true;
    if (!reconfiguring_)
      {
        boost::mutex::scoped_lock lock(mutex_);
        if (state_ == Driver::CLOSED)
          {
            openCamera();        // open with current configuration
          }
        do_sleep = (state_ == Driver::CLOSED);
        if (!do_sleep)                  // openCamera() succeeded?
          {
            // driver is open, read the next image/s still holding lock
					// TODO: should be avoiding unnamed shared pointers here...
					// TODO: these arrays should be allocated once when cameras are open
					std::vector<sensor_msgs::ImagePtr> image;
					//std::vector<TimePtr> bus_stamp;
					for (int i = 0; i < configVec_.size(); i++)
						{
						image.push_back(sensor_msgs::ImagePtr(new sensor_msgs::Image));
						//bus_stamp.push_back(TimePtr(new ros::Time));
						}
          
					ROS_DEBUG("ATTEMPTING to read images");
            if (read(image))
              {
							ROS_DEBUG("Read successful, ATTEMPTING to publish");
                publish(image);
							ROS_DEBUG("Post publish");
              }
          }
      } // release mutex lock

    // Always run the diagnostics updater: no lock required.
    diagnostics_.update();

    if (do_sleep)
      {
        // device was closed or poll is not running, sleeping avoids
        // busy wait (DO NOT hold the lock while sleeping)
        cycle_.sleep();
      }
  }

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void Camera1394Driver::publish(std::vector<sensor_msgs::ImagePtr> &image)
  {
  for (std::vector<sensor_msgs::ImagePtr>::iterator imageIter = image.begin();
       imageIter != image.end();
       ++imageIter)
    {
    unsigned int cameraIdx = distance(image.begin(), imageIter);
    
    (*imageIter)->header.frame_id = (configVec_.at(cameraIdx)).frame_id;

    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo((cinfo_.at(cameraIdx))->getCameraInfo()));

    // check whether CameraInfo matches current video mode
    if (!dev_->checkCameraInfo(**imageIter, *ci))
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches_.at(cameraIdx))
          {
            // warn user once
            calibration_matches_.at(cameraIdx) = false;
            ROS_WARN_STREAM("[" << dev_->device_id_.at(cameraIdx)
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
        ci.reset(new sensor_msgs::CameraInfo());
        ci->height = (*imageIter)->height;
        ci->width = (*imageIter)->width;
      }
    else if (!calibration_matches_.at(cameraIdx))
      {
        // calibration OK now
        calibration_matches_.at(cameraIdx) = true;
        ROS_WARN_STREAM("[" << dev_->device_id_.at(cameraIdx)
                        << "] calibration matches video mode now");
      }

    // fill in operational parameters
    dev_->setOperationalParameters(*ci); // TODO: per camera

		// Assume first camera is the reference frame for the set
    ci->header.frame_id = (configVec_.at(0)).frame_id;
    ci->header.stamp = (*imageIter)->header.stamp;

    // Publish via image_transport
    (image_pub_.at(cameraIdx)).publish(*imageIter, ci);
    
    // Package and publish the 'relative_timestamp' message
    //multiCamera1394::RelativeTimestamp timemsg;
    
    // Ros Time
		//timemsg.ros_stamp = (*imageIter)->header.stamp;

    // Bus Time
		//timemsg.camera_stamp = *bus_stamp.at(cameraIdx);
		
		// Header info
    //timemsg.header.stamp = ros::Time::now();
    //timemsg.header.frame_id = (*imageIter)->header.frame_id;
	
    //(relative_time_pub_.at(cameraIdx)).publish(timemsg);

    // Diagnostic tracking for first camera of set
    if (cameraIdx == 0)
      {
      // Notify diagnostics that a message has been published. That will
      // generate a warning if messages are not published at nearly the
      // configured frame_rate.
      topic_diagnostics_.tick((*imageIter)->header.stamp);
      }
    }
  }

  /** Read camera data.
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool Camera1394Driver::read(std::vector<sensor_msgs::ImagePtr> &image)
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        dev_->readData(image);
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (multiCamera1394::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }
  
  /** String parsing for multi camera configurations
   *
   *  Called during first 'reconfig'.
   *
   *  @param newconfig new Config values 
   *              (containing strings specified at launch)
   *
   *  PRE: 'newconfig' contains values (default or user specified)
   *        in string members -> 'multi_*' (guid, frame_rate, auto_gain, auto_shutter)
   *  POST: 'this.configVec_' holds an array of configs.
   *        -Values from various string members will be paired according to their
   *        sequential order.
   *        -Missing values will be filled with defaults that can 
   *          be user overridden (i.e. non -> 'multi_*' values).
   **/
  void Camera1394Driver::extractConfig(Config &newconfig)
  {
	  // If no individual camera settings are found
	  if (true)//newconfig.stereo_pair)
	  {
			// TODO: Get GUID's from list
			std::string guid_0_, guid_1_;
			ros::NodeHandle tmpPriv_nh_("~");
			tmpPriv_nh_.param<std::string>("guid_0", guid_0_, "00b09d0100acc96d");
			tmpPriv_nh_.param<std::string>("guid_1", guid_1_, "00b09d0100acc969");
			
		  configVec_.clear();
		  configVec_.push_back(newconfig);
		  configVec_.push_back(newconfig);
		  
		  configVec_.at(0).camera_select = 0;
		  configVec_.at(0).guid = guid_0_;//"00b09d0100acc96d";
		  
		  configVec_.at(1).camera_select = 1;
		  configVec_.at(1).guid = guid_1_;//"00b09d0100acc969";
	  }
	  else
	  {
		  // resolve frame ID using tf_prefix parameter
		  if (newconfig.frame_id == "")
			  newconfig.frame_id = "camera";
		  std::string tf_prefix = tf::getPrefixParam(priv_nh_);
		  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
		  newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);
		  
		  // Construct only a single config for all cameras
		  configVec_.clear();
		  configVec_.push_back(newconfig);
		  //configVec_.at(0) = newconfig;
		  
		  // NOTE: if 'newconfig' had 'guid' set, only that camera will load
	  }
  
  // TODO: Finish this...
  /*
   //// Do this for each 'field' containing CSV settings
  while (std::getline(currentField, currentValue, ','))
    {
    
    }
   */
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void Camera1394Driver::reconfig(Config &newconfig, uint32_t level)
  {	
	ROS_DEBUG_STREAM("Attempting to 'reconfig' -> 'newconfig': " 
									<< newconfig.camera_select
									<< " 'config': "
									<< config_.camera_select);
    //// Initialise multiple config if required 
    // Initial: No camera selected (default:-1)
    if (newconfig.camera_select == -1)
      {
			ROS_WARN("No camera CURRENTLY selected for configuration");
			// IF the previous camera selected was invalid OR initialising phase
			if (config_.camera_select == -1 || initialising_)
				{
				ROS_WARN("No camera PREVIOUSLY selected for configuration");
				ROS_INFO("Selecting the first camera by default");
				// No camera is currently selected (default) for configuration
				// Select the first for dynamic reconfigure
				newconfig.camera_select = 0;
				config_ = newconfig;
				
				ROS_DEBUG("Pre multi config extraction");
				// Merge single value config with any CSV config provided
				// (populates 'configVec_')
				extractConfig(newconfig);
				ROS_DEBUG("Post multi config extraction");
				
				// Display config of the camera selected (default:0)
				//config_ = configVec_.at(newconfig.camera_select);
				
				// ???????
				//newconfig = config_;
				}
			else
				{
				ROS_INFO("Previous config was valid, reverting...");
				// revert to the previous configuration
				newconfig.camera_select = config_.camera_select;								
				}
      }
    
    // Do not run concurrently with poll().  Tell it to stop running,
    // and wait on the lock until it does.
    reconfiguring_ = true;
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

	  // Exception is thrown if 'state_' remains closed for maximum
	  // 'retries_' of openCamera()
    while (state_ == Driver::CLOSED)
      {
		  // open with new values
		  //openCamera(newconfig);
		  // NOTE: If 'configVec_.size()==1' and no 'guid' is specified
		  //    all cameras will be opened with the same config
		  openCamera();
      }
	  ROS_INFO_STREAM("Cameras opened, now updating "
					  << (dev_->device_id_).size()
					  << " cameras with "
					  << configVec_.size()
					  << " configurations.");
    
  
    // set config for each camera opened
    for (std::vector<Config>::iterator configIter = configVec_.begin();
       configIter != configVec_.end();
       ++configIter)
    {
		
		// Initialisation requires the application of all configs
		if (initialising_)
			{
			newconfig = *configIter;
			config_ = newconfig;
			}
    
    // If the current config is the one to be updated
    // OR this is the first reconfig..
    if ((*configIter).camera_select == newconfig.camera_select || initialising_)
      {
			ROS_DEBUG_STREAM("Found selected camera to configure OR initialising: " << initialising_);
      // If we are configuring a different camera
      if (config_.camera_select != newconfig.camera_select)
        {
        // Simply load the selected config, 
        // ignoring the possibility of additional changes this round.
				ROS_DEBUG_STREAM("Switching to configure camera: " << newconfig.camera_select
												<< " previously camera " << config_.camera_select);
        newconfig = *configIter;
        }      
      // Update configuration of selected camera
      else
        {
				ROS_DEBUG("Camera selection has not changed...Updating configuration");
        if (config_.camera_info_url != newconfig.camera_info_url || initialising_)
          {
					ROS_DEBUG_STREAM("Calib URL changed from: " << config_.camera_info_url
													<< " to: " << newconfig.camera_info_url);
          // set the new URL and load CameraInfo (if any) from it
          if ((cinfo_.at(newconfig.camera_select))->validateURL(newconfig.camera_info_url))
            {
            (cinfo_.at(newconfig.camera_select))->loadCameraInfo(newconfig.camera_info_url);
            }
          else if (config_.camera_select != -1)
            {
            // new URL not valid, use the old one
						ROS_WARN("Invalid Calib URL provided, reverting to previous");
            newconfig.camera_info_url = config_.camera_info_url;
            }
          }
        
        if (state_ != Driver::CLOSED)       // openCamera() succeeded?
          {
					ROS_WARN("Opening cameras suceeded... Driver is OPEN");
          // configure IIDC features
          if (level & Levels::RECONFIGURE_CLOSE)
            {
						ROS_WARN("Feature modified might require the driver to close");
            // initialize all features for newly opened device
            if (false == (dev_->featuresVec_.at(newconfig.camera_select))->initialize(&newconfig))
              {
              ROS_ERROR_STREAM("[" << camera_name_
                               << "] feature initialization failure");
              closeCamera();          // can't continue
              }
            }
          else
            {
						ROS_INFO("Reconfiguring camera features");
            // update any features that changed
            // TODO replace this with a dev_->reconfigure(&newconfig);
            (dev_->featuresVec_.at(newconfig.camera_select))->reconfigure(&newconfig);
            }
          }
        }
      }
    }       
    
    // If cameras are being initialised for the first time
    if (initialising_)
      {
			ROS_WARN("Initialisation phase is complete");
      // Set 'newconfig' to the first
      newconfig = configVec_.at(0);
			initialising_ = false;
      }
    
    // TODO: do not need to store complete copy of current 'config_'.
    //    Only need the 'camera_select' value to look up in 'configVec_'
    config_ = newconfig;
    configVec_.at(config_.camera_select) = config_;
    
    // let poll() run again
    reconfiguring_ = false;

    ROS_INFO_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url " << newconfig.camera_info_url);
  }


  /** driver initialization
   *
   *  Define dynamic reconfigure callback, which gets called
   *  immediately with level 0xffffffff.  The reconfig() method will
   *  set initial parameter values, then open the device if it can.
   */
  void Camera1394Driver::setup(void)
  {
    srv_.setCallback(boost::bind(&Camera1394Driver::reconfig, this, _1, _2));
  }


  /** driver termination */
  void Camera1394Driver::shutdown(void)
  {
    closeCamera();
  }

}; // end namespace camera1394_driver
