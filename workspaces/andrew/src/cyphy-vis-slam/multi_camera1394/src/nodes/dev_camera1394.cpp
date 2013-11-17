///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010, 2012 Patrick Beeson, Jack O'Quin, Ken Tossell
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// NOTE: On 4 Jan. 2011, this file was re-licensed under the GNU LGPL
// with permission of the original GPL authors: Nate Koenig, Andrew
// Howard, Damien Douxchamps and Dan Dennedy.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////

// $Id: dev_camera1394.cpp 38791 2012-02-04 14:35:23Z joq $

/** @file

    @brief libdc1394 digital camera library interface implementation
 
    This device interface is partly derived from the Player 1394
    camera driver.

    The ROS image pipeline provides Bayer filtering at a higher level
    (in image_proc).  In some cases it is useful to run the driver
    without the entire image pipeline, so libdc1394 Bayer decoding is
    also provided here.

 */

#include <stdint.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>

#include "yuv.h"
#include <sensor_msgs/image_encodings.h>
#include "dev_camera1394.h"
#include "features.h"
#include "modes.h"

#define NUM_DMA_BUFFERS 128

// @todo eliminate these macros
//! Macro for throwing an exception with a message
#define CAM_EXCEPT(except, msg)					\
  {								\
    char buf[100];						\
    snprintf(buf, 100, "[multi_camera1394::%s]: " msg, __FUNCTION__); \
    throw except(buf);						\
  }

//! Macro for throwing an exception with a message, passing args
#define CAM_EXCEPT_ARGS(except, msg, ...)				\
  {									\
    char buf[100];							\
    snprintf(buf, 100, "[multi_camera1394::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf);							\
  }

using namespace multiCamera1394;

////////////////////////////////////////////////////////////////////////////////
// Constructor
MultiCamera1394::MultiCamera1394(): enableEmbeddedTimestamp_(false), globalFirstStamp_(true), resetTimestampLoop_(0)
{}

MultiCamera1394::~MultiCamera1394() 
{
	for (std::vector<dc1394camera_t*>::iterator cameraIter = cameraVec_.begin();
			 cameraIter != cameraVec_.end();
			 ++cameraIter)
		{
		SafeCleanup(*cameraIter);
		*cameraIter = NULL;
		}
	cameraVec_.clear();
}

void MultiCamera1394::findBayerPattern(const char* bayer)
{
  // determine Bayer color encoding pattern
  // (default is different from any color filter provided by DC1394)
  BayerPattern_ = (dc1394color_filter_t) DC1394_COLOR_FILTER_NUM;
  if (0 == strcmp(bayer, "bggr"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_BGGR;
    }
  else if (0 == strcmp(bayer, "grbg"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_GRBG;
    }
  else if (0 == strcmp(bayer, "rggb"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_RGGB;
    }
  else if (0 == strcmp(bayer, "gbrg"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_GBRG;
    }
  else if (0 != strcmp(bayer, ""))
    {
      ROS_ERROR("unknown bayer pattern [%s]", bayer);
    }
}

bool MultiCamera1394::findBayerMethod(const char* method)
{
  // Do Bayer conversion in the driver node?
  bool DoBayer = false;                 // return value
  if (0 != strcmp(method, "")
      && BayerPattern_ != DC1394_COLOR_FILTER_NUM)
    {
      DoBayer = true;                   // decoding in driver
      // add method name to message:
      ROS_WARN("[%s] Bayer decoding in the driver is DEPRECATED;"
               " image_proc decoding preferred.", method);

      // Set decoding method
      if (!strcmp(method, "DownSample"))
        BayerMethod_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
      else if (!strcmp(method, "Simple"))
        BayerMethod_ = DC1394_BAYER_METHOD_SIMPLE;
      else if (!strcmp(method, "Bilinear"))
        BayerMethod_ = DC1394_BAYER_METHOD_BILINEAR;
      else if (!strcmp(method, "HQ"))
        BayerMethod_ = DC1394_BAYER_METHOD_HQLINEAR;
      else if (!strcmp(method, "VNG"))
        BayerMethod_ = DC1394_BAYER_METHOD_VNG;
      else if (!strcmp(method, "AHD"))
        BayerMethod_ = DC1394_BAYER_METHOD_AHD;
      else
        {
          ROS_ERROR("Unknown Bayer method [%s]. Using ROS image_proc instead.",
                    method);
          DoBayer = false;
        }
    }
  return DoBayer;
}

/** Open the 1394 device and start streaming
 *
 *  @param newconfig new configuration parameters
 *  @return 0 if successful
 *
 *  TODO (if successful):
 *     * update newconfig.guid
 *     * validate newconfig.video_mode
 *     * initialize Features class
 */
int MultiCamera1394::open(std::vector<Config> &configVec)
{
  ////
  //* POPULATE CAMERA LIST
  int err;
  dc1394_t *d;
  dc1394camera_list_t *list;

  // TODO: make error exit paths clean up resources properly
  d = dc1394_new ();
  if (d == NULL)
  {
      CAM_EXCEPT(multiCamera1394::Exception,
                 "Could not initialize dc1394_context.\n"
                 "Make sure /dev/raw1394 exists, you have access permission,\n"
                 "and libraw1394 development package is installed.");
  }

  err = dc1394_camera_enumerate(d, &list);
  if (err != DC1394_SUCCESS)
  {
      CAM_EXCEPT(multiCamera1394::Exception, "Could not get camera list");
      return -1;
  }
  
  if (list->num == 0)
  {
      CAM_EXCEPT(multiCamera1394::Exception, "No cameras found");
      return -1;
  }
  
  ROS_WARN_STREAM(list->num << " cameras detected..");
  
  //* CAMERA LIST POPULATED...
  ////
  
  for (std::vector<Config>::iterator configIter = configVec.begin();
       configIter != configVec.end();
       ++configIter)
  {
	  //////////////////////////////////////////////////////////////
	  // Pad GUID (if specified) with leading zeros
	  //////////////////////////////////////////////////////////////
	  const static size_t exact_guid_length = 16;
	  size_t guid_length = (*configIter).guid.length();
	  if (guid_length != 0 && guid_length != exact_guid_length)
	  {
		  if (guid_length < exact_guid_length)
		  {
			  // pad string with leading zeros
			  (*configIter).guid.insert(0, exact_guid_length - guid_length, '0');
		  }
		  else
		  {
			  ROS_ERROR_STREAM_THROTTLE(3, "Invalid GUID [" << (*configIter).guid
										<< "] specified: " << guid_length
										<< " characters long.");
		  }
	  }
		
	  //////////////////////////////////////////////////////////////
	  // First, look for the camera
	  //////////////////////////////////////////////////////////////
		
	  const char *guid = (*configIter).guid.c_str();  // C-style GUID for libdc1394
	  dc1394camera_t *camera_; // Temp camera handle holder		
	  char* temp=(char*)malloc(1024*sizeof(char)); // Temp C-style GUID for comparison
		
	  for (unsigned i=0; i < list->num; i++)
	  {
		  // Create a camera
		  camera_ = dc1394_camera_new (d, list->ids[i].guid);
		  
		  if (!camera_)
			  ROS_WARN_STREAM("Failed to initialize camera with GUID "
							  << std::setw(16) << std::setfill('0') << std::hex
							  << list->ids[i].guid);
		  else
			  ROS_INFO_STREAM("Found camera with GUID "
							  << std::setw(16) << std::setfill('0') << std::hex
							  << list->ids[i].guid);
			
		  uint32_t value[3];
      
		  value[0]= camera_->guid & 0xffffffff;
		  value[1]= (camera_->guid >>32) & 0x000000ff;
		  value[2]= (camera_->guid >>40) & 0xfffff;
      
  		  // TODO: check equivalence of 'camera_' and 'list' guids and remove redundancy
		  // Format 'camera_' GUID -> 'temp'
		  sprintf(temp,"%06x%02x%08x", value[2], value[1], value[0]);
		
		  // Format 'list' GUID -> 'guidSS'
		  std::ostringstream guidSS;
		  guidSS << std::setw(16) << std::setfill('0') 
		  << std::hex << list->ids[i].guid;
			
		  // Case 1: No GUID, add all cameras detected
		  if (strcmp(guid,"")==0)
		  {
			  ROS_INFO("Case 1: Config has Empty GUID");
			  // If the first camera
			  if (i == 0)
			  {
				  ROS_INFO_STREAM("No GUID specified, using all cameras found..");
			  }
			  else
			  {
				  //// Copy default config for new camera
				  ROS_DEBUG_STREAM("Adding default config to array length: " << configVec.size());
				  
				  // New config
				  Config *addConfig = new Config;
				  // Copy default '0'
				  *addConfig = configVec.at(0);
				  
				  // Save and update iterator
				  configVec.push_back(*addConfig);
				  configIter = configVec.end()-1;
				  
				  // Store the selected camera number this config represents
				  (*configIter).camera_select = configVec.size()-1;
				  
				  ROS_DEBUG_STREAM("Suceeded in adding config and setting 'camera_select', array length now: "
								  << configVec.size());
			  }
			  
			  ROS_INFO_STREAM("Adding camera " << i << ": " << guidSS.str());
			  
			  // Array of camera GUID's for public use
			  device_id_.push_back(guidSS.str());
			  ROS_DEBUG("Sucess adding device_id");
			  
			  // Array of camera handles
			  cameraVec_.push_back(camera_);
			  ROS_DEBUG("Sucess adding camera_");
				
			  // Save the GUID
			  (*configIter).guid = guidSS.str();
			  ROS_DEBUG_STREAM("GUID stored in 'configVec' is: "
							  << (*configIter).guid);
				
			  // Save camera name
			  std::ostringstream tempSS;
			  tempSS << "camera_" << distance(configVec.begin(), configIter);
			  ROS_INFO_STREAM("Setting camera [" << (*configIter).guid << "] with 'frame_id' of: " << tempSS.str());
			  (*configIter).frame_id = tempSS.str();
			  
			  continue;
		  }
			
		  // Case 2: Only add specified GUID's
		  // TODO: accept more then one GUID specified
		  // FOR -> all GUID's provided
		  ROS_DEBUG_STREAM("Comparing 'config' GUID " << guid
						  << " to 'camera' GUID " << temp);        
		  if (strcmp(temp,guid)==0)
		  {
			  ROS_INFO("Case 2: GUID match found");
			  ROS_INFO_STREAM("Adding camera " << i << ": " << std::setw(16)
							  << std::setfill('0') << std::hex
							  << camera_->guid);
			  
			  // Array of camera GUID's for public use
			  device_id_.push_back(std::string(temp));
			  ROS_INFO("Sucess adding device_id");
			  
			  // Array of camera handles
			  cameraVec_.push_back(camera_);
			  ROS_INFO("Sucess adding camera_");
        
			  // Save camera name
			  std::ostringstream tempSS;
			  tempSS << "camera_" << distance(configVec.begin(), configIter);
			  ROS_INFO_STREAM("Setting camera [" << (*configIter).guid << "] with 'frame_id' of: " << tempSS.str());
			  (*configIter).frame_id = tempSS.str();
				
				/*if (i != guid.end()-1)
				 {
				 // Search for next GUID in the list
				 continue;
				 }*/
				// IF this was the last GUID provided..
			  break;
		  }
      
		  // Case 3: Not a useful camera
		  ROS_INFO("Case 3: Not a useful camera.");
		  SafeCleanup(camera_);
	  }
	  
	  // If there are more cameras that need to be found
	  if (distance(configVec.begin(), configIter) != configVec.size())
		  continue;
		  
		ROS_DEBUG("Camera list cleanup");
		// Camera handle is held in 'cameraVec_' 
		// or has been passed to 'SafeCleanup()'
		camera_ = NULL; // No longer needed
		
		free (temp);
		dc1394_camera_free_list (list);
    }
  
	ROS_DEBUG("Checking for equal cameras to configs");
  // Ensure we have equal configs to cameras
  if (cameraVec_.size() == configVec.size())
    {
		ROS_DEBUG("Resizing number of video modes and features to match number of cameras.");
    videoModeVec_.resize(cameraVec_.size());
		featuresVec_.resize(cameraVec_.size());
		prevTimestampVec_.resize(cameraVec_.size());
		frameRateVec_.resize(cameraVec_.size());
    }
  else
    {
    CAM_EXCEPT_ARGS(multiCamera1394::Exception,
                    "Can not proceed to configure %d cameras with %d configs defined.", 
                    (int)cameraVec_.size(), (int)configVec.size());
    return -1;
    }
  
	ROS_INFO("Beginning camera config initialisation...");
  // For the number of cameras actually required
  for (std::vector<dc1394camera_t*>::iterator cameraIter = cameraVec_.begin();
       cameraIter != cameraVec_.end();
       ++cameraIter)
    {		
    // Camera number
    unsigned int cameraIdx = distance(cameraVec_.begin(), cameraIter);
    ROS_INFO_STREAM("Camera Index: " << cameraIdx << " now initialising");
			
		const char *guid = (configVec.at(cameraIdx)).guid.c_str();  // C-style GUID for libdc1394
		
		// Get the config for this camera
		Config newconfig = configVec.at(cameraIdx);
    
    if (!(*cameraIter))
      {
      if (strcmp(guid,"")==0)
        { 
          CAM_EXCEPT(multiCamera1394::Exception, "Could not find camera");
        }
      else
        {
        CAM_EXCEPT_ARGS(multiCamera1394::Exception,
                        "Could not find camera with guid %s", guid);
        }
      return -1;
      }
    
    ROS_INFO_STREAM("camera model: " << (*cameraIter)->vendor
                    << " " << (*cameraIter)->model);
    
    //////////////////////////////////////////////////////////////
    // initialize camera
    //////////////////////////////////////////////////////////////
    
    // resetting some cameras is not a good idea
    if (newconfig.reset_on_open
        && DC1394_SUCCESS != dc1394_camera_reset((*cameraIter)))
      {
      // reset failed: log a warning, but continue
      ROS_WARN("Unable to reset camera (continuing).");
      }
    
    // first, set parameters that are common between Format7 and other modes
    if (false == Modes::setIsoSpeed((*cameraIter), newconfig.iso_speed))
      {
      SafeCleanup(*cameraIter);
			*cameraIter = NULL;
      CAM_EXCEPT(multiCamera1394::Exception,
                 "Unable to set ISO speed; is the camera plugged in?");
      return -1;
      }
    
    //// Point Gray specific settings ////
	if (newconfig.camera_type == 1)
	{
		ROS_INFO("Initialise Point Grey specific settings for camera");
		uint64_t offset;
		uint32_t mask;
		uint32_t value;
		
		// set FRAME_INFO
		// -- Enables frame info to be returned as image pixels. 
		// -- Used for relative frame timestamps and camera syncronization.
		// -- 'setRegister(camera, reg_offset, value, mask)'
		if (newconfig.enableEmbeddedTimestamp)
		{	
			ROS_INFO("Enabling 'FRAME_INFO'");
			enableEmbeddedTimestamp_ = true;
			
			offset = 0x12F8;
			value = 0x00000001;
			mask = 0x00000001;
			if (false == setRegister(cameraIdx, offset, value, mask))
			{
				SafeCleanup(*cameraIter);
				*cameraIter = NULL;
				CAM_EXCEPT(multiCamera1394::Exception, "Failed to enable FRAME_INFO");
				return -1;
			}				
		}   
				
		// set AUTO_SHUTTER_RANGE
		// -- Forces an upper bound on the camera auto shutter control.
		// -- Used to avoid motion blur in low light.
		// -- 'setRegister(camera, reg_offset, max_shutter, mask)'		
		ROS_INFO("Enabling 'AUTO_SHUTTER_RANGE'");
		offset = 0x1098;
		value = (uint32_t)newconfig.auto_shutter_maxvalue;
		mask = 0x00000FFF;
		ROS_INFO_STREAM("Setting AUTO_SHUTTER_RANGE to: " << newconfig.auto_shutter_maxvalue);
		if (false == setRegister(cameraIdx, offset, value, mask))
		{
			SafeCleanup(*cameraIter);
			*cameraIter = NULL;
			CAM_EXCEPT(multiCamera1394::Exception, "Failed to set AUTO_SHUTTER_RANGE");
			return -1;
		}
		
		// set AUTO_GAIN_RANGE
		// -- Forces an upper bound on the camera auto gain control.
		// -- Used to avoid excessive noise in low light.
		// -- 'setRegister(camera, reg_offset, max_gain, mask, toggle_val)'
		ROS_INFO("Enabling 'AUTO_GAIN_RANGE'");
		offset = 0x10a0;
		value = (uint32_t)newconfig.auto_gain_maxvalue;
		mask = 0x02000FFF;
		ROS_INFO_STREAM("Setting AUTO_GAIN_RANGE to: " << newconfig.auto_gain_maxvalue);
		if (false == setRegister(cameraIdx, offset, value, mask))
		{
			SafeCleanup(*cameraIter);
			*cameraIter = NULL;
			CAM_EXCEPT(multiCamera1394::Exception, "Failed to set AUTO_GAIN_RANGE");
			return -1;
		}	
	}
	//// END Point Gray specific settings ////
    
    // set video mode
    // TODO: 'videoMode_' should be re-initialised before re-use
    ROS_INFO("Setting video mode");
    videoMode_ = Modes::getVideoMode((*cameraIter), newconfig.video_mode);
    videoModeVec_.at(cameraIdx) = videoMode_;
    
    if (DC1394_SUCCESS != dc1394_video_set_mode((*cameraIter), videoMode_))
      {
      SafeCleanup(*cameraIter);
			*cameraIter = NULL;
      CAM_EXCEPT(multiCamera1394::Exception, "Failed to set video mode");
      return -1;
      }
    
    //////////////////////////////////////////////////////////////
    // special handling for Format7 modes
    //////////////////////////////////////////////////////////////
    ROS_INFO("Format 7 modes");
    DoBayerConversion_ = false;
    
    if (dc1394_is_video_mode_scalable(videoMode_) == DC1394_TRUE)
      {
      // set Format7 parameters
      if (!format7_.start((*cameraIter), videoMode_, newconfig))
        {
        SafeCleanup(*cameraIter);
				*cameraIter = NULL;
        CAM_EXCEPT(multiCamera1394::Exception, "Format7 start failed");
        return -1;
        }
      }
    else
      {
      // Set frame rate and Bayer method (only valid for non-Format7 modes)
      DoBayerConversion_ = findBayerMethod(newconfig.bayer_method.c_str());
      if (!Modes::setFrameRate((*cameraIter), videoMode_, newconfig.frame_rate))
        {
        SafeCleanup(*cameraIter);
				*cameraIter = NULL;
        CAM_EXCEPT(multiCamera1394::Exception, "Failed to set frame rate");
        return -1;
        }
			else
				{
				frameRateVec_.at(cameraIdx) = newconfig.frame_rate;
				}
      }
    
    findBayerPattern(newconfig.bayer_pattern.c_str());
    
    use_ros_time_ = newconfig.use_ros_time;
    
    //////////////////////////////////////////////////////////////
    // start the device streaming data
    //////////////////////////////////////////////////////////////
    ROS_INFO("Start streaming");
    // Set camera to use DMA, improves performance.
    if (DC1394_SUCCESS != dc1394_capture_setup((*cameraIter), NUM_DMA_BUFFERS,
                                               DC1394_CAPTURE_FLAGS_DEFAULT))
      {
      SafeCleanup(*cameraIter);
			*cameraIter = NULL;
      CAM_EXCEPT(multiCamera1394::Exception, "Failed to open device!");
      return -1;
      }
    
    // Start transmitting camera data
    if (DC1394_SUCCESS != dc1394_video_set_transmission((*cameraIter), DC1394_ON))
      {
      SafeCleanup(*cameraIter);
			*cameraIter = NULL;
      CAM_EXCEPT(multiCamera1394::Exception, "Failed to start device!");
      return -1;
      }
			
			// Save the global time offset of when cameras started transmission
			if (cameraIdx == 0)
				{
				start_ = ros::Time::now();
				}
    
    //////////////////////////////////////////////////////////////
    // initialize feature settings
    //////////////////////////////////////////////////////////////
    
    // TODO: pass newconfig here and eliminate initialize() method
    // TODO: allow for different features between cameras
    ROS_WARN("--Before feature reset--");
    featuresVec_.at(cameraIdx).reset(new Features((*cameraIter)));
    ROS_WARN("--After feature reset--");    
  }
	
	// 
 
  return 0;
}


/** Safe Cleanup -- may get called more than once. */
void MultiCamera1394::SafeCleanup(dc1394camera_t *camera_)
{
  if (camera_)
    {
      format7_.stop();
      dc1394_capture_stop(camera_);
      // try to power off the device (#5322):
      dc1394_camera_set_power(camera_, DC1394_OFF);
      dc1394_camera_free(camera_);
      camera_ = NULL;
    }
}


/** close the 1394 device */
int MultiCamera1394::close()
{
  // Close each camera
  for (std::vector<dc1394camera_t*>::iterator cameraIter = cameraVec_.begin();
       cameraIter != cameraVec_.end();
       ++cameraIter)
    {
    if (*cameraIter)
      {
      if (DC1394_SUCCESS != dc1394_video_set_transmission(*cameraIter, DC1394_OFF)
          || DC1394_SUCCESS != dc1394_capture_stop(*cameraIter))
        ROS_WARN("unable to stop camera");
      }
    
    // Free resources
    SafeCleanup(*cameraIter);
		*cameraIter = NULL;
    }
	
	cameraVec_.clear();

  return 0;
}

std::string bayer_string(dc1394color_filter_t pattern, unsigned int bits)
{
  if (bits == 8)
    {
      switch (pattern)
        {
        case DC1394_COLOR_FILTER_RGGB:
          return sensor_msgs::image_encodings::BAYER_RGGB8;
        case DC1394_COLOR_FILTER_GBRG:
          return sensor_msgs::image_encodings::BAYER_GBRG8;
        case DC1394_COLOR_FILTER_GRBG:
          return sensor_msgs::image_encodings::BAYER_GRBG8;
        case DC1394_COLOR_FILTER_BGGR:
          return sensor_msgs::image_encodings::BAYER_BGGR8;
        default:
          return sensor_msgs::image_encodings::MONO8;
        }
    }
  else if (bits == 16)
    {
      switch (pattern)
        {
        case DC1394_COLOR_FILTER_RGGB:
          return sensor_msgs::image_encodings::BAYER_RGGB16;
        case DC1394_COLOR_FILTER_GBRG:
          return sensor_msgs::image_encodings::BAYER_GBRG16;
        case DC1394_COLOR_FILTER_GRBG:
          return sensor_msgs::image_encodings::BAYER_GRBG16;
        case DC1394_COLOR_FILTER_BGGR:
          return sensor_msgs::image_encodings::BAYER_BGGR16;
        default:
          return sensor_msgs::image_encodings::MONO16;
        }
    }

  // bits should always be 8 or 16, but if not MONO8 is a good default
  return sensor_msgs::image_encodings::MONO8;
}

/** Return an image frame */
void MultiCamera1394::readData(std::vector<sensor_msgs::ImagePtr> &image)
{
	ros::Time lastStamp(0);
	
	// TODO: iterate over cameras NOT images... resize image vector if needed...
  for (std::vector<sensor_msgs::ImagePtr>::iterator imageIter = image.begin();
       imageIter != image.end();
       ++imageIter)
    {
    // 'image' has been resized to 'cameraVec_' size
    unsigned int cameraIdx = distance(image.begin(), imageIter);
    sensor_msgs::ImagePtr imgPtr = *imageIter;
		
		// Try to grab a frame from 'cameraIdx'
		ROS_DEBUG_STREAM("Attempting to grab frame for camera " << cameraIdx);
		int err_ = -1;
		while (err_)
			{
			err_ = grabFrame(cameraIdx, imgPtr);
			}
		ROS_DEBUG("Frame acquired successfully");
		
		//// Check monocular frame consistency
		// Keep a record of latest timestamp
		if ( ((*imgPtr).header.stamp.sec > lastStamp.sec) ||
			(((*imgPtr).header.stamp.sec == lastStamp.sec) && ((*imgPtr).header.stamp.nsec > lastStamp.nsec)))
			lastStamp = (*imgPtr).header.stamp;
		
		// Check if there are any inconsistencies in the framerate
		double framePeriod = 1.0/frameRateVec_.at(cameraIdx);
		ROS_DEBUG_STREAM("prevTimeStampVec for camera " << cameraIdx
										<< " is: " << prevTimestampVec_.at(cameraIdx));
		double timestampDiff = (double)((*imgPtr).header.stamp.toSec()) - (double)(prevTimestampVec_.at(cameraIdx).toSec());
		if ((timestampDiff > (framePeriod + framePeriod/3) || // timestamp difference is 33% greater than expected
				 timestampDiff < (framePeriod - framePeriod/3)) && // timestamp difference is 33% less than expected
				prevTimestampVec_.at(cameraIdx).toSec() != 0) // timestamp difference isn't the first frame
			{				
			ROS_WARN_STREAM("DEV_CAMERA1394 Warning: Timestamp inconsistency for camera " << cameraIdx << ".");
				
				ros::Duration tDiff = (*imgPtr).header.stamp - prevTimestampVec_.at(cameraIdx);
				ROS_INFO_STREAM("Current: " << (*imgPtr).header.stamp.sec 
								<< ", " << (*imgPtr).header.stamp.nsec
								<< " Previous: " << prevTimestampVec_.at(cameraIdx).sec 
								<< ", " << prevTimestampVec_.at(cameraIdx).nsec
								<< " Diff: " << tDiff.toSec());

			ROS_WARN_STREAM("Expected timestamp difference: " << framePeriod 
											<< " secs. Observed timestamp difference: " << timestampDiff << "secs.");

			ROS_WARN_STREAM("Camera dropped approximately " << timestampDiff/framePeriod 
											<< " frames, with a buffer size of " << NUM_DMA_BUFFERS);
			}
		
		// Keep a record of the current timestamp for this camera
		prevTimestampVec_.at(cameraIdx) = (*imgPtr).header.stamp;    

    }
	
	//// Syncronise Buffers
	int result = synchroniseBuffers(lastStamp, image);
	if (result == 1) 
		{
		ROS_WARN("DEV_CAMERA1394 Warning: Frames out of sync. Corrected by dropping out of sync frames.");
		}
	else if (result == -1) 
		{
			// TODO: Handle...
		ROS_ERROR("DEV_CAMERA1394 Warning: Synching failed.");
		}
}

int MultiCamera1394::synchroniseBuffers(ros::Time lastStamp, std::vector<sensor_msgs::ImagePtr> &image) 
{
	ROS_DEBUG("DEV_CAMERA1394 Info: Starting synchroniseBuffers()");
	bool outOfSync = false, outOfSyncFlag = false;
	unsigned syncCount = NUM_DMA_BUFFERS*2;
	do { // Check timestamps for inconsistencies
		outOfSync = false;
		//stringstream ss1;
		for (std::vector<sensor_msgs::ImagePtr>::iterator imageIter = image.begin();
				 imageIter != image.end();
				 ++imageIter) 
			{
			unsigned int cameraIdx = distance(image.begin(), imageIter);
			sensor_msgs::ImagePtr imgPtr = *imageIter;
			ros::Duration difference = lastStamp - (*imgPtr).header.stamp;
			double framePeriod = 1.0/frameRateVec_.at(cameraIdx);
			
			// If timestamp difference is 33% greater than expected
			if (difference.toSec() > framePeriod/3) 
				{
				outOfSync = true;
				outOfSyncFlag = true;
				ROS_WARN_STREAM("DEV_CAMERA1394 Warning: Frames out of sync. Timestamp difference: " 
												<< difference <<" secs.");

				// If something went wrong grabbing the frame, break out
				if (grabFrame(cameraIdx, imgPtr)) break;
				// Make sure that we still have the newest timestamp
				if ( ((*imgPtr).header.stamp.sec > lastStamp.sec) ||
					(((*imgPtr).header.stamp.sec == lastStamp.sec) && ((*imgPtr).header.stamp.nsec > lastStamp.nsec)))
					lastStamp = (*imgPtr).header.stamp;
			} 
		}
		syncCount--;
	} while (outOfSync);//&& (syncCount > 0));
	if (syncCount <= 0)
		return -1;
	else if (outOfSyncFlag)
		return 1;
	else
		return 0;
}

int MultiCamera1394::grabFrame(int cameraIdx, sensor_msgs::ImagePtr imgPtr)
{
	ROS_DEBUG("START frame grab");
	ROS_ASSERT_MSG(cameraVec_.at(cameraIdx), "Attempt to read from camera that is not open.");
	
	dc1394video_frame_t *frame = NULL;
	////////////////
	ROS_DEBUG("Dequeue buffered frame");
	int err_ = dc1394_capture_dequeue(cameraVec_.at(cameraIdx), DC1394_CAPTURE_POLICY_WAIT, &frame);
	if (err_ == -2)
    {
		ROS_ERROR("DEV_CAMERA1394 Error: No frame in buffer ....");
		return -1;
    }
	else
    {
		if (dc1394_capture_is_frame_corrupt(cameraVec_.at(cameraIdx), frame) == DC1394_TRUE)
			{
			ROS_ERROR("DEV_CAMERA1394 Error: Buffer corrupted");
			return -1;
			}
		//////////////
		
		// Define the time of frame arriving
		// TODO: If 'enableEmbeddedTimestamp_' this only needs to be done once..
    if (use_ros_time_)
      {
      (*imgPtr).header.stamp = ros::Time::now();
			ROS_DEBUG("Using ROS time");
      }
    else
      {
      (*imgPtr).header.stamp = ros::Time((double) frame->timestamp / 1000000.0);
			ROS_DEBUG("Using DMA time");
      }
		
		uint8_t* capture_buffer;
    dc1394video_frame_t frame2;
    
    if (DoBayerConversion_)
      {
      // debayer frame into RGB8
      size_t frame2_size = (frame->size[0] * frame->size[1]
                            * 3 * sizeof(unsigned char));
      frame2.image = (unsigned char *) malloc(frame2_size);
      frame2.allocated_image_bytes = frame2_size;
      frame2.color_coding = DC1394_COLOR_CODING_RGB8;
      
      frame->color_filter = BayerPattern_;
      int err = dc1394_debayer_frames(frame, &frame2, BayerMethod_);
      if (err != DC1394_SUCCESS)
        {
        free(frame2.image);
        dc1394_capture_enqueue(cameraVec_.at(cameraIdx), frame);
        CAM_EXCEPT(multiCamera1394::Exception, "Could not convert/debayer frames");
        return -1;
        }
      
      capture_buffer = reinterpret_cast<uint8_t *>(frame2.image);
      
      (*imgPtr).width = frame2.size[0];
      (*imgPtr).height = frame2.size[1];
      }
    else
      {
      (*imgPtr).width = frame->size[0];
      (*imgPtr).height = frame->size[1];
      capture_buffer = reinterpret_cast<uint8_t *>(frame->image);
      }
    
    ROS_ASSERT(capture_buffer);   
    
    int image_size;  
    switch (videoMode_)
      {
        case DC1394_VIDEO_MODE_160x120_YUV444:
        (*imgPtr).step = (*imgPtr).width*3;
        image_size = (*imgPtr).height * (*imgPtr).step;
        (*imgPtr).encoding = sensor_msgs::image_encodings::RGB8;
        (*imgPtr).data.resize(image_size);
        yuv::uyv2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                     reinterpret_cast<unsigned char *> (&(*imgPtr).data[0]),
                     (*imgPtr).width * (*imgPtr).height);
        break;
        case DC1394_VIDEO_MODE_640x480_YUV411:
        (*imgPtr).step = (*imgPtr).width*3;
        image_size = (*imgPtr).height * (*imgPtr).step;
        (*imgPtr).encoding = sensor_msgs::image_encodings::RGB8;
        (*imgPtr).data.resize(image_size);
        yuv::uyyvyy2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                        reinterpret_cast<unsigned char *> (&(*imgPtr).data[0]),
                        (*imgPtr).width * (*imgPtr).height);
        break;
        case DC1394_VIDEO_MODE_320x240_YUV422:
        case DC1394_VIDEO_MODE_640x480_YUV422:
        case DC1394_VIDEO_MODE_800x600_YUV422:
        case DC1394_VIDEO_MODE_1024x768_YUV422:
        case DC1394_VIDEO_MODE_1280x960_YUV422:
        case DC1394_VIDEO_MODE_1600x1200_YUV422:
        (*imgPtr).step = (*imgPtr).width*3;
        image_size = (*imgPtr).height * (*imgPtr).step;
        (*imgPtr).encoding = sensor_msgs::image_encodings::RGB8;
        (*imgPtr).data.resize(image_size);
        yuv::uyvy2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                      reinterpret_cast<unsigned char *> (&(*imgPtr).data[0]),
                      (*imgPtr).width * (*imgPtr).height);
        break;
        case DC1394_VIDEO_MODE_640x480_RGB8:
        case DC1394_VIDEO_MODE_800x600_RGB8:
        case DC1394_VIDEO_MODE_1024x768_RGB8:
        case DC1394_VIDEO_MODE_1280x960_RGB8:
        case DC1394_VIDEO_MODE_1600x1200_RGB8:
        (*imgPtr).step = (*imgPtr).width*3;
        image_size = (*imgPtr).height * (*imgPtr).step;
        (*imgPtr).encoding = sensor_msgs::image_encodings::RGB8;
        (*imgPtr).data.resize(image_size);
        memcpy(&(*imgPtr).data[0], capture_buffer, image_size);
        break;
        case DC1394_VIDEO_MODE_640x480_MONO8:
        case DC1394_VIDEO_MODE_800x600_MONO8:
        case DC1394_VIDEO_MODE_1024x768_MONO8:
        case DC1394_VIDEO_MODE_1280x960_MONO8:
        case DC1394_VIDEO_MODE_1600x1200_MONO8:
        if (!DoBayerConversion_)
          {
          (*imgPtr).step = (*imgPtr).width;
          image_size = (*imgPtr).height * (*imgPtr).step;
          // set Bayer encoding in ROS Image message
          (*imgPtr).encoding = bayer_string(BayerPattern_, 8);
          (*imgPtr).data.resize(image_size);
          memcpy(&(*imgPtr).data[0], capture_buffer, image_size);
          }
        else
          {
          (*imgPtr).step = (*imgPtr).width*3;
          image_size = (*imgPtr).height * (*imgPtr).step;
          (*imgPtr).encoding = sensor_msgs::image_encodings::RGB8;
          (*imgPtr).data.resize(image_size);
          memcpy(&(*imgPtr).data[0], capture_buffer, image_size);
          } 
        break;
        case DC1394_VIDEO_MODE_640x480_MONO16:
        case DC1394_VIDEO_MODE_800x600_MONO16:
        case DC1394_VIDEO_MODE_1024x768_MONO16:
        case DC1394_VIDEO_MODE_1280x960_MONO16:
        case DC1394_VIDEO_MODE_1600x1200_MONO16:
        if (!DoBayerConversion_)
          {
          (*imgPtr).step = (*imgPtr).width*2;
          image_size = (*imgPtr).height * (*imgPtr).step;
          (*imgPtr).encoding = bayer_string(BayerPattern_, 16);
          (*imgPtr).is_bigendian = true;
          (*imgPtr).data.resize(image_size);
          memcpy(&(*imgPtr).data[0], capture_buffer, image_size);
          }
        else
          {
          // @todo test Bayer conversions for mono16
          (*imgPtr).step = (*imgPtr).width*3;
          image_size = (*imgPtr).height * (*imgPtr).step;
          (*imgPtr).encoding = sensor_msgs::image_encodings::RGB8;
          (*imgPtr).data.resize(image_size);
          memcpy(&(*imgPtr).data[0], capture_buffer, image_size);
          } 
        break;
        default:
        if (dc1394_is_video_mode_scalable(videoMode_))
          {
          format7_.unpackData(*imgPtr, capture_buffer);
          }
        else
          {
          CAM_EXCEPT(multiCamera1394::Exception, "Unknown image mode");
          return -1;
          }
      }
		
		ROS_DEBUG("Enqueue buffered frame");
    dc1394_capture_enqueue(cameraVec_.at(cameraIdx), frame);
    
    if (DoBayerConversion_) 
      free(capture_buffer);
		
		// Point Grey specific image Embedded Timestamp
		// Overwrite image timestamp with embedded timestamp
		if (enableEmbeddedTimestamp_)
			{
			ROS_DEBUG("Embedded Timestamp enabled");
			//// From 'robovismultiimagedc1394\driver.cpp' Michael Warren et al.
			// Get the embedded Pt Grey timestamp
			unsigned int stamp;
			unsigned char* pStamp = (unsigned char*)&stamp;
			pStamp[ 0 ] = imgPtr->data[3];
			pStamp[ 1 ] = imgPtr->data[2];
			pStamp[ 2 ] = imgPtr->data[1];
			pStamp[ 3 ] = imgPtr->data[0];
			int nSecond = (stamp >> 25)&0x7F;
			int nCycleCount  = (stamp >> 12)&0x1FFF;
			int nCycleOffset = (stamp >> 0)&0xFFF;    
			
			// Is this the very first timestamp of the very first camera?
			// We need an accurate time at the start of the capture sequence because
			// the timestamps are relative and operate in a loop. They don't have
			// an absolute time referece, so we need to make one and work from that
			if (globalFirstStamp_) 
				{
				globalFirstStamp_ = false;
				// Subtract the dc1394 cycle time in seconds from the start time
				int usec = (((double)nCycleCount+((double)nCycleOffset/3072.0))/8000000000.0);
				if (usec > start_.nsec/1000) 
					{
					start_.sec -= 1;
					start_.nsec += 1e9;
					}
				start_.sec = start_.sec - nSecond;
				start_.nsec = start_.nsec - usec*1000;
				}
			if (nSecond == 127) {
				resetTimestampLoop_ = 1;
			}
			if (nSecond == 0 && resetTimestampLoop_ == 1) {
				ROS_ERROR("Embedded Timestamp calculation: nSecond reboot");
				resetTimestampLoop_ = 0;
				start_.sec = start_.sec+128;
			}
			long nUSecond = (((double)nCycleCount+((double)nCycleOffset/3072.0))/8000.0)*1e6 + start_.nsec/1000;
			// If we have more than a second's worth of microseconds, add a second and subtract 1000000 microseconds
				ROS_DEBUG_STREAM("Embedded Timestamp calculation CAMERA: " << cameraIdx);
			while (nUSecond > 1e6) {
				ROS_DEBUG_STREAM("Adding second worth of microseconds: PRE "
								 << nSecond << " secs, "
								 << nUSecond << " Usecs");
				nSecond += 1;
				nUSecond -= 1e6;
				ROS_DEBUG_STREAM("POST: "
								 << nSecond << " secs, "
								 << nUSecond << " Usecs");
			}
			// Copy some data from the other pixels to make the image look normal.
			imgPtr->data[0] = imgPtr->data[4];
			imgPtr->data[1] = imgPtr->data[5];
			imgPtr->data[2] = imgPtr->data[6];
			imgPtr->data[3] = imgPtr->data[7];
			////		
			
			// Save the extracted image time stamp
			ROS_DEBUG("Saving Embedded time");
			(*imgPtr).header.stamp.nsec = (long) nUSecond*1000;
			(*imgPtr).header.stamp.sec = (long) start_.sec+nSecond;
			}
    }
	return 0;
}

bool MultiCamera1394::setRegister(unsigned int cameraIdx, uint64_t offset, uint32_t &value, uint32_t mask)
{
    uint32_t origValue;
    
    if (dc1394_get_control_register(cameraVec_.at(cameraIdx), offset, &origValue) != DC1394_SUCCESS)
    {
		SafeCleanup(cameraVec_.at(cameraIdx));
		cameraVec_.at(cameraIdx) = NULL;
		ROS_WARN_STREAM("Can't get register 0x" << std::hex << offset 
                        << " for cam " << cameraIdx);
        return false;
    }
    else
    {
		ROS_INFO_STREAM("Actual unmodified -> register 0x" << std::hex << offset 
										<< " for camera " << cameraIdx << ": 0x" << std::hex << origValue);
    }
    
    // Make sure the new value is correct according to the mask
    uint32_t modifiedVal = value & mask;
    
    if (modifiedVal != value) 
    {
        ROS_WARN("The requested value does not fit the mask!");
        return false;
    }
    
    // Clear out the old value
    origValue = origValue & ~mask;
    // Insert the new shutter value
    uint32_t newValue = origValue | value;
	ROS_INFO_STREAM("Desired modification -> register 0x" << std::hex << offset 
									<< " for camera " << std::dec << cameraIdx << ": 0x" << std::hex 
									<< std::setfill('0') << std::setw(8) << newValue);
    
    // Rewrite the value back
    if (dc1394_set_control_register(cameraVec_.at(cameraIdx), offset, newValue) != DC1394_SUCCESS)
    {
    SafeCleanup(cameraVec_.at(cameraIdx));
		cameraVec_.at(cameraIdx) = NULL;
		ROS_WARN_STREAM("Can't get register 0x" << std::hex << offset 
                        << " for cam " << cameraIdx);
        return false;
    }
    else
    {
        uint32_t getValue;
        
        if (dc1394_get_control_register(cameraVec_.at(cameraIdx), offset, &getValue) != DC1394_SUCCESS)
        {
				SafeCleanup(cameraVec_.at(cameraIdx));
				cameraVec_.at(cameraIdx) = NULL;
				ROS_WARN_STREAM("Can't get register 0x" << std::hex << offset 
                            << " for cam " << cameraIdx);
            return false;
        }
		ROS_INFO_STREAM("Actual result -> register 0x" << std::hex << offset 
										<< " for camera " << cameraIdx << ": 0x" << std::hex << getValue);
    }
    return true;
}
