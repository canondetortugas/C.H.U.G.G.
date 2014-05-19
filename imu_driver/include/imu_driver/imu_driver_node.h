/***************************************************************************
 *  include/imu_driver/imu_driver_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Dylan Foster (turtlecannon@gmail.com)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of CHUGG nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/


#ifndef CHUGG_IMUDRIVER_IMUDRIVER
#define CHUGG_IMUDRIVER_IMUDRIVER

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/macros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <vectornav.h>

static const char* const COM_PORT = "//dev//ttyUSB0";
static const char * const IMU_FRAME = "/imu";

class ImuDriverNode: public BaseNode
{
private:
  typedef geometry_msgs::Vector3Stamped _Vector3;

private:

  Vn100 vn100_;
  VnVector3 magnet_, acc_, rate_;
  VnQuaternion quat_;
  // VnMatrix3x3 mat_;
  // VnYpr ypr_;
  float temp_;
  unsigned int status_;

  _Vector3 ar_;
  tf::Transform imu_tf_;

  std::map<unsigned int, std::string> status_map_;

  ros::NodeHandle nh_rel_;
  ros::Publisher rate_pub_;

  tf::TransformBroadcaster br_;
  
  bool ori_;
  int baud_rate_;

  bool connected_;
  
 public:
  ImuDriverNode(): BaseNode("ImuDriver"), nh_rel_("~"), connected_(false)
   {
     status_map_ = {{0, "No Error"},
		    {1, "Unknown Error"},
		    {2, "Not Implemented"},
		    {3, "Timeout"},
		    {4, "Invalid Value"},
		    {5, "File Not Found"},
		    {6, "Not Connected"}};
   }

  ~ImuDriverNode()
  {
    if(connected_)
      {
	status_ = vn100_disconnect(&vn100_);
	/// ros printing does not work at this point
	std::cout <<"Disconnected from IMU with status " << brk(status_map_.at(status_)) << std::endl;
      }

  }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
              
       ori_ = uscauv::param::load<bool>(nh_rel_, "ori", false);
       baud_rate_ = uscauv::param::load<int>(nh_rel_, "baud_rate", 921600);

       rate_pub_ = nh_rel_.advertise<_Vector3>("angular_rate", 1);
       
       status_ = vn100_connect(&vn100_, COM_PORT, baud_rate_);
       if(status_)
	 {
	   ROS_FATAL_STREAM("Failed to open IMU with error " << brk(status_map_.at(status_)));
	   ros::shutdown();
	 }
       else
	 {
	   connected_ = true;
	 }
              
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    status_ = vn100_getCalibratedImuMeasurements(&vn100_, &magnet_, &acc_, &rate_, &temp_);
    
    if(status_)
      {
	ROS_ERROR_STREAM("Failed to read IMU rate with error " << brk(status_map_.at(status_)));
      }
    else
      {
    	ar_.vector.x = rate_.c0;
	ar_.vector.y = rate_.c1;
	ar_.vector.z = rate_.c2;
	ar_.header.stamp = ros::Time::now();
	ar_.header.frame_id = IMU_FRAME;

	rate_pub_.publish(ar_);
      }

    if(ori_)
      {
	status_ = vn100_getQuaternion(&vn100_, &quat_);
	if(status_)
	  {
	    ROS_ERROR_STREAM("Failed to read IMU orientation with error " << brk(status_map_.at(status_)));
	  }
	else
	  {
	    /// Quaternion initializes in xyzw order

	    /// Negating the x component fixes what seems to be an issue with handedness, where rotations around the x axis were in the wrong direction
	    /// Inverting the quaternion makes vn100 the convention match the tf convention
	    /// Multiplying by an additional quaternion rotates 180 around the x axis so that the frame is aligned with the markings on the IMU body
	    imu_tf_ = tf::Transform( tf::Quaternion(-quat_.x, quat_.y, quat_.z, quat_.w).inverse()*tf::Quaternion(1, 0, 0, 0) );

	    tf::StampedTransform output( imu_tf_, ros::Time::now(), "/world", IMU_FRAME);
	    
	    br_.sendTransform(output);
	  }

      }
    
  }

};

#endif // CHUGG_IMUDRIVER_IMUDRIVER
