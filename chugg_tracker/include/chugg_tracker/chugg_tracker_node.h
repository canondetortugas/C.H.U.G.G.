/***************************************************************************
 *  include/chugg_tracker/chugg_tracker_node.h
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


#ifndef CHUGG_CHUGGTRACKER_CHUGGTRACKER
#define CHUGG_CHUGGTRACKER_CHUGGTRACKER

// ROS
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

// uscauv
#include <uscauv_common/base_node.h>

/// ALVAR
#include <ar_track_alvar/AlvarMarkers.h>

/// CHUGG
#include <chugg_tracker/chugg_filter.h>
#include <chugg_tracker/Posterior.h>

typedef chugg_tracker::Posterior _Posterior;
typedef chugg_tracker::Sample  _Sample;

typedef ar_track_alvar::AlvarMarker _AlvarMarker;
typedef ar_track_alvar::AlvarMarkers _AlvarMarkers;

typedef geometry_msgs::Vector3 _Vector3;

class ChuggTrackerNode: public BaseNode
{
private:
  ros::NodeHandle nh_rel_;
  ros::Publisher filter_rpy_pub_, post_pub_;
  ros::Subscriber marker_sub_, imu_sub_;
  tf::TransformBroadcaster br_;
  tf::TransformListener lr_;

  std::string base_frame_;
  
  tf::Transform filtered_;

  chugg::ChuggFilter filter_;
  
  bool passthrough_;

public:
  ChuggTrackerNode(): BaseNode("ChuggTracker"), nh_rel_("~"), filtered_( tf::Transform::getIdentity() )
  {
  }

private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    passthrough_ = !( uscauv::param::load<bool>(nh_rel_, "enable_filter", false) );
    base_frame_ = uscauv::param::load<std::string>(nh_rel_, "base_frame", "/camera_link");

    marker_sub_ = nh_rel_.subscribe<_AlvarMarkers>("markers_in", 1, &ChuggTrackerNode::markerCallback, this);
    
    filter_rpy_pub_ = nh_rel_.advertise<geometry_msgs::Vector3>("filter/rpy", 1);
    
    if( !passthrough_ )
      {
	post_pub_ = nh_rel_.advertise<_Posterior>("filter/posterior", 1);
	imu_sub_ = nh_rel_.subscribe<geometry_msgs::Vector3Stamped>("imu_in", 1, &ChuggTrackerNode::rateCallback, this );
      }

  }
  
  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    if( passthrough_ )
      return;

    /// Predict
    filter_.predict();
       
    std::shared_ptr<_Posterior> post = filter_.getPosterior();
    post_pub_.publish(*post);

    tf::Transform filtered = tf::Transform(filter_.getEstimator());

    /// convert filtered orientation to RPY and publish
    double roll, pitch, yaw;
    filtered.getBasis().getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 euler;
    euler.x = roll;
    euler.y = pitch;
    euler.z = yaw;
    filter_rpy_pub_.publish(euler);
       
    tf::StampedTransform filtered_tf( filtered, ros::Time::now(), base_frame_, "chugg/ori/final");
       
    br_.sendTransform(filtered_tf);
  }
  
  void markerCallback( _AlvarMarkers::ConstPtr const & msg)
  {
    if( msg-> markers.size() < 1)
      return;
    if( msg->markers.size() > 1)
      ROS_WARN("More than one cube seems to be being tracked. Taking first cube only.");
    
    /// An annoying thing about ALVAR: The AlvarMarkers message and its PoseStamped children don't have their
    /// stamps populated. Only the AlvarMarker messages do
    _AlvarMarker const & marker = msg->markers[0];
    
    unsigned int const confidence = marker.confidence;
    
    tf::Stamped<tf::Transform> marker_to_world;
    tf::poseStampedMsgToTF( marker.pose, marker_to_world );
    marker_to_world.stamp_ = marker.header.stamp;
    
    std::string const marker_parent = marker.header.frame_id;
    ROS_ASSERT( marker_parent == base_frame_ );

    std::vector<tf::StampedTransform> output;
    
    tf::Quaternion marker_to_world_quat = marker_to_world.getRotation();

    std::string tf_suffix;
    if (passthrough_)
      tf_suffix = "final";
    else
      tf_suffix = "markers";

    tf::StampedTransform 
      marker_to_world_tf(marker_to_world, marker_to_world.stamp_, base_frame_, std::string("/chugg/pose/") + tf_suffix ),
      marker_to_world_rot( tf::Transform(marker_to_world.getRotation()), marker_to_world.stamp_, base_frame_, 
			   std::string("/chugg/ori/") + tf_suffix);
    output.push_back(marker_to_world_tf);
    output.push_back(marker_to_world_rot);
    
    br_.sendTransform(output);

    tf::StampedTransform body_to_marker_tf;
    tf::Quaternion  body_to_marker_quat;

    if( getTransform("/chugg/cm", "/chugg/marker0", body_to_marker_tf) )
      {
	ROS_ERROR("Can't incorporate rate measurement - failed to lookup IMU body transform");
	return;
      }
    body_to_marker_quat = body_to_marker_tf.getRotation();

    tf::Quaternion body_to_world_quat = marker_to_world_quat * body_to_marker_quat;

    //////////////////////////////////////////////////////////
    // Incorporate measurement into filter////////////////////
    //////////////////////////////////////////////////////////
    
    if( !passthrough_ )
      {
	filter_.updateMarkers( body_to_world_quat );
      }
    // filtered_ = marker_to_world_tf;
  }

  void rateCallback( geometry_msgs::Vector3Stamped::ConstPtr const & msg)
  {
    tf::Vector3 rate_imu, rate_body;
    tf::StampedTransform body_to_imu_tf;
    tf::Quaternion body_to_imu_ori;

    if( getTransform("/chugg/cm", "/chugg/imu", body_to_imu_tf) )
      {
	ROS_ERROR("Can't incorporate rate measurement - failed to lookup IMU body transform");
	return;
      }
    body_to_imu_ori = body_to_imu_tf.getRotation();
    
    tf::vector3MsgToTF(msg->vector, rate_imu);

    /// Convert angular rate into cube body coordinate system
    rate_body = tf::quatRotate(body_to_imu_ori, rate_imu);
    
    /// Update the particle filter
    filter_.updateIMU( rate_body );    
  }

  int getTransform( std::string const & parent, std::string const & child, tf::StampedTransform & output )
  {
    if( lr_.canTransform( parent, child, ros::Time(0) ))
      {
	try
	  {
	    lr_.lookupTransform( parent, child, ros::Time(0), output );

	  }
	catch(tf::TransformException & ex)
	  {
	    ROS_ERROR( "Caught exception [ %s ] looking up transform", ex.what() );
	    return -1;
	  }
	return 0;
      }
    else 
      return -1;
  }

};

#endif // CHUGG_CHUGGTRACKER_CHUGGTRACKER
