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
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>

// uscauv
#include <uscauv_common/base_node.h>

/// ALVAR
#include <ar_track_alvar/AlvarMarkers.h>

/// CHUGG
#include <chugg_tracker/chugg_filter.h>

typedef ar_track_alvar::AlvarMarker _AlvarMarker;
typedef ar_track_alvar::AlvarMarkers _AlvarMarkers;

class ChuggTrackerNode: public BaseNode
{
private:
  ros::NodeHandle nh_rel_;
  ros::Publisher filter_rpy_pub_;
  ros::Subscriber marker_sub_;
  tf::TransformBroadcaster br_;
  
  tf::Transform filtered_;

  chugg::ChuggFilter filter_;

 public:
  ChuggTrackerNode(): BaseNode("ChuggTracker"), nh_rel_("~"), filtered_( tf::Transform::getIdentity() )
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       marker_sub_ = nh_rel_.subscribe<_AlvarMarkers>("markers_in", 1, &ChuggTrackerNode::markerCallback, this);

       filter_rpy_pub_ = nh_rel_.advertise<geometry_msgs::Vector3>("filter/rpy", 1);
     }

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {
       /// Predict
       filter_.predict();

       /// convert filtered orientation to RPY and publish
       double roll, pitch, yaw;
       filtered_.getBasis().getRPY(roll, pitch, yaw);
       geometry_msgs::Vector3 euler;
       euler.x = roll;
       euler.y = pitch;
       euler.z = yaw;
       filter_rpy_pub_.publish(euler);
       
       tf::StampedTransform filtered_tf( filtered_, ros::Time::now(), "/camera_link", "chugg/pose/filter"),
	 filtered_quat( tf::Transform(filtered_.getRotation()), ros::Time::now(), "/camera_link", "chugg/ori/filter");
       
       br_.sendTransform(filtered_tf);
       br_.sendTransform(filtered_quat);
     }
  
  void markerCallback( _AlvarMarkers::ConstPtr const & msg)
  {
    if( msg-> markers.size() < 1)
      return;
    if( msg->markers.size() > 1)
      ROS_WARN("More than one cube seems to be tracked. Taking first cube only.");
    
    /// An annoying thing about ALVAR: The AlvarMarkers message and its PoseStamped children don't have their
    /// stamps populated. Only the AlvarMarker messages do
    _AlvarMarker const & marker = msg->markers[0];
    
    unsigned int const confidence = marker.confidence;
    
    tf::Stamped<tf::Transform> marker_to_world;
    tf::poseStampedMsgToTF( marker.pose, marker_to_world );
    marker_to_world.stamp_ = marker.header.stamp;
    
    /// TODO: Use this instead of hard-coding /camera_link
    std::string const marker_parent = marker.header.frame_id;

    std::vector<tf::StampedTransform> output;
    
    tf::Quaternion marker_to_world_quat = marker_to_world.getRotation();

    tf::StampedTransform 
      marker_to_world_tf(marker_to_world, marker_to_world.stamp_, "/camera_link", "/chugg/pose/markers"),
      marker_to_world_rot( tf::Transform(marker_to_world.getRotation()), marker_to_world.stamp_, "/camera_link", 
			    "/chugg/ori/markers");
    output.push_back(marker_to_world_tf);
    output.push_back(marker_to_world_rot);
    
    br_.sendTransform(output);


    //////////////////////////////////////////////////////////
    // Incorporate measurement into filter////////////////////
    //////////////////////////////////////////////////////////
    
    filter_.updateMarkers( marker_to_world_quat );
    
    /// TODO: real filtering
    filtered_ = marker_to_world_tf;
  }

};

#endif // CHUGG_CHUGGTRACKER_CHUGGTRACKER
