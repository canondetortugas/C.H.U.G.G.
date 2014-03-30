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

// uscauv
#include <uscauv_common/base_node.h>

/// ALVAR
#include <ar_track_alvar/AlvarMarkers.h>

typedef ar_track_alvar::AlvarMarker _AlvarMarker;
typedef ar_track_alvar::AlvarMarkers _AlvarMarkers;

class ChuggTrackerNode: public BaseNode
{
private:
  ros::NodeHandle nh_rel_;
  ros::Subscriber marker_sub_;
  tf::TransformBroadcaster br_;
  
  

 public:
  ChuggTrackerNode(): BaseNode("ChuggTracker"), nh_rel_("~")
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
    
       marker_sub_ = nh_rel_.subscribe<_AlvarMarkers>("markers_in", 1, &ChuggTrackerNode::markerCallback, this);
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

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

    std::vector<tf::StampedTransform> output;
    
    tf::StampedTransform 
      marker_to_world_tf(marker_to_world, marker_to_world.stamp_, "/camera_link", "/chugg/pose/markers"),
      marker_to_world_quat( tf::Transform(marker_to_world.getRotation()), marker_to_world.stamp_, "/camera_link", 
			    "/chugg/ori/markers");
    output.push_back(marker_to_world_tf);
    output.push_back(marker_to_world_quat);
    
    br_.sendTransform(output);
  }

};

#endif // CHUGG_CHUGGTRACKER_CHUGGTRACKER
