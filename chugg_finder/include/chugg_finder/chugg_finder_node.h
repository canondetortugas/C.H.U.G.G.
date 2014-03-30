/***************************************************************************
 *  include/chugg_finder/chugg_finder_node.h
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


#ifndef CHUGG_CHUGGFINDER_CHUGGFINDER
#define CHUGG_CHUGGFINDER_CHUGGFINDER

/// std
#include <random>

// ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>

/// point clouds

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
/// Necessary to deal with pcl::PointCloud<T> as message
#include <pcl_ros/point_cloud.h>

/// eigen
#include <Eigen/Geometry>

/// C.H.U.G.G.
#include <chugg_finder/ICPConfig.h>
#include <chugg_finder/ChuggFinderConfig.h>

typedef chugg_finder::ICPConfig _ICPConfig;
typedef chugg_finder::ChuggFinderConfig _ChuggFinderConfig;
typedef pcl::PointCloud<pcl::PointXYZ> _CloudXYZ;
typedef sensor_msgs::PointCloud2 _PointCloud2Msg;

class ChuggFinderNode: public BaseNode, public MultiReconfigure
{
private:
  ros::NodeHandle nh_rel_;
  _ICPConfig * icp_config_;
  _ChuggFinderConfig * config_;
  
  ros::Subscriber cloud_sub_;
  /// Publish point cloud of C.H.U.G.G.
  ros::Publisher chugg_pub_, debug_pub_;

  ////////////////////////////////////////////////////////////
  _CloudXYZ::Ptr chugg_cloud_;
  
public:
  ChuggFinderNode(): BaseNode("ChuggFinder"), nh_rel_("~")
  {
    chugg_cloud_ = boost::make_shared<_CloudXYZ>();
  }

private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    addReconfigureServer<_ICPConfig>("icp", &ChuggFinderNode::reconfigureCallbackICP, this );
    icp_config_ = &getLatestConfig<_ICPConfig>("icp");
    addReconfigureServer<_ChuggFinderConfig>("chugg_finder", &ChuggFinderNode::reconfigureCallback, this );
    config_ = &getLatestConfig<_ChuggFinderConfig>("chugg_finder");
    
    chugg_pub_ = nh_rel_.advertise<_CloudXYZ>("chugg_cloud", 1);
    debug_pub_ = nh_rel_.advertise<_CloudXYZ>("debug_cloud", 1);
    cloud_sub_ = nh_rel_.subscribe<_PointCloud2Msg>("cloud_in", 1, &ChuggFinderNode::pointCloudCallback, this);
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

  void reconfigureCallbackICP( _ICPConfig const & config )
  {
  }

  void reconfigureCallback( _ChuggFinderConfig const & config )
  {
    /// PCL uses float as default scalar
    typedef Eigen::Translation<float, 3> _Translation;

    /// Convert from inches to meters
    double const edge_length = config.edge_length * 0.0254;
    size_t const n_samples = config.side_samples;    

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution( -edge_length/2.0, edge_length/2.0);

    _CloudXYZ side_cloud;
    for( size_t idx = 0; idx < n_samples; ++idx)
      {
	double const x = distribution(generator);
	double const y = distribution(generator);

	pcl::PointXYZ point;
	point.x = x;
	point.y = y;
	point.z = 0;
	side_cloud.push_back(point);
      }

    /// The sides
    _CloudXYZ up, down, right, left, front, back;
    
    Eigen::Affine3f up_tf = Eigen::Affine3f::Identity() * _Translation(0,0,edge_length/2),
      down_tf = Eigen::Affine3f::Identity() *_Translation(0,0,-edge_length/2),
      right_tf = _Translation(0,edge_length/2,0) * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX() ),
      left_tf = _Translation(0,-edge_length/2,0) * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX() ),
      front_tf = _Translation(edge_length/2,0,0) * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitY() ),
      back_tf = _Translation(-edge_length/2,0,0) * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitY() );
      
    pcl::transformPointCloud(side_cloud, up, up_tf);
    pcl::transformPointCloud(side_cloud, down, down_tf);
    pcl::transformPointCloud(side_cloud, right, right_tf);
    pcl::transformPointCloud(side_cloud, left, left_tf);
    pcl::transformPointCloud(side_cloud, front, front_tf);
    pcl::transformPointCloud(side_cloud, back, back_tf);
    
    *chugg_cloud_ = up;
    *chugg_cloud_ += down;
    *chugg_cloud_ += right;
    *chugg_cloud_ += left;
    *chugg_cloud_ += front;
    *chugg_cloud_ += back;
  }
  
  void pointCloudCallback( _PointCloud2Msg::ConstPtr const & msg )
  {
    pcl::PCLPointCloud2 pcl_cloud;
    _CloudXYZ::Ptr cloud =  boost::make_shared<_CloudXYZ>();
    sensor_msgs::PointCloud2 chugg_output;

    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::fromPCLPointCloud2 (pcl_cloud, *cloud);

    if( cloud->size() < 1 )
      return;

    std::string const base_frame = cloud->header.frame_id;
    chugg_cloud_->header.frame_id = base_frame;

    /// Uses SVD internally for transformation estimation
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	    
    /// Estimate the transformation on our arm model that will align it with the sensor point cloud

    /// Setting CHUGG as the source means that we only look for correspondences for points on CHUGG
    /// i.e., for each point on CHUGG, we find the closest point in the sensor cloud
    /// but we don't try to find correspondences for all points in the sensor cloud
    icp.setInputSource( chugg_cloud_ );
    icp.setInputTarget( cloud );
    
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    // icp.setMaxCorrespondenceDistance (icp_config_->max_correspondence_distance);
    icp.setMaximumIterations (500);
    // icp.setTransformationEpsilon (icp_config_->transformation_epsilon);
    // icp.setEuclideanFitnessEpsilon (icp_config_->euclidian_fitness_epsilon);

    _CloudXYZ icp_output;

    /// Uses Identity as an initial guess for the transform
    icp.align( icp_output );

    ROS_INFO("ICP status: [converged: %d], [fitness: %f].", icp.hasConverged(), icp.getFitnessScore() );
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    
    // pcl::toPCLPointCloud2(chugg_cloud_, pcl_cloud);
    // pcl_conversions::fromPCL(pcl_cloud, chugg_output);
    if( icp.hasConverged())
      {
	chugg_pub_.publish(icp_output);
      }
    
    debug_pub_.publish(*chugg_cloud_);
  }

};

#endif // CHUGG_CHUGGFINDER_CHUGGFINDER
