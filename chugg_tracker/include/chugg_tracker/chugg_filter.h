/***************************************************************************
 *  include/chugg_tracker/chugg_filter.h
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


#ifndef CHUGG_CHUGGTRACKER_CHUGGFILTER
#define CHUGG_CHUGGTRACKER_CHUGGFILTER

/// uscauv
#include <uscauv_common/multi_reconfigure.h>

// ROS
#include <ros/ros.h>

/// BFL
#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include <filter/bootstrapfilter.h>

/// CHUGG
#include <chugg_tracker/ChuggFilterConfig.h>
#include <chugg_tracker/system_pdf_constant_velocity.h>

namespace chugg
{
  
  /// TODO: Add a service to reset the filter's state
  class ChuggFilter: public MultiReconfigure
  {
  private:
    typedef chugg_tracker::ChuggFilterConfig _Config;
    typedef BFL::SystemModel<MatrixWrapper::ColumnVector> _SystemModel;
    typedef BFL::Measurement<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> _MeasurementModel;
    
  private:
    _Config * config_;
    
    double dt_;

    std::shared_ptr<chugg::SystemPDFConstantVelocity> system_pdf_;
    std::shared_ptr<_SystemModel> system_;

    std::shared_ptr<chugg::MarkerMeasurementPDF> marker_measurement_pdf_;
    std::shared_ptr<_MeasurementModel> marker_measurement_;
    
  public:
    ChuggFilter(double const & dt): dt_(dt)
    {
      addReconfigureServer<_Config>("filter", &ChuggFilter::reconfigureCallback, this);
      config_ = &getLatestConfig<_Config>("filter");

      initalizeFilter();
    }
    
    /// make a prediction based on constant velocity model
    void predict()
    {
      /// TODO: Call bootstrap filter predict using system_pdf_
    }

    /// Incorporate measurement from ar marker tracker
    void updateMarkers(tf::Quaternion const & ori)
    {
      
    }
     
    /// Get a list of samples from the posterior distribution
    std::shared_ptr<std::vector<tf::Quaternion> > getPosterior()
    {
    }

  private:
    void reconfigureCallback( _Config const & config )
    {

      ////////////////////////////////////////////////////////
      // Set up system PDF////////////////////////////////////
      ////////////////////////////////////////////////////////

      /// Use 3D gaussian to sample rotations - even though we represent them as 4D quaternions they only have 3 DOF
      
      MatrixWrapper::ColumnVector ori_system_noise_mean(6);
      ori_system_noise_mean = 0.0;

      /// First half is covariance for orientation, second is for angular velocity. They are uncorrelated
      MatrixWrapper::SymmetricMatrix ori_system_noise_cov(6);
      ori_system_noise_cov = 0.0;
      ori_system_noise_cov(1,1) = config.system_ori_cov;
      ori_system_noise_cov(2,2) = config.system_ori_cov;
      ori_system_noise_cov(3,3) = config.system_ori_cov;
      ori_system_noise_cov(4,4) = config.system_ori_vel_cov;
      ori_system_noise_cov(5,5) = config.system_ori_vel_cov;
      ori_system_noise_cov(6,6) = config.system_ori_vel_cov;

      BFL::Gaussian ori_system_noise(ori_system_noise_mean, ori_system_noise_cov);

      if( !system_pdf_ )
	system_pdf_ = std::make_shared<chugg::SystemPDFConstantVelocity>( ori_system_noise, dt_);
      else
	*system_pdf_ = chugg::SystemPDFConstantVelocity( ori_system_noise, dt_ );

      if( !system_ )
	system_ = std::make_shared<_SystemModel>( system_pdf_.get() );
      else
	*system_ = _SystemModel( system_pdf_.get() );

      ////////////////////////////////////////////////////////
      /// Set up measurement PDF
      ////////////////////////////////////////////////////////
      MatrixWrapper::ColumnVector marker_measurement_mean(2);
      marker_measurement_mean = 0.0;

      MatrixWrapper::SymmetricMatrix marker_measurement_cov(2);
      marker_measurement_cov = 0.0;
      marker_measurement_cov(1,1) = config.marker_ori_cov;
      marker_measurement_cov(2,2) = config.marker_ori_vel_cov;

      BFL::Gaussian marker_measurement_noise(marker_measurement_mean, marker_measurement_cov);

      if( !marker_measurement_pdf_ )
	marker_measurement_pdf_ = std::make_shared<chugg::MarkerMeasurementPDF>( marker_measurement_noise );
      else
	*marker_measurement_pdf_ = chugg::MarkerMeasurementPDF( marker_measurement_noise );
      if( !marker_measurement_ )
	marker_measurement_ = std::make_shared<_MeasurementModel>( marker_measurement_pdf_.get() );
      else
	*marker_measurement_ = _MeasurementModel( marker_measurement_pdf_.get() );
      
      
    }

    void initializeFilter()
    {
      /// TODO: Set prior and initialize bootstrap filter

    }

  };
    
} // chugg

#endif // CHUGG_CHUGGTRACKER_CHUGGFILTER
