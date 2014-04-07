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
#include <chugg_tracker/ChuggFilterConfig.h>

/// CHUGG
#include <chugg_tracker/system_pdf_constant_velocity.h>
#include <chugg_tracker/marker_measurement_pdf.h>

#include <chugg_tracker/Posterior.h>

typedef chugg_tracker::Posterior _Posterior;
typedef chugg_tracker::Sample  _Sample;

/// NOTE: BFL absolutely needs to be included after ROS packages
/// BFL pollutes the namespace and will interfere with these packages if it is
/// included before them

/// BFL
#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <pdf/uniform.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

#include <filter/bootstrapfilter.h>

namespace chugg
{
  
  /// BFL linear algebra library
  using namespace MatrixWrapper;

  /// TODO: Add a service to reset the filter's state
  class ChuggFilter: public MultiReconfigure
  {
  private:
    typedef chugg_tracker::ChuggFilterConfig _Config;
    typedef BFL::SystemModel<MatrixWrapper::ColumnVector> _SystemModel;
    typedef BFL::MeasurementModel<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> _MeasurementModel;
    typedef BFL::BootstrapFilter<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> _Filter;
    
  private:
    _Config config_;
    
    std::shared_ptr<chugg::SystemPDFConstantVelocity> system_pdf_;
    std::shared_ptr<_SystemModel> system_;

    std::shared_ptr<chugg::MarkerMeasurementPDF> marker_measurement_pdf_;
    std::shared_ptr<_MeasurementModel> marker_measurement_;

    std::shared_ptr<BFL::MCPdf<ColumnVector> > prior_;

    std::shared_ptr<_Filter> filter_;

    ros::Time last_predict_time_, last_marker_update_time_;

    size_t last_sample_count_;
    
  public:
    ChuggFilter(): last_predict_time_( ros::Time::now()), last_marker_update_time_( ros::Time::now() ),
		   last_sample_count_(0)
    {
      addReconfigureServer<_Config>("filter", &ChuggFilter::reconfigureCallback, this);
    }
    
    /// make a prediction based on constant velocity model
    void predict()
    {
      ros::Time now = ros::Time::now();
      double const dt = (now - last_predict_time_).toSec();
      last_predict_time_ = now;
      
      ColumnVector input(1);
      input(1) = dt;
      filter_->Update( system_.get(), input);
    }

    /// Incorporate measurement from ar marker tracker
    void updateMarkers(tf::Quaternion const & ori)
    {
      ros::Time now = ros::Time::now();
      double const measurement_dt = (now - last_marker_update_time_).toSec();
      last_marker_update_time_ = now;

      double const predict_dt = (now - last_predict_time_).toSec();
      last_predict_time_ = now;
      
      ColumnVector measurement(5);
      measurement(1) = ori.getW();
      measurement(2) = ori.getX();
      measurement(3) = ori.getY();
      measurement(4) = ori.getZ();
      measurement(5) = measurement_dt;

      ColumnVector input(1);
      input(1) = predict_dt;

      filter_->Update( system_.get(), input, marker_measurement_.get(), measurement);
    }
     
    /// Get a list of samples from the posterior distribution
    std::shared_ptr<_Posterior> getPosterior()
    {
      typedef std::vector< BFL::WeightedSample<ColumnVector> > _SampleVec;

      std::shared_ptr<_Posterior > output = 
	std::make_shared<_Posterior >();
      
      BFL::MCPdf<ColumnVector> * posterior = filter_->PostGet();
      
      std::vector< BFL::WeightedSample<ColumnVector> > const & samples = posterior->ListOfSamplesGet();

      for(_SampleVec::const_iterator sample_it = samples.begin(); sample_it != samples.end(); ++sample_it)
	{
	  ColumnVector const & sample = sample_it->ValueGet();

	  _Sample msg;
	  msg.ori.w = sample(1);
	  msg.ori.x = sample(2);
	  msg.ori.y = sample(3);
	  msg.ori.z = sample(4);
	    
	  msg.vel.x = sample(5);
	  msg.vel.y = sample(6);
	  msg.vel.z = sample(7);
	  
	  output->samples.push_back(msg);
	}

      return output;
    }

  private:
    void reconfigureCallback( _Config const & config )
    {
      config_ = config;

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

      system_pdf_.reset();
      system_pdf_ = std::make_shared<chugg::SystemPDFConstantVelocity>( ori_system_noise);

      system_.reset();
      system_ = std::make_shared<_SystemModel>( system_pdf_.get() );

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

      marker_measurement_pdf_.reset();
      marker_measurement_pdf_ = std::make_shared<chugg::MarkerMeasurementPDF>( marker_measurement_noise );

      marker_measurement_.reset();
      marker_measurement_ = std::make_shared<_MeasurementModel>( marker_measurement_pdf_.get() );
      
      /// Reset the filter with the new value for N samples
      if( last_sample_count_ != config.samples )
	initializeFilter();
      
    }

    /// TODO: Sample orientation in a way that isn't biased
    void initializeFilter()
    {
      using namespace MatrixWrapper;

      ////////////////////////////////////////////////////////
      // Initialize prior distribution on orientation////////
      ////////////////////////////////////////////////////////
      ColumnVector mean(3);
      /// Ordering: Roll [0, 2pi], pitch [-pi/2, pi/2], yaw [0, 2pi]
      mean(1) = M_PI;
      mean(2) = 0.0;
      mean(3) = M_PI;

      ColumnVector width(3);
      width(1) = 2*M_PI;
      width(2) = M_PI;
      width(3) = 2*M_PI;

      BFL::Uniform ori_prior(mean, width);

      ////////////////////////////////////////////////////////
      // Initialize prior on angular vel.////////////////////
      ////////////////////////////////////////////////////////
      ColumnVector vel_mean(3);
      vel_mean = 0.0;
      
      SymmetricMatrix vel_cov(3);
      vel_cov = 0.0;
      vel_cov(1, 1) = config_.prior_ori_vel_cov;
      vel_cov(2, 2) = config_.prior_ori_vel_cov;
      vel_cov(3, 3) = config_.prior_ori_vel_cov;
      
      BFL::Gaussian vel_prior(vel_mean, vel_cov);
      
      prior_.reset();
      prior_ = std::make_shared<BFL::MCPdf<ColumnVector> >(config_.samples, 7);


      /// What we do here: Get N RPY samples from a uniform distribution and N angular vel samples from a
      /// gaussian distribution. We then convert the RPY samples into quats, combine these and the vel samples
      /// into single vectors, then use these vectors to initialize the prior distribution
    
      std::vector<BFL::Sample<ColumnVector> > ori_samples(config_.samples), vel_samples( config_.samples), 
	combined_samples(config_.samples);
    
      ori_prior.SampleFrom(ori_samples, config_.samples, DEFAULT);
      vel_prior.SampleFrom(vel_samples, config_.samples, DEFAULT);
      
      for( size_t idx = 0; idx < config_.samples; ++idx )
	{
	  BFL::Sample<ColumnVector> sample(7);
	  ColumnVector sample_val(7);

	  ColumnVector const & ori_sample = ori_samples[idx].ValueGet();
	  ColumnVector const & vel_sample = vel_samples[idx].ValueGet();

	  tf::Quaternion ori_quat;
	  ori_quat.setRPY( ori_sample(1), ori_sample(2), ori_sample(3) );
	  
	  sample_val(1) = ori_quat.getW();
	  sample_val(2) = ori_quat.getX();
	  sample_val(3) = ori_quat.getY();
	  sample_val(4) = ori_quat.getZ();

	  sample_val(5) = vel_sample(1);
	  sample_val(6) = vel_sample(2);
	  sample_val(7) = vel_sample(3);
	  
	  sample.ValueSet(sample_val);
	  combined_samples[idx] = sample;
	}
    
      prior_->ListOfSamplesSet(combined_samples);

      /// I believe that the third parameter is the number of effective samples that we have to drop below
      /// before we resample
      filter_.reset();
      filter_ = std::make_shared<_Filter>( prior_.get(), 0, double(config_.samples/4.0), DEFAULT_RS);
      
    }
    
  };
  
} // chugg

#endif // CHUGG_CHUGGTRACKER_CHUGGFILTER
