/***************************************************************************
 *  include/chugg_tracker/system_pdf_constant_velocity.h
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


#ifndef CHUGG_CHUGGTRACKER_SYSTEMPDFCONSTANTVELOCITY
#define CHUGG_CHUGGTRACKER_SYSTEMPDFCONSTANTVELOCITY

// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

/// BFL
#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>

#include <model/systemmodel.h>
#include <model/measurementmodel.h>

namespace chugg
{

  class SystemPDFConstantVelocity: public BFL::ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
  {
  private:
    BFL::Gaussian ori_noise_;
    
  public:
    SystemPDFConstantVelocity(BFL::Gaussian const & ori_noise):
      /// Args: System dimensions (7), Conditional arguments (2) (1 for previous state, 1 for dummy control input)
      BFL::ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>(7, 2),
      ori_noise_(ori_noise)
    {}
    ~SystemPDFConstantVelocity(){};

    /**
     * This is a system model with no inputs. However, we use a dummy input to pass the dt since the last
     * prediction in
     */
    virtual bool SampleFrom (BFL::Sample<MatrixWrapper::ColumnVector>& one_sample, int method=DEFAULT, void * args=NULL) const
    {
      /// Inherited from ConditionalPDF. 0th argument is always old state. arg 1 is "control input" 
      MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
      MatrixWrapper::ColumnVector input = ConditionalArgumentGet(1);

      double const dt = input(1);
      /// Interpret state as  quat: (w, x, y, z), twist: (x,y,z)
      tf::Vector3 const vel(state(5), state(6), state(7));
      double const angle = vel.length()*dt;
      /// Change in quaternion due to velocity
      tf::Quaternion const deltaq = tf::Quaternion( vel.normalized(), angle);

      tf::Quaternion const ori = stateToQuat(state);
      /// Take current orientation and integrate velocity (deltaq), then perturb by gaussian noise
      tf::Quaternion const noisy = ori * deltaq * sampleQuat();
      // tf::Quaternion const noisy = ori * sampleQuat();

      state(1) = noisy.getW();
      state(2) = noisy.getX();
      state(3) = noisy.getY();
      state(4) = noisy.getZ();

      MatrixWrapper::ColumnVector const vel_noise = getVelocityNoise();
      state(5) += vel_noise(1);
      state(6) += vel_noise(2);
      state(7) += vel_noise(3);
       
      one_sample.ValueSet(state);

      /// the internet said I should do this, not sure why
      return true;
    }

  private:

    inline tf::Quaternion stateToQuat(MatrixWrapper::ColumnVector const & state) const
    {
      /// MatrixWrapper classes are indexed starting at 0
      return tf::Quaternion(state(2), state(3), state(4), state(1));
    }

    tf::Quaternion sampleQuat() const
    {
      /// first 3 values are from distribution on orientation
      BFL::Sample<MatrixWrapper::ColumnVector> noise;
      ori_noise_.SampleFrom(noise, DEFAULT, NULL);
      MatrixWrapper::ColumnVector ncv = noise.ValueGet();
      tf::Vector3 nv( ncv(1), ncv(2), ncv(3));
      double norm = nv.length();
       
      /// Creates quaternion from axis and angle
      return tf::Quaternion( nv.normalized(), norm );
    }

    MatrixWrapper::ColumnVector getVelocityNoise() const
    {
      BFL::Sample<MatrixWrapper::ColumnVector> noise;
      ori_noise_.SampleFrom(noise, DEFAULT, NULL);
      MatrixWrapper::ColumnVector ncv = noise.ValueGet();
       
      MatrixWrapper::ColumnVector output(3);
       
      output(1) = ncv(4);
      output(2) = ncv(5);
      output(3) = ncv(6);
      return output;
    }
     
  };
    
} // chugg

#endif // CHUGG_CHUGGTRACKER_SYSTEMPDFCONSTANTVELOCITY
