/***************************************************************************
 *  include/chugg_tracker/marker_measurement_pdf.h
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


#ifndef CHUGG_CHUGGTRACKER_MARKERMEASUREMENTPDF
#define CHUGG_CHUGGTRACKER_MARKERMEASUREMENTPDF

// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <cmath>

/// BFL
#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>

namespace chugg
{

  class MarkerMeasurementPDF: public BFL::ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
   {
 
    private:
     BFL::Gaussian noise_;

    public:
     MarkerMeasurementPDF(BFL::Gaussian const & noise):
       /// Arg 1 is dimensions of the z in p(z | x). Arg 2 is the number of arguments after the |
       /// NOT the dimension of these arguments. I.E. we only pass one state vector, so we pass a 1.
       BFL::ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>(5, 1),
       noise_(noise)
       {}
     
     virtual ~MarkerMeasurementPDF(){};

     /** 
      * Since we don't directly measure velocity, PDF only incorporates the velocities of particles
      * implicitly. The idea is that if we make a prediction at the same instant as we measure
      * orientation, the accuracy of our velocity estimate is reflected in the accuracy of our orientation
      * estimate. Consider a simpler example with linear position and velocity: If our velocity estimate is too high
      * we would expect to predict a position greater than the measurement. If the velocity estimate is too low,
      * we would expect a position smaller than the measurement. arg(2) incorporates this idea by using the
      * distance between our orientation estimate and measurement divided by the dt between measurement updates. The idea here is
      * that the longer the time between measurement updates, the more the error in velocity will accumulate in our pose estimate.
      * Weighting by 1/dt removes this effect.
      * NOTE: For this to work, it's imperative that we make a prediction at the same instant as we incorporate a measurement.
      * 
      */
     virtual BFL::Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
     {
       /// Taking it for granted that argument 0 is actually state. docs are unclear on this
       MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
       
       double const dt = measurement(5);
       /// Interpret state as  quat: (w, x, y, z), twist: (x,y,z)
       tf::Quaternion ori( state(2), state(3), state(4), state(1));
       tf::Quaternion meas( measurement(2), measurement(3), measurement(4), measurement(1));
       tf::Vector3 vel( state(5), state(6), state(7));

       MatrixWrapper::ColumnVector arg(2);
       double const d = quatDistance(ori, meas);
       // ROS_INFO_STREAM("Ori: " << ori.getX() <<", " << ori.getY() << ", " << ori.getZ() << ", " << ori.getW() );
       // ROS_INFO_STREAM("Meas: " << meas.getX() <<", " << meas.getY() << ", " << meas.getZ() << ", " << meas.getW() );
       // ROS_INFO_STREAM("Dist: " << d );
       arg(1) = d;
       arg(2) = d/dt;
       // double const d2 = arg(2);
       BFL::Probability P = noise_.ProbabilityGet( arg );
       double p = P.getValue();
       return noise_.ProbabilityGet( arg );
     }

     /// implements distance metric on SO(3): arccos(|q1.q2])
     double quatDistance(tf::Quaternion const & q1, tf::Quaternion const & q2) const 
     {
       return acos( fabs( q1.dot(q2) ) );
     }
        
    };
    
} // chugg

#endif // CHUGG_CHUGGTRACKER_MARKERMEASUREMENTPDF
