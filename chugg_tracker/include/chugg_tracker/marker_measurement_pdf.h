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

namespace chugg
{

  class MarkerMeasurementPDF: public BFL::ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
   {
 
    private:
     BFL::Gaussian noise_;
    
    public:
     MarkerMeasurementPDF(BFL::Gaussian const & noise):
       /// 7 arguments for state, 4 arguments for orientation measurement
       /// The measurement is conditioned on the state vector, which is why the
       /// measurement dimension is given first.
       BFL::ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>(4, 7),
       noise_(noise)
       {}
     
     virtual ~MarkerMeasurementPDF();

     // implement this virtual function for measurement model of a particle filter
     virtual BFL::Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
     {
       /// Taking it for granted that argument 0 is actually state. docs are unclear on this
       MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
       
       /// Interpret state as  quat: (w, x, y, z), twist: (x,y,z)
       tf::Quaternion ori( state(2), state(3), state(4), state(1));
       tf::Quaternion meas( measurement(2), measurement(3), measurement(4), measurement(1));
       tf::Vector3 vel( state(5), state(6), state(7));
       
       /// TODO: Velocity probability: Gaussian on difference between velocity and difference between state and measurement.
       /// Need delta t between last update and measurement to deal with this
       
     }
        
    };
    
} // chugg

#endif // CHUGG_CHUGGTRACKER_MARKERMEASUREMENTPDF
