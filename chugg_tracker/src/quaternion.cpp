/***************************************************************************
 *  src/quaternion.cpp
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


#include <chugg_tracker/quaternion.h>


namespace chugg
{
  tf::Quaternion quaternionAverage( std::vector<tf::Quaternion> const & samples)
  {
    using namespace Eigen;

    Matrix4d pp = MatrixXd::Zero(4,4);

    for( tf::Quaternion  const & quat : samples)
      {
	Vector4d q;
	q << quat.w(), quat.x(), quat.y(), quat.z();

	pp += q * q.transpose();
      }
    EigenSolver<Matrix4d> solver;

    solver.compute(pp, true);

    EigenSolver<Matrix4d>::EigenvalueType evals = solver.eigenvalues();
    EigenSolver<Matrix4d>::EigenvectorsType evecs = solver.eigenvectors();
    // ROS_INFO_STREAM( evals );
    // ROS_INFO_STREAM( evecs );

    bool found = false;
    size_t max_idx = 0;
    double max_val = -10000;
    for(size_t idx = 0; idx < 4; ++idx)
      {
	ROS_ASSERT( !evals[idx].imag() );
	double const real = evals[idx].real();
	// ROS_INFO_STREAM(real);
	if( real > max_val)
	  {
	    max_idx = idx;
	    max_val = real;
	    found = true;
	  }
      }
    ROS_ASSERT( found );

    Matrix<std::complex<double>, 4, 1> est = evecs.col(max_idx);

    tf::Quaternion const output = tf::Quaternion( est[1].real(), est[2].real(), est[3].real(), est[0].real());
    // ROS_INFO_STREAM( output.length() );
    // ROS_ASSERT( output.length() == 1.0);

    return output;
  }
}
