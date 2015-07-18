/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)    
*/

/*
 * OrientationContinuity.h
 *
 *  Created on: Apr 26, 2013
 *      Author: davide
 */

#ifndef ORIENTATIONCONTINUITYEDGE_H_
#define ORIENTATIONCONTINUITYEDGE_H_

#include <Eigen/Dense>

#include "g2o/core/base_binary_edge.h"

#include "GenericVertex.h"
#include "ROAMfunctions/SE3V.h"

namespace ROAMestimation {

class OrientationContinuityEdge: public g2o::BaseBinaryEdge<3, Eigen::VectorXd,
    GenericVertex<ROAMfunctions::SE3V>, GenericVertex<ROAMfunctions::SE3V> > {

public:

  OrientationContinuityEdge();

  void preIteration();

  void computeError();
  void linearizeOplus();

  Eigen::Matrix3d &accessNoiseCov();

  bool read(std::istream &s);
  bool write(std::ostream &s) const;

protected:
  static const int _OFF = -1;
  double sign;

  Eigen::Matrix3d _noiseCov;

};

} /* namespace ROAMpython */
#endif /* ORIENTATIONCONTINUITY_H_ */
