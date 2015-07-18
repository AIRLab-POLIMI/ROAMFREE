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
* ParameterTemporaries.h
 *
 *  Created on: May 20, 2013
 *      Author: davide
 */

#ifndef PARAMETERTEMPORARIES_H_
#define PARAMETERTEMPORARIES_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "ParameterVerticesManager.h"

namespace ROAMestimation {

class ParameterTemporaries {

public:
  boost::shared_ptr<ParameterVerticesManager> p;
  Eigen::VectorXd value;
  std::vector<Eigen::MatrixXd> jacobians;

  ParameterTemporaries(boost::shared_ptr<ParameterVerticesManager> inp, double ts);
  void updateTemporaries();

protected:

  double _tstamp;

  bool _hasToUpdateValue;
  bool _hasToUpdateJacobian;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} /* namespace ROAMestimation */
#endif /* PARAMETERTEMPORARIES_H_ */
