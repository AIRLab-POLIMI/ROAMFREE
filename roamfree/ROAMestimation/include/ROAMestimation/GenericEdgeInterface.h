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
 * GenericEdgeInterface.h
 *
 *  Created on: Nov 7, 2013
 *      Author: davide
 */

#ifndef GENERICEDGEINTERFACE_H_
#define GENERICEDGEINTERFACE_H_

#include <Eigen/Dense>
#include "ParameterTemporaries.h"

#include "BaseEdgeInterface.h"

namespace ROAMestimation {

class GenericEdgeInterface : public BaseEdgeInterface {

public:
  virtual const Eigen::MatrixXd & getNoiseCov() const = 0;

  virtual int noiseDimension() const = 0;

  /**
   * \brief get the measurement vector
   */
  virtual const Eigen::VectorXd& getMeasurement() const = 0;

  /**
   * \brief update the local copy of the function parameters current estimate and their jacobian, if needed
   */
  virtual void handleParamTemporaries() = 0;

  /**
   * \brief update the shortcut pointers to be passed to F error and errorJacobian
   */
  virtual void updateParamPtrs() = 0;
  virtual const std::vector<ParameterTemporaries> & getParameterTemporariesVector() const = 0;

  /**
   * \brief get the edge associated augmented state
   *
   * each GenericEdge in roamfree has an augmented state, which means
   * x, q, v, w, a, alpha, in reals^19
   */
  virtual const Eigen::VectorXd &getAugmentedState() const = 0;

  /**
   * \brief get the order of the generc edge
   *
   * The return value refers to the order of the finite differences
   * between poses which is required to evaluate the relevant parts of the
   * augmented state estimator.
   *
   * -1 -> not meaningful (i.e. constraints which do not need finite differences nor augmented state)
   * 0  -> pose and orientation only constraints
   * 1  -> also velocities
   * 2  -> also accelerations
   *
   */
  inline virtual int getOrder() const {
    return -1;
  }

  /**
   * \brief get the state componets observed by the sensor function
   *
   * it returns the array of bool defined in the corresponding ROAMfunction element
   */
  virtual const bool *getUsedComponents() const = 0;
};

}

#endif /* GENERICEDGEINTERFACE_H_ */
