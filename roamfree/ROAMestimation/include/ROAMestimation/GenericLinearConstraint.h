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
 * GenericLinearConstraint.h
 *
 *  Created on: Sep 25, 2013
 *      Author: davide
 */

#ifndef GENERICLINEARCONSTRAINT_H_
#define GENERICLINEARCONSTRAINT_H_

#include <Eigen/Dense>

#include "g2o/core/base_multi_edge.h"

namespace ROAMestimation {

/*
 * \brief Generic Linear Constraint edge, to handle node marginalization
 *
 * as described in
 * "Generic Factor-Based Node Marginalization and Edge Sparsification for Pose-Graph SLAM"
 * Nicholas Carlevaris-Bianco and Ryan M. Eustice
 *
 * but also taking into account the fact that state variables belong to manifolds
 *
 * usage:
 * 1 - call the resize(n_vertices) method
 * 2 - fill the _vertices array
 * 3 - fill in _G with accessGain() method
 * 4 - call the resizeStructures() method
 * 5 - fill in _measurement
 *
 */

class GenericLinearConstraint: public g2o::BaseMultiEdge<-1,
    Eigen::VectorXd>
{

public:

  GenericLinearConstraint();

  void computeError();
  void linearizeOplus();

  /*
   * \brief get a non-const reference to the G matrix
   *
   * G is a gain matrix which is to be multiplied to the vector z^(-1) * x
   * so that the edge information matrix is full rank, as described in the paper
   */

  Eigen::MatrixXd &accessGain();

  /*
   * \brief resizes internal error, measurement and jacobian matrices
   */

  void resizeStructures();

  std::string writeDebugInfo() const;

  bool read(std::istream &s);
  bool write(std::ostream &s) const;

protected:
  Eigen::MatrixXd _G;
  Eigen::VectorXd _fullError;

  static const int _OFF = -1;

};

} /* namespace ROAMestimation */
#endif /* GENERICLINEARCONSTRAINT_H_ */
