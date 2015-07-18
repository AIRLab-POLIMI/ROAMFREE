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
 * SparsePopulator.h
 *
 *  Created on: Mar 28, 2013
 *      Author: davide
 */

#ifndef SPARSEPOPULATOR_H_
#define SPARSEPOPULATOR_H_

#include <Eigen/Sparse>

namespace ROAMmath {

// this class is used to mask the fact that Eigen::SparseMatrix does not have the () operator.

// TODO: accessing elements trough (i,j) involves binary searches in Eigen::SparseMatrix
// this could be a major drawback. Still, most of my Jacobian matrices are full of zeros..

class SparsePopulator: public Eigen::SparseMatrix<double> {
public:

  SparsePopulator(int rows, int cols);

  inline double &operator ()(int i, int j) {
    return coeffRef(i, j);
  }
};

} /* namespace ROAMfunctions */
#endif /* SPARSEPOPULATOR_H_ */
