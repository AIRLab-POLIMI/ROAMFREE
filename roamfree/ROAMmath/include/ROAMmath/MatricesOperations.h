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
 * MatricesOperations.h
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#ifndef MATRICESOPERATIONS_H_
#define MATRICESOPERATIONS_H_

#include <cassert>
#include <iostream>

namespace ROAMmath {

// TODO: this is a major cause of code bloat since it causes specialization of a number of templates.

template<typename T1, typename T2>
void inv(T1 & A, T2 & B) {
  Eigen::FullPivLU<T1> lu_decomp(A);

  //check on invertible flag
  if (lu_decomp.isInvertible()) {
    //compute the inverse
    B = lu_decomp.inverse();
  } else {
    std::cout << "[ROAMmath::inv] input matrix is NOT invertible" << std::endl;
    std::cout << A << std::endl;
    assert(0);
  }
}

template<typename T1, typename T2>
void invDiagonal(T1 & A, T2 & B) {
  assert(B.rows() == A.rows() && B.cols() == A.cols());

# ifdef DEBUG_BUILD
    for (int r = 0; r < A.rows(); r++) {
      for (int c = 0; c < A.cols(); c++) {
        if (r != c && A(r, c) != 0.0) {
          std::cerr
              << "[ROAMmath::invDiagonal] called with a non diagonal input matrix"
              << std::endl;
          std::cerr << A << std::endl;
          assert(0);
        }
      }
    }
# endif

  B.setZero();
  for (int i = 0; i < A.rows(); i++) {
    B(i, i) = 1.0 / A(i, i);
  }
}

}

#endif /* MATRICESOPERATIONS_H_ */
