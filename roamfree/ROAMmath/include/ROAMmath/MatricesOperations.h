/*
Copyright (c) 2013-2014 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (cucci@elet.polimi.it)
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
  if(lu_decomp.isInvertible())
  {
    //compute the inverse
    B = lu_decomp.inverse();
  }else {
    std::cout << "ERROR IN INVERSE MATRIX COMPUTATION ! Matrix is NOT invertible" << std::endl ;
    std::cout << A << std::endl;
    assert(0);
  }
}

}

#endif /* MATRICESOPERATIONS_H_ */
