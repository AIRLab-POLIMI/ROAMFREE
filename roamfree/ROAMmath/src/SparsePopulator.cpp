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
 * SparsePopulator.cpp
 *
 *  Created on: Mar 28, 2013
 *      Author: davide
 */

#include "SparsePopulator.h"

namespace ROAMmath {

SparsePopulator::SparsePopulator(int rows, int cols) :
    SparseMatrix<double>(rows, cols) {
}

} /* namespace ROAMmath */
