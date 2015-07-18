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
 * g2oSolverFactory.h
 *
 *  Created on: Nov 22, 2012
 *      Author: davide
 */

#ifndef G2OSOLVERFACTORY_H_
#define G2OSOLVERFACTORY_H_

#include "AutoIDSparseOptimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"


namespace ROAMestimation {

class g2oSolverFactory {

public:
  static g2o::AutoIDSparseOptimizer *getNewSolver();

};

} /* namespace ROAMestimation */
#endif /* G2OSOLVERFACTORY_H_ */
