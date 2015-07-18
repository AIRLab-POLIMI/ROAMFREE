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
 * g2oSolverFactory.cpp
 *
 *  Created on: Nov 22, 2012
 *      Author: davide
 */

#include "g2oSolverFactory.h"

namespace ROAMestimation {

g2o::AutoIDSparseOptimizer* ROAMestimation::g2oSolverFactory::getNewSolver() {

  g2o::AutoIDSparseOptimizer *optimizer = new g2o::AutoIDSparseOptimizer;

  g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType> *linearsolver =
      new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX *blocksolver = new g2o::BlockSolverX(optimizer,
      linearsolver);

  optimizer->setSolver(blocksolver);

  return optimizer;
}

}

/* namespace ROAMestimation */
