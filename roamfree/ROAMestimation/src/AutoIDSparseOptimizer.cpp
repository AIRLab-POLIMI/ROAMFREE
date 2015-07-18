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
 * AutoIDSparseOptimizer.cpp
 *
 *  Created on: May 17, 2013
 *      Author: davide
 */

#include "AutoIDSparseOptimizer.h"

namespace g2o {

AutoIDSparseOptimizer::AutoIDSparseOptimizer() : _maxId(0) {

}

bool AutoIDSparseOptimizer::addVertex(OptimizableGraph::Vertex* v,
    Data* userData) {

  v->setId(_maxId++);
  return SparseOptimizer::addVertex(v,userData);
}

} /* namespace g2o */

