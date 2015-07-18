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
 * AutoIDSparseOptimizer.h
 *
 *  Created on: May 17, 2013
 *      Author: davide
 *
 *      this extends SparseOptimizer so that the vertices
 *      id are handled transparently each time a new vertex
 *      is added to the graph
 *
 */

#ifndef AUTOIDSPARSEOPTIMIZER_H_
#define AUTOIDSPARSEOPTIMIZER_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/graph_optimizer_sparse.h"

namespace g2o {

class AutoIDSparseOptimizer: public SparseOptimizer {

protected:
  int _maxId;

public:
  AutoIDSparseOptimizer();

  virtual bool addVertex(OptimizableGraph::Vertex* v, Data* userData = 0);

};

} /* namespace g2o */
#endif /* AUTOIDSPARSEOPTIMIZER_H_ */
