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
 * GenericVertexInterface.h
 *
 *  Created on: Jun 4, 2013
 *      Author: davide
 */

#ifndef GENERICVERTEXINTERFACE_H_
#define GENERICVERTEXINTERFACE_H_

#include <string>

#include <Eigen/Dense>
#include "g2o/core/optimizable_graph.h"

namespace ROAMestimation {

class GenericVertexInterface {

public:

  virtual ~GenericVertexInterface () {

  }

  virtual double getTimestamp() const = 0;
  virtual void setTimestamp(double t) = 0;

  virtual const std::string &getCategory() const = 0;
  virtual void setCategory(const std::string &category) = 0;

  virtual const Eigen::VectorXd &getEstimate() const = 0;
  virtual void setEstimate(const Eigen::VectorXd &) = 0;

  // TODO: ugly tweak to overcome the bad double inheritance structure
  virtual g2o::OptimizableGraph::Vertex *getg2oOptGraphPointer() = 0;

};

} /* namespace ROAMestimation */
#endif /* GENERICVERTEXINTERFACE_H_ */
