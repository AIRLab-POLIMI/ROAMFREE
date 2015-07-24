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
 * BasePriorEdgeInterface.h
 *
 *  Created on: Jun 12, 2013
 *      Author: davide
 */

#ifndef BASEPRIOREDGEINTERFACE_H_
#define BASEPRIOREDGEINTERFACE_H_

#include "g2o/core/optimizable_graph.h"

namespace ROAMestimation {

class BasePriorEdgeInterface {

public:
  virtual void setNoiseCov(const Eigen::MatrixXd &noiseCov) = 0;
  virtual void setMeasurement(const Eigen::VectorXd &meas) = 0;

  virtual void setCategory(const std::string &name) = 0;
  virtual const std::string &getCategory() const  = 0;

  virtual void setTimestamp(double timestamp) = 0;
  virtual double getTimestamp() const = 0;

  virtual g2o::OptimizableGraph::Edge *getg2oOptGraphPointer() = 0;

  virtual std::string writeDebugInfo() const = 0;

};

} /* namespace ROAMestimation */
#endif /* BASEPRIOREDGEINTERFACE_H_ */
