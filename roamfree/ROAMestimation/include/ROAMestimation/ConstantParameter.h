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
 * ConstantParameter.h
 *
 *  Created on: May 16, 2013
 *      Author: davide
 */

#ifndef CONSTANTPARAMETER_H_
#define CONSTANTPARAMETER_H_

#include <boost/shared_ptr.hpp>

#include "GenericVertexInterface.h"
#include "g2o/core/optimizable_graph.h"

#include "ParameterVerticesManager.h"

namespace g2o {
class AutoIDSparseOptimizer;
}

namespace ROAMestimation {

class ConstantParameter: public ParameterVerticesManager {

  public:

    ConstantParameter(g2o::AutoIDSparseOptimizer * opt, ParameterTypes typ,
        const std::string &name, double t0, const Eigen::VectorXd & x0);

    ConstantParameter(g2o::AutoIDSparseOptimizer * opt, ParameterTypes typ,
        const std::string &name, GenericVertexInterface *vertex);

    virtual ~ConstantParameter() {
    }

    inline virtual std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator getVertices(
        double tstamp) const {
      return _v.begin();
    }

    inline virtual bool updateVertexSet(double mintstamp, double maxtstamp) {
      return true;
    }

    virtual bool getValueAt(double tstamp, Eigen::VectorXd &ret) const;

    inline virtual void resizeJacobianMatrix(Eigen::MatrixXd &ret) {
      ret.resize(1, 1); // constant parameter, single vertex, jacobian is the identity
    }

    virtual bool getJacobianAt(double tstamp, int j,
        Eigen::MatrixXd &ret) const;

    inline virtual int getWindowSize() const {
      return 1;
    }

    virtual void prepareForPoseRemoval(double mintstamp, double maxtstamp) {
    }
};

} /* namespace ROAMestimation */
#endif /* CONSTANTPARAMETER_H_ */
