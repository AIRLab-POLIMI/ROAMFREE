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
 * LimitedBandwithParameter.h
 *
 *  Created on: May 16, 2013
 *      Author: davide
 */

#ifndef LIMITEDBANDWITHPARAMETER_H_
#define LIMITEDBANDWITHPARAMETER_H_

#include "g2o/core/optimizable_graph.h"

#include "ParameterVerticesManager.h"

namespace g2o {
class AutoIDSparseOptimizer;
}

namespace ROAMestimation {

class LimitedBandwithEuclideanParameter: public ParameterVerticesManager {

  protected:

    double _bandwith;
    int _a;

    // stuff for initialization

    Eigen::VectorXd _x0;

  public:

    LimitedBandwithEuclideanParameter(double bandwith, int a,
        g2o::AutoIDSparseOptimizer * opt, ParameterTypes typ,
        const std::string &name, const Eigen::VectorXd &x0);

    virtual ~LimitedBandwithEuclideanParameter() {
    }

    virtual std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator getVertices(
        double tstamp) const;
    virtual bool updateVertexSet(double mintstamp, double maxtstamp);

    virtual bool getValueAt(double tstamp, Eigen::VectorXd &ret) const;

    inline virtual void resizeJacobianMatrix(Eigen::MatrixXd &ret) {
      // Lanczos window interpolation, the jacobian is diagonal and each
      // element is L(t_vertex)
      ret.resize(1, 1);
    }

    virtual bool getJacobianAt(double tstamp, int j,
        Eigen::MatrixXd &ret) const;

    inline virtual int getWindowSize() const {
      return 2 * _a;
    }

    virtual void prepareForPoseRemoval(double mintstamp, double maxtstamp);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ;
};

} /* namespace ROAMestimation */
#endif /* LIMITEDBANDWITHPARAMETER_H_ */
