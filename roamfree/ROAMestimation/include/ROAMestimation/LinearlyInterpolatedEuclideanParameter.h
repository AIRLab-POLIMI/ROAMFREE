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
 * LinearlyInterpolatedEuclideanParameter.h
 *
 *  Created on: Apr 27, 2015
 *      Author: davide
 */

#ifndef ROAMESTIMATION_LINEARLYINTERPOLATEDEUCLIDEANPARAMETER_H_
#define ROAMESTIMATION_LINEARLYINTERPOLATEDEUCLIDEANPARAMETER_H_

#include "g2o/core/optimizable_graph.h"

#include "ParameterVerticesManager.h"

namespace g2o {
class AutoIDSparseOptimizer;
}

namespace ROAMestimation {

class LinearlyInterpolatedEuclideanParameter: public ParameterVerticesManager {

  public:

    LinearlyInterpolatedEuclideanParameter(double spacing,
        g2o::AutoIDSparseOptimizer * opt, ParameterTypes typ,
        const std::string &name, const Eigen::VectorXd &x0);

    virtual ~LinearlyInterpolatedEuclideanParameter() {
    }

    virtual std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator getVertices(
        double tstamp) const;

    virtual bool updateVertexSet(double mintstamp, double maxtstamp);

    virtual bool getValueAt(double tstamp, Eigen::VectorXd &ret) const;

    inline virtual void resizeJacobianMatrix(Eigen::MatrixXd &ret) {
      ret.resize(1, 1);
    }

    virtual bool getJacobianAt(double tstamp, int j,
        Eigen::MatrixXd &ret) const;

    inline virtual int getWindowSize() const {
      return 2;
    }

    virtual void prepareForPoseRemoval(double mintstamp, double maxtstamp);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ;

  protected:

    double _spacing; /*!< the distance in time between two vertices */

    Eigen::VectorXd _x0; /*!< initial value for the parameter */

}
;

} /* namespace ROAMestimation */

#endif /* ROAMESTIMATION_LINEARLYINTERPOLATEDEUCLIDEANPARAMETER_H_ */
