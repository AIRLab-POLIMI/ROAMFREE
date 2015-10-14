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
 * ParameterBlender.h
 *
 *  Created on: Aug 3, 2015
 *      Author: davide
 */

#ifndef PARAMETERBLENDER_H_
#define PARAMETERBLENDER_H_

#include "ParameterVerticesManager.h"

namespace ROAMestimation {

class ParameterBlender: public ParameterVerticesManager {
  public:
    ParameterBlender(g2o::AutoIDSparseOptimizer * opt, ParameterTypes type,
        const std::string& name,
        std::vector<ParameterVerticesManager *> toblend);

    virtual void getVerticesPointers(double tstamp,
        std::vector<g2o::HyperGraph::Vertex *> & to,
        int freePosition = 0) const;

    virtual void setFixed(bool isfixed);

    virtual void setComputeCovariance(bool computeCovariance);

    /**
     * \brief returns true if ALL the blended parameters are fixed
     */
    virtual bool fixed() const;

    virtual bool getValueAt(double tstamp, Eigen::VectorXd &ret) const;

    inline virtual void resizeJacobianMatrix(Eigen::MatrixXd &ret);

    virtual bool getJacobianAt(double tstamp, int j,
        Eigen::MatrixXd &ret) const;

    inline virtual bool updateVertexSet(double mintstamp, double maxtstamp) {
      return true;
    }

    virtual void prepareForPoseRemoval(double mintstamp, double maxtstamp) {
    }

    inline virtual int getWindowSize() const {
      return _windowSize;
    }

    inline virtual int parameterEstimateDimension() const {
      return _toblend.front()->parameterEstimateDimension();
    }

    // disabled methods for limitations of the current implementation

    virtual std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator getVertices(
        double tstamp) const {
      // the vertices for this parameter do not live in only one collection, meaningless.
      assert(0);
    }

    virtual const Eigen::VectorXd &getVertexEstimate(double tstamp) {
      // TODO: there is more than one vertex for each timestamp
      assert(0);
    }

    virtual GenericVertexInterface *getVertexNearestTo(double tstamp) {
      // TODO: there is more than one vertex for each timestamp
      assert(0);
    }

    virtual void setVertexEstimate(double tstamp, const Eigen::VectorXd &x) {
      // TODO: there is more than one vertex for each timestamp
      std::cerr
          << "[BlendedParameter] you cannot set the estimate of a blended parameter, you have to refer to individual ones"
          << std::endl;
      assert(0);
    }

    virtual ~ParameterBlender() {
    }

  protected:

    std::vector<ParameterVerticesManager *> _toblend;
    int _windowSize;
};

} /* namespace ROAMestimation */

#endif /* PARAMETERBLENDER_H_ */
