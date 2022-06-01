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

#include <Eigen/Dense>

#include "g2o/core/optimizable_graph.h"

namespace ROAMestimation {

class BaseEdgeInterface {

  public:
    virtual void setNoiseCov(const Eigen::MatrixXd &noiseCov) = 0;

    virtual const Eigen::VectorXd &getMeasurement() const = 0;
    virtual void setMeasurement(const Eigen::VectorXd &meas) = 0;

    /**
     * \brief category gives an understandable name to different observations.
     */
    virtual void setCategory(const std::string &name) = 0;
    virtual const std::string &getCategory() const = 0;

    virtual void setTimestamp(double timestamp) = 0;
    virtual double getTimestamp() const = 0;

    /**
     * \brief prediciton based on measurement and other states (if meaningful)
     *
     * predict the next state based on the measurement
     * and on other states posterios (depending on the edge type)
     */
    virtual bool predict() = 0;

    /**
     * \brief returns a string containing debug informations about the edge
     *
     * i.e. category, timestamp and vertices it is connected to
     */
    virtual std::string writeDebugInfo() const = 0;

    /*
     * \brief gets a pointer to the g2o base edge class
     *
     * this is needed because the final objects in the graph double inherit from this
     * and from base g2o edge on another branch
     *
     * this method will be implemented in the classes that first inherit from both
     * branches but we need to anticipate their existence here
     */
    virtual g2o::OptimizableGraph::Edge *getg2oOptGraphPointer() = 0;
};

} /* namespace ROAMestimation */
#endif /* BASEPRIOREDGEINTERFACE_H_ */
