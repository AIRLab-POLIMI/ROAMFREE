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
 * GenericEdgeInterface.h
 *
 *  Created on: Nov 7, 2013
 *      Author: davide
 */

#ifndef GENERICEDGEINTERFACE_H_
#define GENERICEDGEINTERFACE_H_

#include <Eigen/Dense>
#include "ParameterTemporaries.h"

namespace ROAMestimation {

class GenericEdgeInterface {

public:

  virtual void setNoiseCov(const Eigen::MatrixXd &noiseCov) = 0;
  virtual const Eigen::MatrixXd & getNoiseCov() const = 0;

  virtual int measurementDimension() const = 0;
  virtual double *accessMeasurementData() = 0;

  virtual int noiseDimension() const = 0;

  /**
   * \brief update the local copy of the function parameters current estimate and their jacobian, if needed
   */
  virtual void handleParamTemporaries() = 0;

  /**
   * \brief update the shortcut pointers to be passed to F error and errorJacobian
   */
  virtual void updateParamPtrs() = 0;
  virtual const std::vector<ParameterTemporaries> & getParameterTemporariesVector() const = 0;

  /**
   * this method tells the edge to predict the next state based on the posterior of the current state
   * and the associated measurement.
   *
   * TODO: this has a precise meaning only for normal state estimation edges, i.e. QuaternionGenericEdge
   *       consider some other place to put this stuff
   */

  virtual void predictNextState() = 0;

  virtual void setTimestamp(double timestamp) = 0;
  virtual double getTimestamp() const = 0;

  /**
   * \brief category now means the logical sensor it is associated to.
   *
   * TODO: change the name to something more related to the logical sensor name
   */
  virtual void setCategory(const std::string &name) = 0;
  virtual const std::string &getCategory() const = 0;

  /**
   * \brief get the edge associated augmented state
   *
   * each GenericEdge in roamfree has an augmented state, which means
   * x, q, v, w, a, alpha, in reals^19
   */

  virtual const Eigen::VectorXd &getAugmentedState() const = 0;

  /**
   * \brief get the measurement vector
   *
   * TODO: these methods already exist at the g2o::BaseEdge
   *       for inheritance reasons I have to put there also here (tweak)
   */

  virtual const Eigen::VectorXd& getMeasurement_GE() const = 0;
  virtual void setMeasurement_GE(const Eigen::VectorXd& m) = 0; /**< this sets the measurement instead */

  /**
   * \brief get the order of the generc edge
   *
   * The return value refers to the order of the finite differences
   * between poses which is required to evaluate the relevant parts of the
   * augmented state estimator.
   *
   * -1 -> not meaningful (i.e. constraints which do not need finite differences nor augmented state)
   * 0  -> pose and orientation only constraints
   * 1  -> also velocities
   * 2  -> also accelerations
   *
   */

  inline
  virtual int getOrder() const {
    return -1;
  }

  /**
   * \brief get the state componets observed by the sensor function
   *
   * it returns the array of bool defined in the corresponding ROAMfunction element
   */

  virtual const bool *getUsedComponents() const = 0;

  /*
   * \brief abstract GenericEdgeInterface -> & OptimizableGraph conversion operator.
   *
   * We need this since children o this class in ROAMFREE are also children of
   * g2o::OptimizableGraph::Edge and we would like to go back in the inheritance on the other branch
   *
   */
  virtual operator g2o::OptimizableGraph::Edge &() = 0;

  /**
   * \brief abstract GenericEdgeInterface -> OptimizableGraph * conversion operator.
   */
  virtual operator g2o::OptimizableGraph::Edge *() = 0;

  /**
   * \brief returns a string containing debug informations about the edge
   *
   * i.e. category, timestamp and vertices it is connected to
   */
  virtual std::string writeDebugInfo() const = 0;

  /**
   * the frame counter is an integer which gives the position of this edge
   * with respect to the ordering of this sensor readings
   */
  virtual long int getFrameCounter() const = 0;
  virtual void setFrameCounter(long int c) = 0;

  /**
   * \brief returns a string which uniquely identifies this edge
   */
  virtual std::string getEdgeHash() const = 0;

};

}

#endif /* GENERICEDGEINTERFACE_H_ */
