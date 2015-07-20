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
 * Parameter.h
 *
 *  Created on: May 20, 2013
 *      Author: davide
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <map>

#include <Eigen/Dense>

#include "g2o/core/optimizable_graph.h"

#include "Enums.h"
#include "AutoIDSparseOptimizer.h"
#include "GenericVertex.h"
#include "ROAMfunctions/AllVariables.h"

class FactorGraphFilter_Impl;

using namespace ROAMfunctions;

namespace ROAMestimation {

/**
 * \brief manages the vertices needed to hold a parameter state variable
 *
 */

class ParameterVerticesManager {

  public:

    typedef std::map<double, g2o::OptimizableGraph::Vertex *> VertexMap;

    /**
     * \brief constructor
     *
     * parameters are fixed by default. call the setFixed(true) method to change this
     *
     * @param opt the sparse optimizer where the parameter vertices live
     * @param typ the parameter type
     * @param name unique key which is used to refer to this parameter
     *
     */

    ParameterVerticesManager(g2o::AutoIDSparseOptimizer *opt,
        ParameterTypes typ, const std::string &name);
    virtual ~ParameterVerticesManager();

    /**
     * \brief if enabled, a RandomWalkProcessEdge will be inserted incident on successive vertices
     */
    virtual void setRandomWalkProcessEnabled(bool enable) = 0;

    /**
     * \brief set the noise covariance matrix for the RandomWalkProcessEdge
     */
    virtual void setRandomWalkProcessNoisCov(const Eigen::MatrixXd &cov) = 0;

    /**
     * \brief returns an iterator to relevant parameter nodes for tstamp
     *
     * here the I return an iterator to the first vertex neded to compute
     * the value of the parameter at time t. The getWindowSize method
     * has to be called to determine the last one
     *
     * @param time at which we ask for needed vertices pointers
     */
    virtual std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator getVertices(
        double tstamp) const = 0;

    /**
     * \brief adds necessary parameter vertices in the future or in the past
     *
     * this function update the vertices in the window such that we can compute
     * the value of the parameter for all t such that t > mintstamp and t < maxtstamp
     */
    virtual bool updateVertexSet(double mintstamp, double maxtstamp) = 0;

    /**
     * \brief gets the value of the parameter at a given timestamp
     *
     * @param tstamp the timestamp at which the parameter has to be evaluated
     * @param ret the return variable
     *
     * @return true if the value depends on t
     */
    virtual bool getValueAt(double tstamp, Eigen::VectorXd &ret) const = 0;

    /**
     * \brief gets the current estimate of the vertex nearest to tstamp
     *
     * @param tstamp the query timestamp
     *
     * @return the estimate
     */
    virtual const Eigen::VectorXd &getVertexEstimate(double tstamp); //TODO: should be const but it calls a non const method for code reuse.

    /**
     * \brief sets the estimate for the vertex nearest to tstamp
     *
     * @param tstamp the query timestamp
     */
    virtual void setVertexEstimate(double tstamp, const Eigen::VectorXd &x);

    /**
     * \brief resize the Jacobian matrix passed as a reference so that it can contain
     * the matrix evaluated by getJacobianAt
     */
    virtual void resizeJacobianMatrix(Eigen::MatrixXd &ret) = 0;

    /**
     * \brief evaluates jacobian wrt jth vertex at tstamp
     *
     * returns
     * true if the value of the Jacobian depends on the value of the parameters
     * false if the value of the Jacobian depends only on tstamp
     *
     * @param tstamp time a which the Jacobian has to be evaluated
     * @param j
     * @param ret will contain a maitrx (or eventually a scalar) which is
     *        meaningful to post-multiply to J(something) wrt parameter to obtain
     *        the Jacobian of something wrt vertex j
     */
    virtual bool getJacobianAt(double tstamp, int j,
        Eigen::MatrixXd &ret) const = 0;

    /**
     *  \brief schedules the deletion of a vertex from the optimizer
     *
     *  it is used in the updateVertexSet if we want to mark a vertex to be deleted
     *  we don't delete immediately because this may trigger the deletion of
     *  edges depending on this vertex, eventually corrupting pointers somewhere else
     */
    inline
    void scheduleDelete(double t, g2o::OptimizableGraph::Vertex * v) {
      _noLongerNeeded.insert(std::pair<double, g2o::OptimizableGraph::Vertex *> (t, v));
    }

    /**
     *  \brief notifies the parameterVertexManager that a set of adjacent pose vertices is being removed
     *
     *  when the poses from mintstamp to maxtstamp are being removed from the graph
     *  (either marginalized or dropped) a set of edges will also be removed.
     *  The parameter nodes needed to compute values for these and only these edges are
     *  no longer needed. They are moved to the trash been and made accessible to the
     *  optimizer (i.e., for being included in a GenericLinearConstraint).
     *
     *  TODO: this method is implemented assuming that no pose exists before mintstamp
     *
     *  @param mintstamp unused in current implementations
     *  @param maxtstamp the timestamp of the newest pose that is going to be removed
     *
     */
    virtual void prepareForPoseRemoval(double mintstamp, double maxtstamp) = 0;

    inline const VertexMap &getTrash() {
      return _noLongerNeeded;
    }

    /**
     * \brief empty the local trash
     *
     * vertices are NOT removed from the graph, this has to be done externally iterating trough the trash
     */
    void emptyTrash();

    /**
     * \brief set the fixed flag for all the vertices
     */
    void setFixed(bool fixed);

    /**
     * \brief true => these nodes are considered fixed during optimization
     */
    inline bool fixed() const {
      return _isFixed;
    }

    /**
     * \brief return the dimension of the internal representation of a parameter
     */
    inline int parameterEstimateDimension() const {
      return _v.begin()->second->estimateDimension();
    }

    /**
     * \brief returns the number of vertices that are needed to compute the value of the parameter
     */
    virtual int getWindowSize() const = 0;

  protected:

    g2o::AutoIDSparseOptimizer * _optimizer;

    std::string _name;
    ParameterTypes _type;
    bool _isFixed;

    VertexMap _v;

    /**
     * the vertices no longer needed after the last call of updatedVerterxSet
     * but that still are in the factor graph
     */
    VertexMap _noLongerNeeded;

    /**
     * adds a new vertex to the set, according to the parameter type
     */
    g2o::OptimizableGraph::Vertex * newVertex(double tstamp,
        const Eigen::VectorXd &x0);

    /**
     * insert edges between two vertices limiting the difference between their estimate
     */
    g2o::OptimizableGraph::Edge * addRandomWalkProcessEdge(
        g2o::OptimizableGraph::Vertex *older,
        g2o::OptimizableGraph::Vertex *newer, const Eigen::MatrixXd &noiseCov);

    /**
     * returns the vertex in the set which is nearest to tstamp
     */
    GenericVertexInterface *getVertexNearestTo(double tstamp);

    friend class FactorGraphFilter_Impl;
    // TODO: only for debugging purposes
};

} /* namespace ROAMestimation */
#endif /* PARAMETER_H_ */
