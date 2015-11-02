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
 * Parameter.cpp
 *
 *  Created on: May 20, 2013
 *      Author: davide
 */

#include "ParameterVerticesManager.h"

#include "PriorEdges/AllDerivativePriors.h"
#include "GenericLinearConstraint.h"

#include "ROAMutils/StringUtils.h"

namespace ROAMestimation {

ParameterVerticesManager::ParameterVerticesManager(
    g2o::AutoIDSparseOptimizer *opt, ParameterTypes typ,
    const std::string& name) :
    _optimizer(opt), _name(name), _type(typ), _isFixed(true), _process(None), _computeCovariance(
        false) {
}

ParameterVerticesManager::~ParameterVerticesManager() {
}

g2o::OptimizableGraph::Vertex * ParameterVerticesManager::newVertex(
    double tstamp, const Eigen::VectorXd &x0) {
  g2o::OptimizableGraph::Vertex * v = NULL;
  GenericVertexInterface *v2 = NULL;

  switch (_type) {
  case Euclidean1D: {
    GenericVertex<Eucl1DV> *gv = new GenericVertex<Eucl1DV>;
    v = gv;
    v2 = gv;
    gv->setTimestamp(tstamp);
  }
    break;
  case Euclidean2D: {
    GenericVertex<Eucl2DV> *gv = new GenericVertex<Eucl2DV>;
    v = gv;
    v2 = gv;
    gv->setTimestamp(tstamp);
  }
    break;
  case Euclidean3D: {
    GenericVertex<Eucl3DV> *gv = new GenericVertex<Eucl3DV>;
    v = gv;
    v2 = gv;
    gv->setTimestamp(tstamp);
  }
    break;
  case Quaternion: {
    GenericVertex<QuaternionV> *gv = new GenericVertex<QuaternionV>;
    v = gv;
    v2 = gv;
    gv->setTimestamp(tstamp);
  }
    break;
  case Matrix3D: {
    GenericVertex<Matrix3DV> *gv = new GenericVertex<Matrix3DV>;
    v = gv;
    v2 = gv;
    gv->setTimestamp(tstamp);
  }
    break;
  case SE3: {
    GenericVertex<SE3V> *gv = new GenericVertex<SE3V>;
    v = gv;
    v2 = gv;
    gv->setTimestamp(tstamp);
  }
    break;
  default:
    std::cerr << "[ParameterVerticesManager] Error: unknown parameter type"
        << std::endl;
    break;
    return NULL;
  }

  v->setFixed(_isFixed);
  //v2->setDisplayEstimate(true); // this is for the viewer, for now we don't care

  Eigen::Map<Eigen::VectorXd> x(v->accessEstimateData(),
      v->estimateDimension());
  x = x0;

  /* this is if we want the timestamp in the name, discarded for now *
   std::stringstream buf;
   buf << _name << ",t=" << std::fixed << std::setprecision(3) << tstamp;
   v2->setCategory(buf.str());
   */

  v2->setCategory(_name);

  _optimizer->addVertex(v);

  auto ret = _v.insert(
      std::pair<double, g2o::OptimizableGraph::Vertex *>(tstamp, v));
  assert(ret.second == true); // check that this vertex was really new

  // in case we have to put some process model edge
  if (_process != None && _v.size() > 1) {

    VertexMap::iterator it = ret.first;

    // decide the direction
    g2o::OptimizableGraph::Vertex *older, *newer;
    double dt = 0;

    if (it != _v.begin()) { // forward
      newer = it->second; //the vertex just inserted
      dt += it->first;

      --it;

      older = it->second;
      dt -= it->first;
    } else {
      older = it->second;
      dt -= it->first;

      ++it;

      newer = it->second;
      dt += it->first;

      assert(it != _v.end());
    }

    switch (_process) {
    case RandomWalk:
      addRandomWalkProcessEdge(older, newer, _randomWalkNoiseCov, dt);
      break;

    case GaussMarkov:
      addGaussMarkovProcessEdge(older, newer, _gaussMarkovNoiseCov,
          _gaussMarkovBeta, dt);
      break;
    }

  }

  return v;
}

void ParameterVerticesManager::getVerticesPointers(double tstamp,
    std::vector<g2o::HyperGraph::Vertex *>& to, int freePosition) const {
  std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator first =
      getVertices(tstamp);

  for (int i = 0; i < getWindowSize(); ++i, ++first) {
    to[i + freePosition] = first->second;
  }
}

void ParameterVerticesManager::setProcessModelType(ProcessTypes t) {
  _process = t;
}

g2o::OptimizableGraph::Edge* ParameterVerticesManager::addRandomWalkProcessEdge(
    g2o::OptimizableGraph::Vertex* older, g2o::OptimizableGraph::Vertex* newer,
    const Eigen::MatrixXd& noiseCov, double dt) {

  g2o::OptimizableGraph::Edge * oe;

  switch (_type) {
  case Euclidean3D: {
    Eucl3DRandomWalkProcessEdge*gmpe = new Eucl3DRandomWalkProcessEdge;

    gmpe->setNoiseCov(noiseCov);
    gmpe->init(dt);

    gmpe->setCategory(_name + "_proc");
    gmpe->setTimestamp(
        static_cast<GenericVertex<Eucl3DV> *>(newer)->getTimestamp());

    oe = gmpe->getg2oOptGraphPointer();

    break;
  }
  default:
    std::cerr
        << "[ParameterVerticesManager] Error: non implemented for this parameter type"
        << std::endl;
    return NULL;

    break;
  }

  oe->vertices()[0] = older;
  oe->vertices()[1] = newer;

  _optimizer->addEdge(oe);

  return oe;
}

g2o::OptimizableGraph::Edge* ParameterVerticesManager::addGaussMarkovProcessEdge(
    g2o::OptimizableGraph::Vertex* older, g2o::OptimizableGraph::Vertex* newer,
    const Eigen::MatrixXd& noiseCov, const Eigen::VectorXd &beta, double dt) {

  g2o::OptimizableGraph::Edge * oe;

  switch (_type) {
  case Euclidean3D: {
    Eucl3DGaussMarkovProcessEdge *gmpe = new Eucl3DGaussMarkovProcessEdge;

    gmpe->setNoiseCov(noiseCov);
    gmpe->init(beta, dt);

    gmpe->setCategory(_name + "_proc");
    gmpe->setTimestamp(
        static_cast<GenericVertex<Eucl3DV> *>(newer)->getTimestamp());

    oe = gmpe->getg2oOptGraphPointer();

    break;
  }
  default:
    std::cerr
        << "[ParameterVerticesManager] Error: non implemented for this parameter type"
        << std::endl;
    return NULL;

    break;
  }

  oe->vertices()[0] = older;
  oe->vertices()[1] = newer;

  _optimizer->addEdge(oe);

  return oe;
}

void ParameterVerticesManager::setGaussMarkovProcessNoiseCov(
    const Eigen::MatrixXd &cov) {
  if (_process != ProcessTypes::GaussMarkov) {
    std::cerr
        << "[ParameterVerticesManager] Warning: setting Gauss-Markov process noise covariance with different process type"
        << std::endl;
  }

  _gaussMarkovNoiseCov = cov;
}

void ParameterVerticesManager::setGaussMarkovProcessBeta(
    const Eigen::VectorXd &beta) {
  if (_process != ProcessTypes::GaussMarkov) {
    std::cerr
        << "[ParameterVerticesManager] Warning: setting Gauss-Markov beta with different process type"
        << std::endl;
  }

  _gaussMarkovBeta = beta;

}

void ParameterVerticesManager::setRandomWalkProcessNoiseCov(
    const Eigen::MatrixXd& cov) {
  if (_process != ProcessTypes::RandomWalk) {
    std::cerr
        << "[ParameterVerticesManager] Warning: setting Random-Walk process noise covariance with different process type"
        << std::endl;
  }

  _randomWalkNoiseCov = cov;
}

GenericVertexInterface* ParameterVerticesManager::getVertexNearestTo(
    double tstamp) {
  if (_v.size() == 1) {
    return dynamic_cast<GenericVertexInterface *>(_v.begin()->second);
  } else {

    auto M = _v.lower_bound(tstamp); // the first one for which key >= tstamp

    if (M == _v.begin()) {
      return dynamic_cast<GenericVertexInterface *>(M->second);
    } else {

      auto m = M;
      m--;

      GenericVertexInterface *vM =
          dynamic_cast<GenericVertexInterface *>(M->second);
      GenericVertexInterface *vm =
          dynamic_cast<GenericVertexInterface *>(m->second);

      if (vM->getTimestamp() - tstamp < tstamp - vm->getTimestamp()) {
        return vM;
      } else {
        return vm;
      }
    }
  }
}

void ParameterVerticesManager::emptyTrash() {
  _noLongerNeeded.clear();
}

void ParameterVerticesManager::setFixed(bool fixed) {
  std::map<double, g2o::OptimizableGraph::Vertex *>::iterator it;

  for (it = _v.begin(); it != _v.end(); ++it) {
    it->second->setFixed(fixed);
  }

  _isFixed = fixed;
}

const Eigen::VectorXd& ParameterVerticesManager::getVertexEstimate(
    double tstamp) {
  return getVertexNearestTo(tstamp)->getEstimate();
}

void ParameterVerticesManager::setVertexEstimate(double tstamp,
    const Eigen::VectorXd& x) {
  getVertexNearestTo(tstamp)->setEstimate(x);
}

} /* namespace ROAMestimation */
