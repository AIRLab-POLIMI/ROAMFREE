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
 * GenericLinearConstraint.cpp
 *
 *  Created on: Sep 25, 2013
 *      Author: davide
 */

#include "GenericLinearConstraint.h"

#include "GenericVertex.h"
#include "ROAMfunctions/AllVariables.h"

namespace ROAMestimation {

GenericLinearConstraint::GenericLinearConstraint() {
}

void GenericLinearConstraint::computeError() {

  // the beginning row of the current measurement block
  int startE = 0;
  int startZ = 0;

  for (int k = 0; k < _vertices.size(); k++) {

    g2o::OptimizableGraph::Vertex *v =
        static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[k]);

    Eigen::Map<Eigen::VectorXd> x(v->accessEstimateData(),
        v->estimateDimension());

    const Eigen::VectorBlock<Eigen::VectorXd> &z = _measurement.segment(startZ,
        v->estimateDimension());

    // this shadows the _error class member!!! To reuse the already computed error equations
    Eigen::VectorBlock<Eigen::VectorXd> &&_error = _fullError.segment(startE,
        v->dimension());

    // sadly here we have to distinguish for the type of the vertex
    if (dynamic_cast<GenericVertex<ROAMfunctions::SE3V> *>(v) != NULL) {

      static Eigen::VectorXd w(1);
      double sign;

#     include "generated/SE3Prior_TestW.cppready"

      if (w(0) < 0) {
        sign = -1.0;
      } else {
        sign = 1.0;
      }

#     include "generated/SE3Prior_Err.cppready"

    } else if (dynamic_cast<GenericVertex<ROAMfunctions::QuaternionV> *>(v)
        != NULL) {

      static Eigen::VectorXd w(1);
      double sign;

#     include "generated/QuaternionPrior_TestW.cppready"

      if (w(0) < 0) {
        sign = -1.0;
      } else {
        sign = 1.0;
      }

#     include "generated/QuaternionPrior_Err.cppready"

    } else {
      // this is the basic Euclidean case, it is sufficient to do x-z
      _error = x - z;
    }

    /* debug
    std::cerr << "[GLC] Info: x " << std::endl <<x.transpose() << std::endl;
    std::cerr << "[GLC] Info: z " << std::endl <<z.transpose() << std::endl;
    std::cerr << "[GLC] Info: err " << std::endl <<_error.transpose() << std::endl;
    //*/

    startZ += v->estimateDimension();
    startE += v->dimension();

  }

  _error = _G * _fullError;

}

void GenericLinearConstraint::linearizeOplus() {
  // the beginning row of the current measurement block
  int startE = 0;
  int startZ = 0;

  for (int k = 0; k < _vertices.size(); k++) {

    g2o::OptimizableGraph::Vertex *v =
        static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[k]);

    // sadly here we have to distinguish for the type of the vertex
    // if it is a manifold vertex, update jacobian, otherwise it is the identity (and it has been already set in resizeStructures
    if (dynamic_cast<GenericVertex<ROAMfunctions::SE3V> *>(v) != NULL) {

      Eigen::Map<Eigen::VectorXd> x(v->accessEstimateData(),
          v->estimateDimension());
      const Eigen::VectorBlock<Eigen::VectorXd> &z = _measurement.segment(startZ,
              v->estimateDimension());

      //temporary place to put the jacobian of X wrt deltaX
      // to be confortable with the name assumed in the .ccpready file
      Eigen::Matrix<double, 6, 6> _jacobianOplusXi;

      static Eigen::VectorXd w(1);
      double sign;

#     include "generated/SE3Prior_TestW.cppready"

      if (w(0) < 0) {
        sign = -1.0;
      } else {
        sign = 1.0;
      }

#     include "generated/SE3Prior_JErr.cppready"

      _jacobianOplus[k] = _G.block(0, startE, _G.rows(), v->dimension())
          * _jacobianOplusXi;

    } else if (dynamic_cast<GenericVertex<ROAMfunctions::QuaternionV> *>(v)
        != NULL) {

      Eigen::Map<Eigen::VectorXd> x(v->accessEstimateData(),
          v->estimateDimension());
      const Eigen::VectorBlock<Eigen::VectorXd> &z = _measurement.segment(startZ,
              v->estimateDimension());

      //temporary place to put the jacobian of X wrt deltaX
      // to be confortable with the name assumed in the .ccpready file
      Eigen::Matrix<double, 3, 3> _jacobianOplusXi;

      static Eigen::VectorXd w(1);
      double sign;

#     include "generated/QuaternionPrior_TestW.cppready"

      if (w(0) < 0) {
        sign = -1.0;
      } else {
        sign = 1.0;
      }

#     include "generated/QuaternionPrior_JErr.cppready"

      _jacobianOplus[k] = _G.block(0, startE, _G.rows(), v->dimension())
          * _jacobianOplusXi;

    }

    startZ += v->estimateDimension();
    startE += v->dimension();

  }

}

Eigen::MatrixXd& GenericLinearConstraint::accessGain() {
  return _G;
}

void GenericLinearConstraint::resizeStructures() {

// the error size is the number of rows in _G
  _error.resize(_G.rows());
  _dimension = _G.rows();

  int dimG = 0;
  int dimZ = 0;

  for (int k = 0; k < _vertices.size(); k++) {
    g2o::OptimizableGraph::Vertex *ov =
        static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[k]);

    _jacobianOplus[k].resize(_G.rows(), ov->dimension());

    // in case this is not vertex living in odd manifolds, the jacobian is constant
    // so we precompute it and store properly

    if (dynamic_cast<GenericVertex<ROAMfunctions::SE3V> *>(ov) == NULL
        && dynamic_cast<GenericVertex<ROAMfunctions::QuaternionV> *>(ov) == NULL) {
      _jacobianOplus[k] = _G.block(0, dimG, _G.rows(), ov->dimension());
    }

    dimG += ov->dimension();
    dimZ += ov->estimateDimension();
  }

  // check if the user has provided a compatible gain matrix
  assert(_G.cols() == dimG);

  _measurement.resize(dimZ);
  _fullError.resize(dimG);

  //the information matrix is the identity
  _information = Eigen::MatrixXd::Identity(_G.rows(), _G.rows());

}

bool GenericLinearConstraint::read(std::istream &s) {

  std::cerr
      << "[GenericLinearConstraint] ImplementMe: read from g2o log not implemented"
      << std::endl;

  assert(false);

  return false;
}

std::string GenericLinearConstraint:: writeDebugInfo() const {
  std::stringstream s;

  s << "GLC[";

  int k;
  for (k = 0; k<_vertices.size(); k++) {
    g2o::OptimizableGraph::Vertex * ov = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[k]);
    s << ov->id() << (k<_vertices.size()-1 ? "," : "");
  }

  s << "]";

  return s.str();
}

bool GenericLinearConstraint::write(std::ostream &s) const {

  s << _G.rows() << " " << _G.cols();

  for (unsigned int i = 0; i < _measurement.rows(); i++) {
    s << std::fixed << std::setprecision(8) << _measurement(i) << " ";
  }

  for (unsigned int i = 0; i < _G.rows(); i++) {
    for (unsigned int j = i; j < _G.cols(); j++) {
      s << std::fixed << std::setprecision(8) << _G(i, j) << " ";
    }
  }

  return s.good();
}

} /* namespace ROAMestimation */
