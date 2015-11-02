/*
 * ParameterBlender.cpp
 *
 *  Created on: Aug 3, 2015
 *      Author: davide
 */

#include "ParameterBlender.h"

namespace ROAMestimation {

ParameterBlender::ParameterBlender(g2o::AutoIDSparseOptimizer * opt,
    ParameterTypes type, const std::string& name,
    std::vector<ParameterVerticesManager *> toblend) :
    ParameterVerticesManager(opt, type, name), _toblend(toblend.begin(),
        toblend.end()) {

  _windowSize = 0;
  for (auto pit = _toblend.begin(); pit != _toblend.end(); ++pit) {

    if ((*pit)->getType() != type) {
      std::cerr << "[ParameterBlender] supplied parameters are not compatible"
          << std::endl;
      assert(0);
    }

    _windowSize += (*pit)->getWindowSize();
  }
}

void ParameterBlender::getVerticesPointers(double tstamp,
    std::vector<g2o::HyperGraph::Vertex*>& to, int freePosition) const {

  for (auto pit = _toblend.begin(); pit != _toblend.end(); ++pit) {
    (*pit)->getVerticesPointers(tstamp, to, freePosition);
    freePosition += (*pit)->getWindowSize();
  }
}

bool ParameterBlender::getValueAt(double tstamp, Eigen::VectorXd& ret) const {

  Eigen::VectorXd tmp;

  tmp.resize(tmp.rows());
  ret.setZero();

  for (auto pit = _toblend.begin(); pit != _toblend.end(); ++pit) {
    (*pit)->getValueAt(tstamp, tmp);
    ret += tmp;
  }
}

inline void ParameterBlender::resizeJacobianMatrix(Eigen::MatrixXd& ret) {

  // since the parameters have to be homogeneous, then also their vertices must be.
  // so the jacobian of the value of the parameter wrt each vertex has the same size
  // for every blended parameter vertex.
  _toblend.front()->resizeJacobianMatrix(ret);
}

bool ParameterBlender::getJacobianAt(double tstamp, int j,
    Eigen::MatrixXd& ret) const {
  int which = 0;
  while (j >= _toblend[which]->getWindowSize()) {
    j -= _toblend[which]->getWindowSize();
    which++;
  }

  return _toblend[which]->getJacobianAt(tstamp, j, ret);
}

void ParameterBlender::setFixed(bool isfixed) {
  for (auto pit = _toblend.begin(); pit != _toblend.end(); ++pit) {
    (*pit)->setFixed(isfixed);
  }
}

void ParameterBlender::setComputeCovariance(bool computeCovariance) {
  for (auto pit = _toblend.begin(); pit != _toblend.end(); ++pit) {
    (*pit)->setComputeCovariance(computeCovariance);
  }
}

bool ParameterBlender::fixed() const {
  for (auto pit = _toblend.begin(); pit != _toblend.end(); ++pit) {
    if ((*pit)->fixed() == false) {
      return false;
    }
  }

  return true;
}

} /* namespace ROAMestimation */

