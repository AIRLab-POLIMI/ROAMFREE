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
 * FHPFeatureHandlerBootstrap.cpp
 *
 *  Created on: 16/feb/2015
 *      Author: dave
 */

#include "FHPFeatureHandlerBootstrap.h"
#include "SufficientZChange.h"

#include <iostream>

using namespace std;
using namespace ROAMestimation;

namespace ROAMvision {

FHPFeatureHandlerBootstrap::FHPFeatureHandlerBootstrap(double initialDepth) :
    FHPFeatureHandler(initialDepth) {
  _bootstrap = true;
}

bool FHPFeatureHandlerBootstrap::addFeatureObservation(long int id, double t,
    const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) {
  if (_bootstrap && FHPFeatureHandler::getNActiveFeatures() > 10) {
    _bootstrap = false;
  }

  return FHPFeatureHandler::addFeatureObservation(id, t, z, cov);

}

void FHPFeatureHandlerBootstrap::fixImmutableFeaturePoses(
    const Eigen::VectorXd &pose, double percentageThreshold, double minZDistance) {

  map<double, unsigned int> candidates;

  for (auto& feature : _features) {
    auto& map = feature.second.zHistory;
    voteFixedPoseCandidates(candidates, map, minZDistance);
  }

  for (auto& candidate : candidates) {
    double percentage = static_cast<double>(candidate.second)
        / static_cast<double>(_features.size());

    if (percentage >= percentageThreshold) {
      auto pose_ptr = _filter->getNearestPoseByTimestamp(candidate.first);
      pose_ptr->setEstimate(pose);
      pose_ptr->setFixed(true);
    }
  }

}

void FHPFeatureHandlerBootstrap::voteFixedPoseCandidates(
    std::map<double, unsigned int>& candidates, ObservationMap& map, double minZDistance) {

  auto it = map.begin();
  Eigen::Vector2d& firstObservation = it->second.z;

  while (it != map.end()) {
    if ((it->second.z - firstObservation).norm() <= minZDistance) {
      candidates[it->second.pose->getTimestamp()]++;
      it++;
    } else {
      break;
    }

  }

}

bool FHPFeatureHandlerBootstrap::initFeature(const std::string& sensor,
    const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr pv,
    long int id) {

  if (_bootstrap) {
    //TODO extract method in superclass
    _filter->addSensor(sensor, FramedHomogeneousPoint, false, true);
    _filter->shareSensorFrame("Camera", sensor);
    _filter->shareParameter("Camera_CM", sensor + "_CM");

    ParameterWrapper_Ptr K_par = _filter->getParameterByName("Camera_CM");
    assert(K_par);

    //add to current track list
    FHPTrackDescriptor &d = _features[id];
    d.isInitialized = false;
    d.initStrategy = new SufficientZChange(5.0, _initialDepth, d.zHistory,
        K_par->getEstimate().data());

    d.anchorFrame = pv;

    _filter->setRobustKernel(sensor, true, 3.0);

# ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
    cerr << "[FHPFeatureHandler] New feature, id " << id << endl;
# endif

    return true;

  } else {
    return FHPFeatureHandler::initFeature(sensor, z, pv, id);
  }
}

}
