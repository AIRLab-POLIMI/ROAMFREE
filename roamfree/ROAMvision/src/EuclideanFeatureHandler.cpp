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
 * EuclideanFeatureHandler.cpp
 *
 *  Created on: Jan 6, 2016
 *      Author: davide
 */

#include "EuclideanFeatureHandler.h"

#include "SufficientParallax.h"

#include <iostream>

using namespace std;
using namespace ROAMestimation;

namespace ROAMvision {

EuclideanFeatureHandler::EuclideanFeatureHandler() {
}

bool EuclideanFeatureHandler::init(FactorGraphFilter* f, const string &name,
    const Eigen::VectorXd & T_OS, const Eigen::VectorXd & K) {

  _filter = f;
  _sensorName = name;

  _filter->addConstantParameter(_sensorName + "_Cam_SOx", T_OS(0), true);
  _filter->addConstantParameter(_sensorName + "_Cam_SOy", T_OS(1), true);
  _filter->addConstantParameter(_sensorName + "_Cam_SOz", T_OS(2), true);

  _filter->addConstantParameter(_sensorName + "_Cam_qOSx", T_OS(4), true);
  _filter->addConstantParameter(_sensorName + "_Cam_qOSy", T_OS(5), true);
  _filter->addConstantParameter(_sensorName + "_Cam_qOSz", T_OS(6), true);

  _filter->addConstantParameter(Matrix3D, _sensorName + "_Cam_CM", K, true);

  return true;
}

bool EuclideanFeatureHandler::addFeatureObservation(long int id, double t,
    const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) {

  // there must already exist a pose
  PoseVertexWrapper_Ptr cur_frame = _filter->getNearestPoseByTimestamp(t);
  if (!cur_frame) {
    return false;
  }

  // time of the message must match the vertex found
  if (fabs(t - cur_frame->getTimestamp()) > _timestampOffsetTreshold) {
    return false;
  }

# ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
  if (_features.find(id) == _features.end()) {
    cerr << "[EuclideanFeatureHandler] New feature, id " << id << endl;
  }
# endif

  const string &sensor = getFeatureSensor(id);

  // this implicitly initialize the new track if it was not there
  EuclideanTrackDescriptor &d = _features[id];

  if (!d.isInitialized) {

    // add the current observation to history
    ObservationDescriptor &obs = d.zHistory[t];
    obs.pose = cur_frame;
    obs.z = z;
    obs.t = t;

    // TODO: if initialization is succesful
    Eigen::Vector3d Lw;

    if (d.zHistory.size() > 5) {

      _filter->addSensor(sensor, ImagePlaneProjection, false, true);
      _filter->shareSensorFrame(_sensorName + "_Cam", sensor);
      _filter->shareParameter(_sensorName + "_Cam_CM", sensor + "_CM");

      // do we want the robust kernel by default?
      // _filter->setRobustKernel(sensor, true, 3.0);

      // add parameter vertices

      _filter->addConstantParameter(Euclidean3D, sensor + "_Lw",
          d.zHistory.begin()->first, Lw, false);

      // add all the edges

      for (auto it = d.zHistory.begin(); it != d.zHistory.end(); ++it) {
        const ObservationDescriptor &obs = it->second;

        MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor, obs.t,
            obs.z, cov, obs.pose);

        assert(ret);
      }

      // done
      d.zHistory.clear();
      d.isInitialized = true;

#			ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
      cerr << "[EuclideanFeatureHandler] Ready to estimate depth for track " << id
      << endl;
#			endif
    }

  } else { // it has already been initialized, just add the measurement
    MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor, t, z, cov,
        cur_frame);

    assert(ret);
  }

  return true;
}

void EuclideanFeatureHandler::fixOlderPosesWRTVisibleFeatures() {
  /* TODO: implement this
   const double _KEEPALIVE_TS_TRESHOLD = 0.25; // TODO: put somewhere.

   double ts = _filter->getNewestPose()->getTimestamp();
   double oldestAnchorTs = std::numeric_limits<double>::infinity();

   for (auto it = _features.begin(); it != _features.end(); ++it) {
   if ((ts - it->second.lastFrame->getTimestamp()) < _KEEPALIVE_TS_TRESHOLD) {
   oldestAnchorTs = min(oldestAnchorTs,
   it->second.anchorFrame->getTimestamp());
   }
   }

   double k;
   PoseVertexWrapper_Ptr curPose;
   while ((curPose = _filter->getNthOldestPose(k++))
   && curPose->getTimestamp() < oldestAnchorTs) {
   curPose->setFixed(true);
   }
   */
}

bool EuclideanFeatureHandler::getFeaturePositionInWorldFrame(long int id,
    Eigen::VectorXd& fw) const {

  ParameterWrapper_Ptr lw_par = _filter->getParameterByName(
      getFeatureSensor(id) + "_Lw"); // euclidean coordinates

  fw = lw_par->getEstimate();
}

bool EuclideanFeatureHandler::getFeaturesIds(std::vector<long int>& to) const {
  to.clear();

  for (auto it = _features.begin(); it != _features.end(); ++it) {
    if (it->second.isInitialized) {
      to.push_back(it->first);
    }
  }

  return true;
}

long int EuclideanFeatureHandler::getNActiveFeatures() const {
  long int N = 0;

  for (auto it = _features.begin(); it != _features.end(); ++it) {
    if (it->second.isInitialized) {
      N++;
    }
  }

  return N;
}

string EuclideanFeatureHandler::getFeatureSensor(long int id) const {
  stringstream s;
  s << _sensorName << "_" << id;
  return s.str();
}

void EuclideanFeatureHandler::setTimestampOffsetTreshold(double dt) {
  _timestampOffsetTreshold = dt;
}

} /* namespace ROAMvision */

