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
 * FHPFeatureHandler.cpp
 *
 *  Created on: Nov 27, 2014
 *      Author: davide
 */

#include "FHPFeatureHandler.h"

#include "SufficientParallax.h"

#include <iostream>

using namespace std;
using namespace ROAMestimation;

namespace ROAMvision {

FHPFeatureHandler::FHPFeatureHandler(double initialDepth) :
    _initialDepth(initialDepth) {
}

bool FHPFeatureHandler::init(FactorGraphFilter* f, const string &name,
    const Eigen::VectorXd & T_OS, const Eigen::VectorXd & K) {

  _filter = f;
  _sensorName = name;

  // TODO: currently FHP works only if system is camera-centric, i.e., T_OS = I
  assert(T_OS.head(3).norm() == 0.0 && T_OS.tail(3).norm() == 0.0);

  _filter->addConstantParameter("Camera_SOx", 0.000, true);
  _filter->addConstantParameter("Camera_SOy", 0.000, true);
  _filter->addConstantParameter("Camera_SOz", 0.000, true);

  // camera centric
  _filter->addConstantParameter("Camera_qOSx", 0.0, true);
  _filter->addConstantParameter("Camera_qOSy", 0.0, true);
  _filter->addConstantParameter("Camera_qOSz", 0.0, true);

  _filter->addConstantParameter(Matrix3D, "Camera_CM", K, true);
}

bool FHPFeatureHandler::addFeatureObservation(long int id, double t,
    const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) {

  const string &sensor = getFeatureSensor(id);

  // there must already exist a pose
  PoseVertexWrapper_Ptr cur_frame = _filter->getNearestPoseByTimestamp(t);
  if (!cur_frame) {
    return false;
  }

  // time of the message must match the vertex found
  if (fabs(t - cur_frame->getTimestamp()) > _timestampOffsetTreshold) {
    return false;
  }

  if (_features.find(id) == _features.end()) {
    // we need to add a new sensor
    initFeature(sensor, z, cur_frame, id);
  }

  FHPTrackDescriptor &d = _features[id];

  // only one reading per frame
  if (d.lastFrame && cur_frame->sameVertexAs(d.lastFrame)) {
    return false;
  }
  d.lastFrame = cur_frame;

  if (!d.isInitialized) {

    // add the current observation to history
    ObservationDescriptor &obs = d.zHistory[t];
    obs.pose = cur_frame;
    obs.z = z;
    obs.t = t;

    Eigen::VectorXd HP(3);

    if (d.initStrategy->initialize(HP)) {

      // add parameter vertices

      _filter->addConstantParameter(Euclidean3D, sensor + "_HP",
          d.anchorFrame->getTimestamp(), HP, false);
      _filter->poseVertexAsParameter(d.anchorFrame, sensor + "_F");

      // add all the edges

      // TODO: make the FHP special case in which anchor = current pose, so we can use also the first observation
      for (auto it = ++d.zHistory.begin(); it != d.zHistory.end(); ++it) {
        const ObservationDescriptor &obs = it->second;

        MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor, obs.t,
            obs.z, cov, obs.pose);

        assert(ret);
      }

      // add prior on the viewing ray

      ParameterWrapper_Ptr K_par = _filter->getParameterByName("Camera_CM");
      assert(K_par);
      double fx = K_par->getEstimate()(0);
      double fy = K_par->getEstimate()(4);

      //TODO: let user decide this
      const double sigma_pixel = 1.0;

      // prior on viewing ray only
      Eigen::MatrixXd prior_cov(2, 2);
      prior_cov << sigma_pixel / pow(fx, 2), 0, 0, sigma_pixel / pow(fy, 2);

      Eigen::VectorXd prior_mean(2);
      prior_mean = HP.head(2);

      _filter->addPriorOnConstantParameter(FHPPriorOnHomogeneousPoint,
          sensor + "_HP", prior_mean, prior_cov);
      //*/

      /* prior on viewing ray and inverse depth

       double _maximum_depth = 100; // TODO put in constructor

       Eigen::MatrixXd prior_cov(3, 3);
       prior_cov <<
       sigma_pixel / pow(fx, 2), 0, 0,
       0, sigma_pixel / pow(fy, 2), 0,
       0, 0, pow(_initialDepth/3.0,2);

       Eigen::VectorXd prior_mean(3);
       prior_mean << HP(0), HP(1), 1/_initialDepth;

       _filter->addPriorOnConstantParameter(Euclidean3DPrior,
       sensor + "_HP", prior_mean, prior_cov);

       cerr << "add euclidean prior" << endl;
       //*/

      // done
      d.zHistory.clear();
      d.isInitialized = true;

#			ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
      cerr << "[FHPFeatureHandler] Ready to estimate depth for track " << id
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

bool FHPFeatureHandler::initFeature(const std::string& sensor,
    const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr pv,
    long int id) {

  _filter->addSensor(sensor, FramedHomogeneousPoint, false, true);
  _filter->shareSensorFrame("Camera", sensor);
  _filter->shareParameter("Camera_CM", sensor + "_CM");

  ParameterWrapper_Ptr K_par = _filter->getParameterByName("Camera_CM");
  assert(K_par);

  //add to current track list
  FHPTrackDescriptor &d = _features[id];
  d.isInitialized = false;
  d.initStrategy = new SufficientParallax(0.0, _initialDepth, d.zHistory,
      K_par->getEstimate().data());

  d.anchorFrame = pv;

  _filter->setRobustKernel(sensor, true, 3.0);

# ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
  cerr << "[FHPFeatureHandler] New feature, id " << id << endl;
# endif

  return true;
}

void FHPFeatureHandler::fixOlderPosesWRTVisibleFeatures() {
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
}

bool FHPFeatureHandler::getFeaturePositionInWorldFrame(long int id,
    Eigen::VectorXd& fw) const {

  const string &sensor = getFeatureSensor(id);

  ParameterWrapper_Ptr hp_par = _filter->getParameterByName(sensor + "_HP"); //homogeneous point
  ParameterWrapper_Ptr f_par = _filter->getParameterByName(sensor + "_F"); // anchor frame

  const Eigen::VectorXd &hp = hp_par->getEstimate();
  const Eigen::VectorXd &f = f_par->getEstimate();

  const static int _OFF = -1;

#	include "generated/FHP_FeatureInWorld.cppready"

}

bool FHPFeatureHandler::getFeaturesIds(std::vector<long int>& to) const {
  to.clear();

  for (auto it = _features.begin(); it != _features.end(); ++it) {
    if (it->second.isInitialized) {
      to.push_back(it->first);
    }
  }
}

long int FHPFeatureHandler::getNActiveFeatures() const {
  long int N = 0;

  for (auto it = _features.begin(); it != _features.end(); ++it) {
    if (it->second.isInitialized) {
      N++;
    }
  }

  return N;
}

string FHPFeatureHandler::getFeatureSensor(long int id) const {
  stringstream s;
  s << _sensorName << "_" << id;
  return s.str();
}

void FHPFeatureHandler::setTimestampOffsetTreshold(double dt) {
  _timestampOffsetTreshold = dt;
}

} /* namespace ROAMvision */

