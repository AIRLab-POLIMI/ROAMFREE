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

#include <iostream>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "EuclideanFeatureHandler.h"

#include "SufficientParallax.h"

using namespace std;
using namespace ROAMestimation;

// #define DEBUG_PRINT_VISION_INFO_MESSAGES 1
// #define DEBUG_OPTIMIZATION_VERBOSE 1

namespace ROAMvision {

EuclideanFeatureHandler::EuclideanFeatureHandler() {
}

bool EuclideanFeatureHandler::init(FactorGraphFilter* f, const string &name,
    const Eigen::VectorXd & T_OS, const Eigen::VectorXd & K,
    const Eigen::VectorXd & RD, const Eigen::VectorXd & TD) {

  _filter = f;
  _sensorName = name;

  Eigen::VectorXd SO = T_OS.head(3);
  Eigen::VectorXd qOS = T_OS.tail(4);
  
  _filter->addConstantParameter(Euclidean3D, _sensorName + "_Cam_SO", SO, true);
  qOS_par = _filter->addConstantParameter(Quaternion, _sensorName + "_Cam_qOS", qOS, true);

  K_par = _filter->addConstantParameter(Euclidean3D, _sensorName + "_Cam_CM", K,
      true);

  _filter->addConstantParameter(Euclidean3D, _sensorName + "_Cam_RD",
      RD, true);
  _filter->addConstantParameter(Euclidean2D, _sensorName + "_Cam_TD",
      TD, true);

  return true;
}

bool EuclideanFeatureHandler::addFeatureObservation(long int id, double t,
    const Eigen::VectorXd &z, const Eigen::MatrixXd &cov, bool dontInitialize) {

  // there must already exist a pose
  PoseVertexWrapper_Ptr cur_frame = _filter->getNearestPoseByTimestamp(t);
  if (!cur_frame) {
    return false;
  }

  // time of the message must match the vertex found
  if (fabs(t - cur_frame->getTimestamp()) > _timestampOffsetTreshold) {
    cerr << "[EuclideanFeatureHandler]: ERROR, frame too fare for feature " << id << endl;
    return false;
  }

# ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
  if (_features.find(id) == _features.end()) {
    cerr << "[EuclideanFeatureHandler] New feature, id " << id << endl;
  }
# endif

  // this implicitly initialize the new track if it was not there
  EuclideanTrackDescriptor &d = _features[id];

  if (!d.isInitialized) {

    // add the current observation to history
    ObservationDescriptor &obs = d.zHistory[t];
    obs.pose = cur_frame;
    obs.t = t;
    obs.z = z;
    obs.cov = cov;

    if (!dontInitialize && d.zHistory.size() >= 3) {
      return initializeFeature_i(d, id);
    }

    return false;

  } else { // it has already been initialized, just add the measurement
    const string &sensor = getFeatureParameterName(id);
    
    MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor, t, z, cov,
        cur_frame);

    assert(ret);
  }

  return true;
}

bool EuclideanFeatureHandler::initializeFeature(long int id)
{
  auto it = _features.find(id);
  if ( it != _features.end() ) {
    return initializeFeature_i(it->second, id); 
  } else {    
    cerr << "[EuclideanFeatureHandler]: ERROR, feature " << id << " does not exist." << endl;
    return false;
  }  
}

bool EuclideanFeatureHandler::initializeFeature_i(EuclideanTrackDescriptor &d, long int id) {
  if (d.isInitialized) {
    cerr << "[EuclideanFeatureHandler] Feature " << id << " is already initialized " << endl;
    return true;
  }
  
# ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
    cerr << "[EuclideanFeatureHandler] Initializing track " << id << endl;
# endif

  // TODO: if initialization is succesful
  Eigen::VectorXd Lw(3);
    
  if (initialize(d, K_par->getEstimate(), Lw)) {
    
    const string &sensor = getFeatureParameterName(id);

    _filter->addSensor(sensor, ImagePlaneProjection, false, true);

    // it does not work, there is no sensor called _sensorName + "_Cam"
    // we have to share manually the parameters
    // _filter->shareSensorFrame(_sensorName + "_Cam", sensor);

    const string suffixes[] = { "_SO", "_qOS"};
    for (int k = 0; k < 2; k++) {
      _filter->shareParameter(_sensorName + "_Cam" + suffixes[k],
	  sensor + suffixes[k]);
    }

    // TODO: hardcoded
    // do we want the robust kernel by default?

    // considering a 1.5 sigma plus 1e6 scaling, threshold at 10 pix
    // beta =  || err || / sigma
    // where ||err|| is where we want the weight to start to decrease

    // TODO: sigma is hardcoded
    double beta = 5 / sqrt(1e8 * pow(0.8, 2));

    _filter->setRobustKernel(sensor, true, beta);

    // add parameter vertices

    _filter->shareParameter(_sensorName + "_Cam_CM", sensor + "_CM");
    _filter->shareParameter(_sensorName + "_Cam_RD", sensor + "_RD");
    _filter->shareParameter(_sensorName + "_Cam_TD", sensor + "_TD");

    _filter->addConstantParameter(Euclidean3D, sensor + "_Lw", d.zHistory.begin()->first, Lw, false);

    // add all the edges

    for (auto it = d.zHistory.begin(); it != d.zHistory.end(); ++it) {
      const ObservationDescriptor &obs = it->second;

      MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor,
	  obs.t, obs.z, obs.cov, obs.pose);

      assert(ret);
    }

    // done
    d.zHistory.clear();
    d.isInitialized = true;

#   ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
      cerr << "[EuclideanFeatureHandler] Ready to estimate depth for track " << id  << endl;
#   endif

    return true;
  }
  return false;
}

void EuclideanFeatureHandler::fixOlderPosesWRTVisibleFeatures() {
  /* TODO: implement this
   const double _KEEPALIVE_TS_TRESHOLD = 0.25; // TODO: put somewhere.

   double ts = _filter->getNewestPose()->getTimestamp();
   double oldestAnchorTs = numeric_limits<double>::infinity();

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
      getFeatureParameterName(id) + "_Lw"); // euclidean coordinates

  fw = lw_par->getEstimate();
}

bool EuclideanFeatureHandler::getFeaturesIds(vector<long int>& to) const {
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

string EuclideanFeatureHandler::getFeatureParameterName(long int id) const {
  stringstream s;
  s << _sensorName << "_feat" << id;
  return s.str();
}

void EuclideanFeatureHandler::setTimestampOffsetTreshold(double dt) {
  _timestampOffsetTreshold = dt;
}

bool EuclideanFeatureHandler::initialize(const EuclideanTrackDescriptor &track,
    const Eigen::VectorXd &K, Eigen::VectorXd &lw) {

  // this method triangulates a 3D feature from n cameras refining
  // the initial estimate minimizing the reprojection error with Gauss-Newton
  // the original code is due to Andrea Romanoni (andrea.romanoni@polimi.it)

  // convert the camera calibration once for all
  cv::Mat K_cv(3, 3, CV_64F, cv::Scalar(0));
  K_cv.at<double>(0, 0) = K(0);
  K_cv.at<double>(1, 1) = K(0);
  K_cv.at<double>(0, 2) = K(1);
  K_cv.at<double>(1, 2) = K(2);
  K_cv.at<double>(2, 2) = 1.0;

  vector<cv::Mat> curCams;
  vector<cv::Point2f> curPoints;

  //Pack the information to start the Gauss Newton algorithm
  for (auto it = track.zHistory.begin(); it != track.zHistory.end(); ++it) {
    if (it->second.pose->hasBeenEstimated()) {
      cv::Mat T_WS_cv;

      // TODO: this assume system is camera_centric
      buildProjectionMatrix(it->second.pose->getEstimate(), K_cv, T_WS_cv);

      curCams.push_back(T_WS_cv);
      curPoints.push_back(cv::Point2f(it->second.z(0), it->second.z(1)));
    }
  }

  if (curCams.size() < 2) {
    return false;
  }

  // triangulate first and last cameras
  vector<cv::Point2f> firstObsVec, lastObsVec;

  cv::Vec4f triangulated3DPointInitTemp;
  cv::Point3d triangulated3DPointInit, triangulated3DPoint;

  firstObsVec.push_back(curPoints.front());
  lastObsVec.push_back(curPoints.back());

  cv::triangulatePoints(curCams.front(), curCams.back(), firstObsVec,
      lastObsVec, triangulated3DPointInitTemp);

  triangulated3DPointInit.x = triangulated3DPointInitTemp[0]
      / triangulated3DPointInitTemp[3];
  triangulated3DPointInit.y = triangulated3DPointInitTemp[1]
      / triangulated3DPointInitTemp[3];
  triangulated3DPointInit.z = triangulated3DPointInitTemp[2]
      / triangulated3DPointInitTemp[3];

  // run Gauss-Newton with all the cameras
  int resGN = GaussNewton(curCams, curPoints, triangulated3DPointInit,
      triangulated3DPoint);

  lw << triangulated3DPoint.x, triangulated3DPoint.y, triangulated3DPoint.z;
  //*/

  if (resGN != -1) {
    // test that the triangulated points lies in front of each camera
    
    const Eigen::VectorXd &qos = qOS_par->getEstimate();
    
    for (auto it = track.zHistory.begin(); it != track.zHistory.end(); ++it) {

      Eigen::VectorXd testz(1); // it could be a double, but automated equation generation always works with vectors
      const int _OFF = -1;

      if (it->second.pose->hasBeenEstimated()) {
        const Eigen::VectorXd & x = it->second.pose->getEstimate();

#       include "../../ROAMfunctions/generated/ImagePlaneProjection_testZ.cppready"

        if (testz(0) < 0) {
          cerr << "[EuclideanFeatureHandler]: point behind camera: z = " << testz(0) << ". Initialization failed" << endl;
          resGN = -1;
          break;
        }
      }
    }
  }

# ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
  if (resGN != -1) {
    cerr << "[EuclideanFeatureHandler] Track initialization: " << endl;
    cerr << "Triangulated3DPointInit: " << endl;
    cerr << "x: " << triangulated3DPointInit.x;
    cerr << ", y: " << triangulated3DPointInit.y;
    cerr << ", z: " << triangulated3DPointInit.z << endl;

    cerr << "Triangulated3DPoint: " << endl;
    cerr << "x: " << triangulated3DPoint.x;
    cerr << ", y: " << triangulated3DPoint.y;
    cerr << ", z: " << triangulated3DPoint.z << endl;

    cerr << "Measures. " << endl;
    for (auto it = track.zHistory.begin(); it != track.zHistory.end(); ++it) {
      cerr << it->second.t << ": (" << it->second.z(0) << ", "
      << it->second.z(1) << ")" << endl;
    }
  } else {
    cerr << "[EuclideanFeatureHandler] Track initialization: FAILED!" << endl;
  }
# endif

  return resGN > 0;
}

void EuclideanFeatureHandler::buildProjectionMatrix(const Eigen::VectorXd &two,
    const cv::Mat &K, cv::Mat &projMat) {

  cv::Mat T_SW_cv(3, 4, CV_64F);
  
  const Eigen::VectorXd &qos = qOS_par->getEstimate();

  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> Tcw(T_SW_cv.ptr<double>());

  // RF eigen pose (x,q) from world to camera is transformed in
  // cv transformation matrix from camera to world.
  
  const static int _OFF = -1;
  
# include "generated/FromRFtoCV.cppready"

  projMat = K * T_SW_cv;
}

//#define DEBUG_OPTIMIZATION_VERBOSE
//#define DEBUG_OPTIMIZATION

int EuclideanFeatureHandler::GaussNewton(const vector<cv::Mat> &cameras,
    const vector<cv::Point2f> &points, cv::Point3d init3Dpoint,
    cv::Point3d &optimizedPoint) {

  int numMeasures = points.size();
  cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_64F);
  cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_64F);
  cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_64F);

  curEstimate3DPoint.at<double>(0, 0) = init3Dpoint.x;
  curEstimate3DPoint.at<double>(1, 0) = init3Dpoint.y;
  curEstimate3DPoint.at<double>(2, 0) = init3Dpoint.z;

  cv::Mat J, H;
  double last_mse = 0;

  for (int i = 0; i < 25; i++) {

    double mse = 0;
    //compute residuals
    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
      curEstimate3DPointH.at<double>(0, 0) = curEstimate3DPoint.at<double>(0,
          0);
      curEstimate3DPointH.at<double>(1, 0) = curEstimate3DPoint.at<double>(1,
          0);
      curEstimate3DPointH.at<double>(2, 0) = curEstimate3DPoint.at<double>(2,
          0);
      curEstimate3DPointH.at<double>(3, 0) = 1.0;
      cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

      r.at<double>(2 * curMeas, 0) = ((points[curMeas].x
          - cur2DpositionH.at<double>(0, 0) / cur2DpositionH.at<double>(2, 0)));
      mse += r.at<double>(2 * curMeas, 0) * r.at<double>(2 * curMeas, 0);

      r.at<double>(2 * curMeas + 1, 0) = ((points[curMeas].y
          - cur2DpositionH.at<double>(1, 0) / cur2DpositionH.at<double>(2, 0)));
      mse += r.at<double>(2 * curMeas + 1, 0)
          * r.at<double>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION_VERBOSE
      cout << "CurMeas: " << curMeas << endl << "curEstimate3DPointH="
      << curEstimate3DPointH.t() << endl;
      cout << "CurCam" << cameras[curMeas] << endl;
      cout << "cur2DpositionH: "
      << cur2DpositionH.at<double>(0, 0) / cur2DpositionH.at<double>(2, 0)
      << ", "
      << cur2DpositionH.at<double>(1, 0) / cur2DpositionH.at<double>(2, 0)
      << endl;
      cout << "points[curMeas]: " << points[curMeas] << endl;
      cout << "residual on x: " << r.at<double>(2 * curMeas, 0) << endl;
      cout << "residual on y: " << r.at<double>(2 * curMeas + 1, 0) << endl;
      cout << endl;
#endif
    }

// if the error is very low, it  ends the function
    if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000005) {
      break;
    }
    last_mse = mse / (numMeasures * 2);

    if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1)
      return -1;
#ifdef DEBUG_OPTIMIZATION_VERBOSE
    cout << "J: " << J << endl;
    cout << "H: " << H << endl;
#endif

    curEstimate3DPoint += H.inv() * J.t() * r;

#ifdef DEBUG_OPTIMIZATION
    printf("%d %f\n", i, last_mse);
#endif
  }

//  if (last_mse < 1e4/*100 pixels*/) {
  if (true) {
    optimizedPoint.x = curEstimate3DPoint.at<double>(0, 0);
    optimizedPoint.y = curEstimate3DPoint.at<double>(1, 0);
    optimizedPoint.z = curEstimate3DPoint.at<double>(2, 0);
    return 1;
  } else {
    return -1;
  }
}

int EuclideanFeatureHandler::point2D3DJacobian(const vector<cv::Mat> &cameras,
    const cv::Mat &cur3Dpoint, cv::Mat &J, cv::Mat &hessian) {

  int numMeasures = cameras.size();
  cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_64F);

  cur3DPointHomog.at<double>(0, 0) = cur3Dpoint.at<double>(0, 0);
  cur3DPointHomog.at<double>(1, 0) = cur3Dpoint.at<double>(1, 0);
  cur3DPointHomog.at<double>(2, 0) = cur3Dpoint.at<double>(2, 0);
  cur3DPointHomog.at<double>(3, 0) = 1.0;

  J = cv::Mat(2 * numMeasures, 3, CV_64FC1); //2 rows for each point: one for x, the other for y
  hessian = cv::Mat(3, 3, CV_64FC1);

  for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
    cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
    double xH = curReproj.at<double>(0, 0);
    double yH = curReproj.at<double>(1, 0);
    double zH = curReproj.at<double>(2, 0);
    double p00 = cameras[curMeas].at<double>(0, 0);
    double p01 = cameras[curMeas].at<double>(0, 1);
    double p02 = cameras[curMeas].at<double>(0, 2);
    double p10 = cameras[curMeas].at<double>(1, 0);
    double p11 = cameras[curMeas].at<double>(1, 1);
    double p12 = cameras[curMeas].at<double>(1, 2);
    double p20 = cameras[curMeas].at<double>(2, 0);
    double p21 = cameras[curMeas].at<double>(2, 1);
    double p22 = cameras[curMeas].at<double>(2, 2);

    //d(P*X3D)/dX
    J.at<double>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
    J.at<double>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

    //d(P*X3D)/dY
    J.at<double>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
    J.at<double>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

    //d(P*X3D)/dZ
    J.at<double>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
    J.at<double>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
  }

  hessian = J.t() * J;
  double d;
  d = cv::determinant(hessian);
  if (d < 0.00001) {
    //printf("doh");
    return -1;
  } else {
    return 1;
  }
}

} /* namespace ROAMvision */

