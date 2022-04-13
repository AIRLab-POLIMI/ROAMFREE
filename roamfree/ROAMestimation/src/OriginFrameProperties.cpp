/*
 * Datum.cpp
 *
 *  Created on: Jul 2, 2021
 *      Author: davide
 */

#include "../include/ROAMestimation/OriginFrameProperties.h"

namespace ROAMestimation {

OriginFrameProperties::OriginFrameProperties() {

  // default configuration leads to the fusion on an Euclidean frame
  // which does not rotate and in which the gravity has the same direction everywhere
  // as it is commonly done in robotics

  frametype = TangentPlane;

  earthrate = 0;

  epshift << 0.0, 0.0, 0.0;

  epa = std::numeric_limits<double>::infinity();
  epb = 1.0;
}

void OriginFrameProperties::evaluateGravityVectorAt(const Eigen::VectorXd &x2,
    Eigen::Vector3d &gravityVector) {

  if (frametype == TangentPlane) {
    gravityVector << 0.0, 0.0, 1.0;
  } else if (frametype == ECEF) {
#   include "generated/GravityVector.cppready"
  }

}

void OriginFrameProperties::evaluateRLocalENUToWorldAt(const Eigen::VectorXd &x2,
    Eigen::Matrix3d &RLocalENUtoWorld) {
  
  if (frametype == TangentPlane) {
    RLocalENUtoWorld = Eigen::Matrix3d::Identity();
  } else if (frametype == ECEF) {
#   include "generated/RLocalENUToWorld.cppready"
  }
}

}
