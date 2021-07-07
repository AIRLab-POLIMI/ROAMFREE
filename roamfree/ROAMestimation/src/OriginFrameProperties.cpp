/*
 * Datum.cpp
 *
 *  Created on: Jul 2, 2021
 *      Author: davide
 */

#include "../include/ROAMestimation/OriginFrameProperties.h"

OriginFrameProperties::OriginFrameProperties() {

  // default configuration leads to the fusion on an Euclidean frame
  // which does not rotate and in which the gravity has the same direction everywhere
  // as it is commonly done in robotics

  earthrate = 0;

  epshift << 0.0, 0.0, 0.0;

  epa = std::numeric_limits<double>::infinity();
  epb = 1.0;
}
