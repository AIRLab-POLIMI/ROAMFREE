/*
 * testPoseGetters.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: davide
 */

#include <random>
#include <iostream>

#include "ROAMestimation/ROAMestimation.h"

using namespace std;
using namespace ROAMestimation;

default_random_engine generator;
uniform_int_distribution<int> distribution(0,99);

FactorGraphFilter *f;

bool getRandomPose() {
  int N = distribution(generator);

  PoseVertexWrapper_Ptr pose = f->getNthPose(N);

  cerr << pose->getTimestamp() << " " << N << endl;

  if (fabs(pose->getTimestamp() - (99-N) ) < 0.5) {
    return true;
  }
  return false;
}


int main(int argc, char *argv[]) {

   f = FactorGraphFilterFactory::getNewFactorGraphFilter();

  for (int n = 0; n < 100; n++) {
    f->addPose(n);
  }

  for (int n =0; n< 1000; n++) {
    assert(getRandomPose());
  }
}


