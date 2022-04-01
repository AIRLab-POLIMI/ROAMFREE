/*
 * TestSE3InterpolationEdge.cpp
 *
 *  Created on: Aug 31, 2016
 *      Author: davide
 */

#include "ROAMestimation/ROAMestimation.h"

using namespace std;
using namespace ROAMestimation;

int main(int argc, char *argv[]) {
  FactorGraphFilter *f;

  f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setLowLevelLogging(true, "/tmp/roamfree");
  f->setWriteGraph(true);
  f->setDeadReckoning(false);

  PoseVertexWrapper_Ptr x1 = f->addPose(0);
  Eigen::VectorXd x1e(7);
  x1e << 0,0,0,0.710997408193224,0.360544029310185,0.594459869568306,0.105395217842782;
  x1->setEstimate(x1e);
  x1->setFixed(true);

  PoseVertexWrapper_Ptr x2 = f->addPose(1);
  Eigen::VectorXd x2e(7);
  x2e << 1,2,3,0.263360579192421,0.571813128030932,0.494678363680335,0.599136268678053;
  x2->setEstimate(x2e);
  x2->setFixed(true);

  ParameterWrapper_Ptr delayParam = f->addConstantParameter("delay", 0, true);

  for (int k = 1; k< 100; k++) {
    PoseVertexWrapper_Ptr xi = f->addInterpolatingPose(0.01*k, delayParam, Eigen::MatrixXd::Identity(6,6));
  }

  f->estimate(10);
}
