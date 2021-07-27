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
 * StochasticProcessFactory.cpp
 *
 *  Created on: Sep 24, 2015
 *      Author: davide
 */

#include "StochasticProcessFactory.h"

#include "ParameterWrapper.h"
#include "FactorGraphFilter.h"

namespace ROAMestimation {




ParameterWrapper_Ptr StochasticProcessFactory::addEucl1DRandomConstant(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0) {
  return f->addConstantParameter(Euclidean1D, name, x0, false);
}


ParameterWrapper_Ptr StochasticProcessFactory::addEucl2DRandomConstant(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0) {
  return f->addConstantParameter(Euclidean2D, name, x0, false);
}


ParameterWrapper_Ptr StochasticProcessFactory::addEucl3DRandomConstant(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0) {
  return f->addConstantParameter(Euclidean3D, name, x0, false);
}


ParameterWrapper_Ptr StochasticProcessFactory::addEucl3DRandomWalk(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& randomWalkNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p = addTimeVaryingParameter(f, name, x0, spacing,
      intType, a);

  p->setProcessModelType(RandomWalk);
  p->setRandomWalkNoiseCov(randomWalkNoiseCov_cnt);

  return p;
}

ParameterWrapper_Ptr StochasticProcessFactory::addEucl1DGaussMarkov(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    const Eigen::VectorXd& gaussMarkovBeta,
    const Eigen::MatrixXd& gaussMarkovNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p = addTimeVarying1DParameter(f, name, x0, spacing,
      intType, a);

  p->setProcessModelType(GaussMarkov);
  p->setGaussMarkovNoiseCov(gaussMarkovNoiseCov_cnt);
  p->setGaussMarkovBeta(gaussMarkovBeta);

  return p;
}

ParameterWrapper_Ptr StochasticProcessFactory::addEucl2DGaussMarkov(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    const Eigen::VectorXd& gaussMarkovBeta,
    const Eigen::MatrixXd& gaussMarkovNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p = addTimeVarying2DParameter(f, name, x0, spacing,
      intType, a);

  p->setProcessModelType(GaussMarkov);
  p->setGaussMarkovNoiseCov(gaussMarkovNoiseCov_cnt);
  p->setGaussMarkovBeta(gaussMarkovBeta);

  return p;
}


ParameterWrapper_Ptr StochasticProcessFactory::addEucl3DGaussMarkov(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    const Eigen::VectorXd& gaussMarkovBeta,
    const Eigen::MatrixXd& gaussMarkovNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p = addTimeVaryingParameter(f, name, x0, spacing,
      intType, a);

  p->setProcessModelType(GaussMarkov);
  p->setGaussMarkovNoiseCov(gaussMarkovNoiseCov_cnt);
  p->setGaussMarkovBeta(gaussMarkovBeta);

  return p;
}

ParameterWrapper_Ptr StochasticProcessFactory::addEucl1DGaussMarkovPlusRandomConstant(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0_rc,
    const Eigen::VectorXd& x0_gm, const Eigen::VectorXd& gaussMarkovBeta,
    const Eigen::MatrixXd& gaussMarkovNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p_gm = addEucl1DGaussMarkov(f, name + "_GM", x0_gm,
      gaussMarkovBeta, gaussMarkovNoiseCov_cnt, spacing, intType, a);
  ParameterWrapper_Ptr p_rc = f->addConstantParameter(Euclidean1D, name + "_RC",
      x0_rc, false);

  ParameterWrapperVector_Ptr toblend(new ParameterWrapperVector);
  toblend->push_back(p_gm);
  toblend->push_back(p_rc);

  return f->addParameterBlender(Euclidean1D, name, toblend);
}


ParameterWrapper_Ptr StochasticProcessFactory::addEucl2DGaussMarkovPlusRandomConstant(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0_rc,
    const Eigen::VectorXd& x0_gm, const Eigen::VectorXd& gaussMarkovBeta,
    const Eigen::MatrixXd& gaussMarkovNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p_gm = addEucl2DGaussMarkov(f, name + "_GM", x0_gm,
      gaussMarkovBeta, gaussMarkovNoiseCov_cnt, spacing, intType, a);
  ParameterWrapper_Ptr p_rc = f->addConstantParameter(Euclidean2D, name + "_RC",
      x0_rc, false);

  ParameterWrapperVector_Ptr toblend(new ParameterWrapperVector);
  toblend->push_back(p_gm);
  toblend->push_back(p_rc);

  return f->addParameterBlender(Euclidean2D, name, toblend);
}


ParameterWrapper_Ptr StochasticProcessFactory::addEucl3DGaussMarkovPlusRandomConstant(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0_rc,
    const Eigen::VectorXd& x0_gm, const Eigen::VectorXd& gaussMarkovBeta,
    const Eigen::MatrixXd& gaussMarkovNoiseCov_cnt, double spacing,
    InterpolationTypes intType, unsigned int a) {

  ParameterWrapper_Ptr p_gm = addEucl2DGaussMarkov(f, name + "_GM", x0_gm,
      gaussMarkovBeta, gaussMarkovNoiseCov_cnt, spacing, intType, a);
  ParameterWrapper_Ptr p_rc = f->addConstantParameter(Euclidean2D, name + "_RC",
      x0_rc, false);

  ParameterWrapperVector_Ptr toblend(new ParameterWrapperVector);
  toblend->push_back(p_gm);
  toblend->push_back(p_rc);

  return f->addParameterBlender(Euclidean3D, name, toblend);
}


ParameterWrapper_Ptr StochasticProcessFactory::addTimeVarying1DParameter(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    double spacing, InterpolationTypes intType, unsigned int a) {
  if (intType == Linear) {
    return f->addLinearlyInterpolatedParameter(Euclidean1D, name, x0, false,
        spacing);
  } else {
    return f->addLimitedBandwithParameter(Euclidean1D, name, x0, false,
        1.0 / spacing / 2.0, a);
  }
}


ParameterWrapper_Ptr StochasticProcessFactory::addTimeVarying2DParameter(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    double spacing, InterpolationTypes intType, unsigned int a) {
  if (intType == Linear) {
    return f->addLinearlyInterpolatedParameter(Euclidean2D, name, x0, false,
        spacing);
  } else {
    return f->addLimitedBandwithParameter(Euclidean2D, name, x0, false,
        1.0 / spacing / 2.0, a);
  }
}

ParameterWrapper_Ptr StochasticProcessFactory::addTimeVaryingParameter(
    FactorGraphFilter* f, std::string name, const Eigen::VectorXd& x0,
    double spacing, InterpolationTypes intType, unsigned int a) {
  if (intType == Linear) {
    return f->addLinearlyInterpolatedParameter(Euclidean3D, name, x0, false,
        spacing);
  } else {
    return f->addLimitedBandwithParameter(Euclidean3D, name, x0, false,
        1.0 / spacing / 2.0, a);
  }
}

} /* namespace ROAMestimation */
