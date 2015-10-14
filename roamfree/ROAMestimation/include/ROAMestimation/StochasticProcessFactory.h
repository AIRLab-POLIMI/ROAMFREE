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
 * StochasticProcessFactory.h
 *
 *  Created on: Sep 24, 2015
 *      Author: davide
 */

#ifndef STOCHASTICPROCESSFACTORY_H_
#define STOCHASTICPROCESSFACTORY_H_

#include <string>

#include <Eigen/Dense>

#include "Enums.h"
#include "interfaceTypes.h"

namespace ROAMestimation {

class StochasticProcessFactory {
  public:

    static ParameterWrapper_Ptr addEucl3DRandomConstant(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0);

    static ParameterWrapper_Ptr addEucl3DRandomWalk(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0,
        const Eigen::MatrixXd &randomWalkNoiseCov, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

    static ParameterWrapper_Ptr addEucl3DGaussMarkov(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0,
        const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

    static ParameterWrapper_Ptr addEucl3DGaussMarkovPlusRandomConstant(
        FactorGraphFilter *f, std::string name, const Eigen::VectorXd &x0_rc,
        const Eigen::VectorXd &x0_gm, const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

  protected:
    static ParameterWrapper_Ptr addTimeVaryingParameter(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

};

} /* namespace ROAMestimation */

#endif /* STOCHASTICPROCESSFACTORY_H_ */
