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

    /**
     * add an Euclidean 3D random constant process
     */
    static ParameterWrapper_Ptr addEucl3DRandomConstant(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0);

    /**
     * add an Euclidean 3D random constant process
     */
    static ParameterWrapper_Ptr addEucl2DRandomConstant(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0);

    /**
     * add an Euclidean 3D random constant process
     */
    static ParameterWrapper_Ptr addEucl1DRandomConstant(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0);

    /**
     * add an Euclidean 3D random walk process
     *
     * @ param randomWalkNoiseCov_cnt the *continuous* time variance of the Random Walk innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl3DRandomWalk(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0,
        const Eigen::MatrixXd &randomWalkNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

    /**
     * add an Euclidean 1D 1st order Gauss-Markov process
     *
     * @ param gaussMarkovNoiseCov_cnt the *continuous* time variance of the Gauss-Markov innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl1DGaussMarkov(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0,
        const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

    /**
     * add an Euclidean 2D 1st order Gauss-Markov process
     *
     * @ param gaussMarkovNoiseCov_cnt the *continuous* time variance of the Gauss-Markov innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl2DGaussMarkov(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0,
        const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);
    /**
     * add an Euclidean 3D 1st order Gauss-Markov process
     *
     * @ param gaussMarkovNoiseCov_cnt the *continuous* time variance of the Gauss-Markov innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl3DGaussMarkov(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0,
        const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);
    /**
     * add an Euclidean 1D 1st order Gauss-Markov + random constant process
     *
     * @ param gaussMarkovNoiseCov_cnt the *continuous* time variance of the Gauss-Markov innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl1DGaussMarkovPlusRandomConstant(
        FactorGraphFilter *f, std::string name, const Eigen::VectorXd &x0_rc,
        const Eigen::VectorXd &x0_gm, const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

    /**
     * add an Euclidean 2D 1st order Gauss-Markov + random constant process
     *
     * @ param gaussMarkovNoiseCov_cnt the *continuous* time variance of the Gauss-Markov innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl2DGaussMarkovPlusRandomConstant(
        FactorGraphFilter *f, std::string name, const Eigen::VectorXd &x0_rc,
        const Eigen::VectorXd &x0_gm, const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

    /**
     * add an Euclidean 3D 1st order Gauss-Markov + random constant process
     *
     * @ param gaussMarkovNoiseCov_cnt the *continuous* time variance of the Gauss-Markov innovations (in <unit>/Hz)
    */
    static ParameterWrapper_Ptr addEucl3DGaussMarkovPlusRandomConstant(
        FactorGraphFilter *f, std::string name, const Eigen::VectorXd &x0_rc,
        const Eigen::VectorXd &x0_gm, const Eigen::VectorXd &gaussMarkovBeta,
        const Eigen::MatrixXd &gaussMarkovNoiseCov_cnt, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);
    

  protected:

    static ParameterWrapper_Ptr addTimeVarying1DParameter(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);
    static ParameterWrapper_Ptr addTimeVarying2DParameter(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);
    static ParameterWrapper_Ptr addTimeVaryingParameter(FactorGraphFilter *f,
        std::string name, const Eigen::VectorXd &x0, double spacing,
        InterpolationTypes intType = Linear, unsigned int a = 3);

};

} /* namespace ROAMestimation */

#endif /* STOCHASTICPROCESSFACTORY_H_ */
