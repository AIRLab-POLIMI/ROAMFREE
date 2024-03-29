/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)
    Davide Tateo (davide.tateo@polimi.it)    
*/

#ifndef AUGMENTEDSTATECALCULATOR_H_
#define AUGMENTEDSTATECALCULATOR_H_

#include "GenericCalculator.h"

namespace ROAMestimation
{

class AugmentedStateCalculator: public GenericCalculator
{
public:
	AugmentedStateCalculator(std::vector<ParameterTemporaries>& params,
				Eigen::VectorXd& x, const bool* usedComponents) :
				GenericCalculator(params), _x(x),
				_usedComponents(usedComponents)
	{

	}

	inline bool calculate(const Eigen::VectorXd& x2)
	{

		if (_usedComponents[POSITION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(0, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_POSE.cppready"
		}
		if (_usedComponents[ORIENTATION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(3, 4);

#     include "generated/BackwardAugmentedStateEstimator_v7_Q.cppready"
		}

		return true;
	}

	inline bool calculate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
				const double& Dt12)
	{
		calculate(x2);

		if (_usedComponents[LINEARVELOCITY])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(7, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_V.cppready"
		}
		if (_usedComponents[ANGULARVELOCITY])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(10, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_W.cppready"
		}

		if (_usedComponents[DELTA_POSITION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(19, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_Disp.cppready"
		}
		if (_usedComponents[DELTA_ORIENTATION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(22, 4);

#     include "generated/BackwardAugmentedStateEstimator_v7_DispQ.cppready"
		}
		if (_usedComponents[PREVIOUS_ORIENTATION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(29, 4);

#     include "generated/BackwardAugmentedStateEstimator_v7_Qprev.cppready"			
		}

		return true;

	}

	inline bool calculate(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
				const Eigen::VectorXd& x2, const double &Dt01,
				const double& Dt12)
	{
		calculate(x1, x2, Dt12);

		if (_usedComponents[ACCELERATION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(13, 3);

			OriginFrameProperties::getInstance().evaluateGravityVectorAt(x2, gravityVector);

#     include "generated/BackwardAugmentedStateEstimator_v7_A.cppready"
		}
		if (_usedComponents[ANGULARACCELERATION])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(16, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_Alpha.cppready"
		}
		if (_usedComponents[IMUINT_DELTAPOSE])
		{

			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(26, 3);

			OriginFrameProperties::getInstance().evaluateGravityVectorAt(x2, gravityVector);

#     include "generated/BackwardAugmentedStateEstimator_v7_IMUintdP.cppready"
		}
		if (_usedComponents[PREVIOUS_LINEARVELOCITY])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(33, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_Vprev.cppready"
		}
		if (_usedComponents[PREVIOUS_ANGULARVELOCITY])
		{
			Eigen::VectorBlock<Eigen::VectorXd> &&part = _x.segment(36, 3);

#     include "generated/BackwardAugmentedStateEstimator_v7_Wprev.cppready"
		}

		return true;

	}

private:
	//Augmented state
	Eigen::VectorXd& _x;
	const bool* _usedComponents;

};

}

#endif /* AUGMENTEDSTATECALCULATOR_H_ */
