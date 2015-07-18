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

#ifndef GENERICCALCULATOR_H_
#define GENERICCALCULATOR_H_

#include "ParameterTemporaries.h"

namespace ROAMestimation
{
class GenericCalculator
{
public:
	GenericCalculator(std::vector<ParameterTemporaries>& params) :
				_params(params)
	{
		setParameters();
	}

	virtual bool calculate(const Eigen::VectorXd& x2) = 0;
	virtual bool calculate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
				const double& Dt12) = 0;
	virtual bool calculate(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
				const Eigen::VectorXd& x2, const double &Dt01,
				const double& Dt12) = 0;

	virtual ~GenericCalculator() {}

private:
	inline void setParameters()
	{
		// S(O) and qOS are special parameters and are assumed to be constant
		//otherwise I have complications in the Jacobian computation

		sO1 = _params[0].value(0);
		sO2 = _params[1].value(0);
		sO3 = _params[2].value(0);

		const double& qOS1_raw = _params[3].value(0);
		const double& qOS2_raw = _params[4].value(0);
		const double& qOS3_raw = _params[5].value(0);

		/*
		 * this is an ugly workaround since misalignment parameter is separated into components
		 * and its manifold structure is lost
		 */

		const double& qOS_norm = std::pow(qOS1_raw, 2) + std::pow(qOS2_raw, 2)
					+ std::pow(qOS3_raw, 2);

		qOS1 = _params[3].value(0) / (qOS_norm > 1.0 ? qOS_norm : 1.0);
		qOS2 = _params[4].value(0) / (qOS_norm > 1.0 ? qOS_norm : 1.0);
		qOS3 = _params[5].value(0) / (qOS_norm > 1.0 ? qOS_norm : 1.0);
	}

protected:
	double sO1;
	double sO2;
	double sO3;

	double qOS1;
	double qOS2;
	double qOS3;

	const static int _OFF = -1;

private:
	std::vector<ParameterTemporaries>& _params;

};

}

#endif /* GENERICCALCULATOR_H_ */
