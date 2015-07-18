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
 * IMUImtegralM.h
 *
 *  Created on: Sept 9, 2014
 *      Author: davide
 */

#ifndef IMUINTEGRALDELTAQM_H_
#define IMUINTEGRALDELTAQM_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class IMUImtegralDeltaQM {

public:
	static const bool _usedComponents[];

	static const std::string _paramsNames[];
	static const int _nParams = 1;

	static const unsigned int _ORDER = 1;

	static const unsigned int _ERROR_SIZE = 3;
	static const unsigned int _NOISE_SIZE = 3;
	static const unsigned int _MEASUREMENT_SIZE = 16;

	const std::string* getParamsList() {
		return _paramsNames;
	}
	const int getNParams() {
		return _nParams;
	}

	bool predict(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {
		return false;
	}

	template<typename T>
	bool error(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

		Eigen::Map<Eigen::VectorXd> bw(params[0], 3);

		Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

		static Eigen::VectorXd w(1);
		double sign;

		{
#			include "generated/IMUIntegralDeltaQ_TestW.cppready"
		}

		sign = w(0) < 0 ? -1.0 : 1.0;

#   include "generated/IMUIntegralDeltaQ_Err.cppready"

		return false;
	}

	template<typename T>
	bool errorJacobian(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, int wrt,
			Eigen::MatrixBase<T> const &const_ret) {

		Eigen::Map<Eigen::VectorXd> bw(params[0], 3);

		Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

		static Eigen::VectorXd w(1);
		double sign;

		{
#			include "generated/IMUIntegralDeltaQ_TestW.cppready"
		}

		sign = w(0) < 0 ? -1.0 : 1.0;

		switch (wrt) {

		case -8: // jacobian wrt dispQ
		{
#     include "generated/IMUIntegralDeltaQ_JErrdQ12.cppready"
			return true;
			break;
		}

		case 0: // jacobian wrt to noises
		{
#			include "generated/IMUIntegralDeltaQ_JErrNoises.cppready"
			return true; // it is NOT the identity matrix
			break;
		}
		case 1: // jacobian wrt gyroscope bias
		{
#  	include "generated/IMUIntegralDeltaQ_JErrBw.cppready"
			return true;
			break;
		}

		}

		assert(false);
		return false;
	}
};

} /* namespace ROAMfunctions */

#endif /* IMUINTEGRALDELTAQM_H_ */
