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
 * AnchoredRectangularObjectFirstM.h
 *
 *  Created on: Feb 20, 2015
 *      Author: davide
 */

#ifndef ANCHOREDRECTANGULAROBJECT_FIRST_H_
#define ANCHOREDRECTANGULAROBJECT_FIRST_H_

#include <Eigen/Dense>

namespace ROAMfunctions {

class AnchoredRectangularObjectFirstM {

public:
	static const bool _usedComponents[];

	static const std::string _paramsNames[];
	static const int _nParams = 4;

	static const unsigned int _ORDER = 0;

	static const unsigned int _ERROR_SIZE = 8;
	static const unsigned int _NOISE_SIZE = 8;
	static const unsigned int _MEASUREMENT_SIZE = 8;

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

		Eigen::Map<Eigen::Vector2d> d(params[0]);

		Eigen::Map<Eigen::Vector3d> fohp(params[1]);
		Eigen::Map<Eigen::Vector4d> foq(params[2]);

		Eigen::Map<Eigen::Matrix<double, 9, 1> > cm(params[3]);

		Eigen::MatrixBase<T> & err =
				const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

# 	include "generated/AnchoredRectangularObject_first_Err.cppready"

		return false;
	}

	template<typename T>
	bool errorJacobian(const Eigen::VectorXd &x, double **params,
			const Eigen::VectorXd& z, int wrt,
			Eigen::MatrixBase<T> const &const_ret) {

		Eigen::Map<Eigen::Vector2d> d(params[0]);

		Eigen::Map<Eigen::Vector3d> fohp(params[1]);
		Eigen::Map<Eigen::Vector4d> foq(params[2]);

		Eigen::Map<Eigen::Matrix<double, 9, 1> > cm(params[3]);

		Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

		const static int _OFF = -1;

		switch (wrt) {

		case -2: // jacobian wrt to q
		{
#     		include "generated/AnchoredRectangularObject_first_JErrQ.cppready"
			return true;
			break;
		}

		case -1: // jacobian wrt pose
		{
#   		include "generated/AnchoredRectangularObject_first_JErrPose.cppready"
			return true;
			break;
		}
		case 0: // jacobian wrt to noises
		{
			// it is the identity matrix
			// #include "generated/RectangularObject_JErrNoises.cppready"
			return false;
			break;
		}
		case 1: // jacobian wrt ff and w
		{
#  			include "generated/AnchoredRectangularObject_first_JErrDim.cppready"
			return true;
			break;
		}
		case 2: // jacobian wrt homogeneous point
		{
# 		 	include "generated/AnchoredRectangularObject_first_JErrFOhp.cppready"
			return true;
			break;
		}
		case 3: // jacobian wrt object orientation wrt anchor frame
		{
#  			include "generated/AnchoredRectangularObject_first_JErrFOq.cppready"
			return true;
			break;
		}

			/*
			 case 5: // jacobian wrt camera matrix
			 {
			 #			include "generated/IMUint_JErrBw.cppready"
			 return true;
			 break;
			 }
			 */
		}

		assert(false);
		return false;
	}
};

} /* namespace ROAMfunctions */

#endif /* ANCHOREDRECTANGULAROBJECT_FIRST_H_ */
