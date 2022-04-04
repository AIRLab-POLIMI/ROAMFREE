/*
Copyright (c) 2013-2016 Politecnico di Milano.
All rights reserved. This program and the accompanying materials
are made available under the terms of the GNU Lesser Public License v3
which accompanies this distribution, and is available at
https://www.gnu.org/licenses/lgpl.html

Contributors:
    Davide A. Cucci (davide.cucci@epfl.ch)    
	Kenneth Joseph Paul (kenneth.josephpaul@epfl.ch)
*/

/*
 *  Modified on: July 13, 2021
 *      Author: kenneth
*/

/*
 * IMUImtegralM.h
 *
 *  Created on: March 7, 2017
 *      Author: davide

 */

#ifndef QUADDYNAMICMODELM_H_
#define QUADDYNAMICMODELM_H_

#include <Eigen/Dense>

namespace ROAMfunctions
{

	class QuadDynamicModelM
	{

	public:
		static const bool _usedComponents[];

		static const std::string _paramsNames[];
		static const int _nParams = 12;

		static const unsigned int _ORDER = 2;

		static const unsigned int _ERROR_SIZE = 6;
		static const unsigned int _NOISE_SIZE = 6;
		static const unsigned int _MEASUREMENT_SIZE = 4;

		const std::string *getParamsList()
		{
			return _paramsNames;
		}
		const int getNParams()
		{
			return _nParams;
		}

		bool predict(const Eigen::VectorXd &x, double **params,
					 const Eigen::VectorXd &z, double dt, Eigen::VectorXd &xhat)
		{

			xhat = x;
			return true;
		}

		template <typename T>
		bool error(const Eigen::VectorXd &x, double **params,
				   const Eigen::VectorXd &z, Eigen::MatrixBase<T> const &const_ret)
		{

			Eigen::Map<Eigen::VectorXd> windxy(params[0], 2);
			Eigen::Map<Eigen::VectorXd> windz(params[1], 1);
			Eigen::Map<Eigen::VectorXd> dragmodel(params[2], 2);
			Eigen::Map<Eigen::VectorXd> ldrag(params[3], 3);
			Eigen::Map<Eigen::VectorXd> qdrag(params[4], 3);
			Eigen::Map<Eigen::VectorXd> rdrag(params[5], 3);
			Eigen::Map<Eigen::VectorXd> lambda(params[6], 1);
			Eigen::Map<Eigen::VectorXd> mu(params[7], 2);
			Eigen::Map<Eigen::VectorXd> mc(params[8], 2);
			Eigen::Map<Eigen::VectorXd> cp(params[9], 3);
			Eigen::Map<Eigen::VectorXd> ibd(params[10], 3);
			Eigen::Map<Eigen::VectorXd> ibod(params[11], 3);

			Eigen::MatrixBase<T> &err = const_cast<Eigen::MatrixBase<T> &>(const_ret);

			const static int _OFF = -1;

#include "generated/QuadDynamicModel_Err.cppready"

			//err.tail(3) << 0,0,0;
			return false;
		}

		template <typename T>
		bool errorJacobian(const Eigen::VectorXd &x, double **params,
						   const Eigen::VectorXd &z, int wrt,
						   Eigen::MatrixBase<T> const &const_ret)
		{

			Eigen::Map<Eigen::VectorXd> windxy(params[0], 2);
			Eigen::Map<Eigen::VectorXd> windz(params[1], 1);
			Eigen::Map<Eigen::VectorXd> dragmodel(params[2], 2);
			Eigen::Map<Eigen::VectorXd> ldrag(params[3], 3);
			Eigen::Map<Eigen::VectorXd> qdrag(params[4], 3);
			Eigen::Map<Eigen::VectorXd> rdrag(params[5], 3);
			Eigen::Map<Eigen::VectorXd> lambda(params[6], 1);
			Eigen::Map<Eigen::VectorXd> mu(params[7], 2);
			Eigen::Map<Eigen::VectorXd> mc(params[8], 2);
			Eigen::Map<Eigen::VectorXd> cp(params[9], 3);
			Eigen::Map<Eigen::VectorXd> ibd(params[10], 3);
			Eigen::Map<Eigen::VectorXd> ibod(params[11], 3);

			Eigen::MatrixBase<T> &J = const_cast<Eigen::MatrixBase<T> &>(const_ret);

			const static int _OFF = -1;

			switch (wrt)
			{

			case -12: // jacobian wrt to previous omega
    		{
#include "generated/QuadDynamicModel_JErrWprev.cppready"
      			return true;
      			break;
    		}

    		case -11: // jacobian wrt to previous v
    		{
#include "generated/QuadDynamicModel_JErrVprev.cppready"
      			return true;
      			break;
    		}

    		case -10: // jacobian wrt to previous q
    		{
#include "generated/QuadDynamicModel_JErrQprev.cppready"
      			return true;
      			break;
    		}

			case -6: // jacobian wrt to Alpha
			{
#include "generated/QuadDynamicModel_JErrAlpha.cppready"
				return false; // "it is useless to evaluate me again"
				break;
			}
			case -5: // jacobian wrt to A
			{
#include "generated/QuadDynamicModel_JErrA.cppready"
				return false; // "it is useless to evaluate me again"
				break;
			}
			case -4: // jacobian wrt to W
			{
#include "generated/QuadDynamicModel_JErrW.cppready"
				return true;
				break;
			}
			case -3: // jacobian wrt to V
			{
#include "generated/QuadDynamicModel_JErrV.cppready"
				return true;
				break;
			}
			case -2: // jacobian wrt to Q
			{
#include "generated/QuadDynamicModel_JErrQ.cppready"
				return true;
				break;
			}

			case 0: // jacobian wrt to noises
			{
#include "generated/QuadDynamicModel_JErrNoises.cppready"
				return true; // it is not the identity matrix
				break;
			}
			case 1: // jacobian wrt wind xy
			{
#include "generated/QuadDynamicModel_JErrWINDXY.cppready"
				return true;
				break;
			}
			case 2: // jacobian wrt wind Z
			{
#include "generated/QuadDynamicModel_JErrWINDZ.cppready"
				return true;
				break;
			}
			case 3: // jacobian wrt drag model  -----> Remove this case
			{
				assert(false);
				break;
			}
			case 4: // jacobian wrt linear drag
			{
#include "generated/QuadDynamicModel_JErrLDRAG.cppready"
				return true;
				break;
			}
			case 5: // jacobian wrt quadratic drag
			{
#include "generated/QuadDynamicModel_JErrQDRAG.cppready"
				return true;
				break;
			}
			case 6: // jacobian wrt rotor drag
			{
#include "generated/QuadDynamicModel_JErrRDRAG.cppready"
				return true;
				break;
			}
			case 7: // jacobian wrt lambda parameter(s)
			{
#include "generated/QuadDynamicModel_JErrLAMBDA.cppready"
				return true;
				break;
			}
			case 8: // jacobian wrt mu parameter(s)
			{
#include "generated/QuadDynamicModel_JErrMU.cppready"
				return true;
				break;
			}
			case 9: // jacobian wrt rotors mechanical propertiees
			{
#include "generated/QuadDynamicModel_JErrMC.cppready"
				return true;
				break;
			}
			case 10: // jacobian wrt copter properties
			{
#include "generated/QuadDynamicModel_JErrCP.cppready"
				return true;
				break;
			}
			case 11: // jacobian wrt inertia matrix diagonal
			{
#include "generated/QuadDynamicModel_JErrIBD.cppready"
				return true;
				break;
			}
			case 12: // jacobian wrt inertia off diagonal elements
			{
#include "generated/QuadDynamicModel_JErrIBOD.cppready"
				return true;
				break;
			}

			
			}

			assert(false);
			return false;
		}
	};

} /* namespace ROAMfunctions */

#endif /* QUADDYNAMICMODELM_H_ */
