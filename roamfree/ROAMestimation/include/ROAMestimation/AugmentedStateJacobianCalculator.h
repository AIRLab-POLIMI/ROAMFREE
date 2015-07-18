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

#ifndef AUGMENTEDSTATEJACOBIANCALCULATOR_H_
#define AUGMENTEDSTATEJACOBIANCALCULATOR_H_

#include "GenericCalculator.h"
#include "Enums.h"

namespace ROAMestimation
{

template<typename Derived, int ORDER>
class AugmentedStateJacobianCalculator: public GenericCalculator
{
public:
	AugmentedStateJacobianCalculator(std::vector<ParameterTemporaries>& params, int y, int x, Eigen::MatrixBase<Derived> &J) :
				GenericCalculator(params), x(x), y(y), J(J)
	{
		// J is sparse! We clear the non zero entries without freeing the memory
		J.setZero();
	}

	/**
	 * this function computes the (i,j) block of the Jacobian matrix of _x wrt _vertices()
	 * and stores the results in J
	 *
	 * N.B. in the following it is not that everything is full of DENSE code... :)
	 * a lot of files are empty or near empty
	 *
	 * @param y is the row [x,q,v,omega,a,alpha,dispx,dispq,imuintdp,imuintdq], y in 0-9
	 * @param x is the col, i.e. the vertex, [x(t), dt12, x(t-1), dt01, x(t-2), SOx, SOy, SOz, qOSx, qOSy, qOSz]
	 *          note that depending on the MT::_ORDER of the edge, older poses and dt may be missing.
	 */

	bool calculate(const Eigen::VectorXd& x2)
	{

		if (x == 0) //TODO non si può mettere nell'altro switch?
		{ // with respect to x(t)
			switch (y)
			{
				case POSITION:
				{
#    		  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSEX2.cppready"
					return true;
				}
				case ORIENTATION:
				{
#      		include "generated/BackwardAugmentedStateEstimator_v6_JAugQX2.cppready"
					return true;
				}
			}
		}

		switch (y)
		{
			case POSITION:
			{
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#     		  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSESO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSESO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSESO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSEqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSEqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#       		include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSEqOS3.cppready"
						return true;
					}
				}

				break;
			}
			case ORIENTATION:
			{
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugQSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugQSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#     		  include "generated/BackwardAugmentedStateEstimator_v6_JAugQSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugQqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugQqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#      		  include "generated/BackwardAugmentedStateEstimator_v6_JAugQqOS3.cppready"
						return true;
					}
				}

				break;
			}
		}

		return false;
	}

	bool calculate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
				const double& Dt12)
	{
		if(calculate(x2))
			return true;

		if (x == 0) //TODO idem come sopra...
		{ // with respect to x(t)
			switch (y)
			{
				case LINEARVELOCITY:
				{
					assert(ORDER > 0);
#       		include "generated/BackwardAugmentedStateEstimator_v6_JAugVX2.cppready"
					return true;
				}
				case ANGULARVELOCITY:
				{
					assert(ORDER > 0);
#       	  include "generated/BackwardAugmentedStateEstimator_v6_JAugWX2.cppready"
					return true;
				}
				case DELTA_POSITION:
				{
					assert(ORDER > 0);
#       	  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispX2.cppready"
					return true;
				}
				case DELTA_ORIENTATION:
				{
					assert(ORDER > 0);
#       	  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQX2.cppready"
					return true;
				}

			}
		}

		assert(x != 1); // it would mean jacobian wrt dt vertex..

		if (x == 2)  //TODO idem come sopra...
		{ // with respect to x(t-1)
			switch (y)
			{
				case POSITION:
				{
#        	  include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSEX1.cppready"
					return true;
				}
				case ORIENTATION:
				{
#        	  include "generated/BackwardAugmentedStateEstimator_v6_JAugQX1.cppready"
					return true;
				}
				case LINEARVELOCITY:
				{
#         	include "generated/BackwardAugmentedStateEstimator_v6_JAugVX1.cppready"
					return true;
				}
				case ANGULARVELOCITY:
				{
#         	include "generated/BackwardAugmentedStateEstimator_v6_JAugWX1.cppready"
					return true;
				}
				case DELTA_POSITION:
				{
#         	include "generated/BackwardAugmentedStateEstimator_v6_JAugDispX1.cppready"
					return true;
				}
				case DELTA_ORIENTATION:
				{
#         	include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQX1.cppready"
					return true;
				}

			}
		}

		switch (y)
		{
			case LINEARVELOCITY:
			{
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugVSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugVSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugVSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugVqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugVqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugVqOS3.cppready"
						return true;
					}
				}

				break;
			}
			case ANGULARVELOCITY:
			{
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#         		include "generated/BackwardAugmentedStateEstimator_v6_JAugWSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#         		include "generated/BackwardAugmentedStateEstimator_v6_JAugWSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#         		include "generated/BackwardAugmentedStateEstimator_v6_JAugWSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#         		include "generated/BackwardAugmentedStateEstimator_v6_JAugWqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#         		include "generated/BackwardAugmentedStateEstimator_v6_JAugWqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#         		include "generated/BackwardAugmentedStateEstimator_v6_JAugWqOS3.cppready"
						return true;
					}
				}

				break;
			}
			case DELTA_POSITION:
			{
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispqOS3.cppready"
						return true;
					}
				}

				break;
			}
			case DELTA_ORIENTATION:
			{
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQqOS3.cppready"
						return true;
					}
				}

				break;
			}

		}

		return false;
	}

	bool calculate(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
				const Eigen::VectorXd& x2, const double &Dt01,
				const double& Dt12)
	{
		if(calculate(x1, x2, Dt12))
			return true;

		if (x == 0)
		{ // with respect to x(t)
			switch (y)
			{
				case ACCELERATION:
				{
					assert(ORDER > 1);
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugAX2.cppready"
					return true;
				}
				case ANGULARACCELERATION:
				{
					assert(ORDER > 1);
#          	  include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaX2.cppready"
					return true;
				}
				case IMUINT_DELTAPOSE:
				{
					assert(ORDER > 1);
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPX2.cppready"
					return true;
				}
			}
		}

		if (x == 2)
		{ // with respect to x(t-1)
			switch (y)
			{
				case ACCELERATION:
				{
					assert(ORDER > 1);
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugAX1.cppready"
					return true;
				}
				case ANGULARACCELERATION:
				{
					assert(ORDER > 1);
#           	include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaX1.cppready"
					return true;
				}
				case IMUINT_DELTAPOSE:
				{
					assert(ORDER > 1);
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPX1.cppready"
					return true;
				}
			}
		}

		assert(x != 3); // it would mean jacobian wrt dt vertex..

		if (x == 4)
		{ // with respect to x(t-2)
			switch (y)
			{
				case POSITION:
				{
#           	include "generated/BackwardAugmentedStateEstimator_v6_JAugPOSEX0.cppready"
					return true;
				}
				case ORIENTATION:
				{
#           	include "generated/BackwardAugmentedStateEstimator_v6_JAugQX0.cppready"
					return true;
				}
				case LINEARVELOCITY:
				{
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugVX0.cppready"
					return true;
				}
				case ANGULARVELOCITY:
				{
#           	include "generated/BackwardAugmentedStateEstimator_v6_JAugWX0.cppready"
					return true;
				}
				case DELTA_POSITION:
				{
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugDispX0.cppready"
					return true;
				}
				case DELTA_ORIENTATION:
				{
#           	include "generated/BackwardAugmentedStateEstimator_v6_JAugDispQX0.cppready"
					return true;
				}
				case ACCELERATION:
				{
					assert(ORDER > 1);
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugAX0.cppready"
					return true;
				}
				case ANGULARACCELERATION:
				{
					assert(ORDER > 1);
#         	  include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaX0.cppready"
					return true;
				}
				case IMUINT_DELTAPOSE:
				{
					assert(ORDER > 1);
#        		  include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPX0.cppready"
					return true;
				}
			}
		}

		switch (y)
		{
			case ACCELERATION:
			{
				assert(ORDER > 1);
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#         		  include "generated/BackwardAugmentedStateEstimator_v6_JAugASO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#         		  include "generated/BackwardAugmentedStateEstimator_v6_JAugASO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#         	    include "generated/BackwardAugmentedStateEstimator_v6_JAugASO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugAqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#           	  include "generated/BackwardAugmentedStateEstimator_v6_JAugAqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#          			include "generated/BackwardAugmentedStateEstimator_v6_JAugAqOS3.cppready"
						return true;
					}
				}

				break;
			}
			case ANGULARACCELERATION:
			{
				assert(ORDER > 1);
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#         		  include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugAlphaqOS3.cppready"
						return true;
					}
				}

				break;
			}
			case IMUINT_DELTAPOSE:
			{
				assert(ORDER > 1);
				switch (x)
				{
					case (2 * ORDER + 1):
					{
#         		  include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPSO1.cppready"
						return true;
					}
					case (2 * ORDER + 2):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPSO2.cppready"
						return true;
					}
					case (2 * ORDER + 3):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPSO3.cppready"
						return true;
					}
					case (2 * ORDER + 4):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPqOS1.cppready"
						return true;
					}
					case (2 * ORDER + 5):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPqOS2.cppready"
						return true;
					}
					case (2 * ORDER + 6):
					{
#           		include "generated/BackwardAugmentedStateEstimator_v6_JAugIMUintdPqOS3.cppready"
						return true;
					}
				}

				break;
			}
		}

		return false;
	}

private:
	//Data needed by the algorithm
	int x;
	int y;
	Eigen::MatrixBase<Derived>& J;

};

}

#endif /* AUGMENTEDSTATEJACOBIANCALCULATOR_H_ */
