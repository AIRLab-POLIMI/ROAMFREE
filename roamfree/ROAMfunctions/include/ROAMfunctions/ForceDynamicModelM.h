#ifndef FORCEDYNAMICMODEL_H_
#define FORCEDYNAMICMODEL_H_

#include <Eigen/Dense>

namespace ROAMfunctions
{
    class ForceDynamicModelM
    {
        public:
            static const bool _usedComponents[];
            static const std::string _paramsNames[];
            static const int _nParams = 2;

            static const unsigned int _ORDER = 2;

            static const unsigned int _ERROR_SIZE = 6;
		    static const unsigned int _NOISE_SIZE = 6;
		    static const unsigned int _MEASUREMENT_SIZE = 6;

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
                Eigen::Map<Eigen::VectorXd> ibd(params[0], 3);
			    Eigen::Map<Eigen::VectorXd> ibod(params[1], 3);

                Eigen::MatrixBase<T> &err = const_cast<Eigen::MatrixBase<T> &>(const_ret);

                const static int _OFF = -1;

                #include "generated/ForceDynamicModel_Err.cppready"
                return false;
            }

            template <typename T>
		    bool errorJacobian(const Eigen::VectorXd &x, double **params,
			    			   const Eigen::VectorXd &z, int wrt,
				    		   Eigen::MatrixBase<T> const &const_ret)
            {
                Eigen::Map<Eigen::VectorXd> ibd(params[0], 3);
			    Eigen::Map<Eigen::VectorXd> ibod(params[1], 3);

                Eigen::MatrixBase<T> &J = const_cast<Eigen::MatrixBase<T> &>(const_ret);

                const static int _OFF = -1;

                switch (wrt)
                {
                    case -6: // jacobian wrt to Alpha
			        {
                        #include "generated/ForceDynamicModel_JErrAlpha.cppready"
				        return false; // "it is useless to evaluate me again"
				        break;
			        }
			        case -5: // jacobian wrt to A
			        {
                        #include "generated/ForceDynamicModel_JErrA.cppready"
				        return false; // "it is useless to evaluate me again"
				        break;
			        }
			        case -4: // jacobian wrt to W
			        {
                        #include "generated/ForceDynamicModel_JErrW.cppready"
				        return true;
				        break;
			        }
                    case -2: // jacobian wrt to Q
			        {
                        #include "generated/ForceDynamicModel_JErrQ.cppready"
				        return true;
				        break;
			        }

			        case 0: // jacobian wrt to noises
			        {
                        #include "generated/ForceDynamicModel_JErrNoises.cppready"
				        return true; // it is not the identity matrix
				        break;
			        }

                    case 1: // jacobian wrt inertia matrix diagonal
			        {
                        #include "generated/ForceDynamicModel_JErrIBD.cppready"
				        return true;
				        break;
			        }
			        case 2: // jacobian wrt inertia off diagonal elements
			        {
                        #include "generated/ForceDynamicModel_JErrIBOD.cppready"
				        return true;
				        break;
			        }
                }

                assert(false);
			    return false;   
            }

    };
} /* namespace ROAMfunctions */
 
#endif /* FORCEDYNAMICMODELM_H_ */