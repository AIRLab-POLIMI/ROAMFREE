#ifndef POSEDERIVATIVEM_H_
#define POSEDERIVATIVEM_H_

#include <Eigen/Dense>
#include <iostream>
namespace ROAMfunctions {

class PoseDerivativeM {

public:
  static const bool _usedComponents[];

  static const std::string _paramsNames[];
  static const int _nParams = 0;

  static const unsigned int _ORDER = 2;

  static const unsigned int _ERROR_SIZE = 3;
  static const unsigned int _NOISE_SIZE = 3;
  static const unsigned int _MEASUREMENT_SIZE = 3;

  const std::string* getParamsList() {
    return _paramsNames;
  }
  const int getNParams() {
    return _nParams;
  }

  bool predict(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {
    xhat = x; // TODO: dummy predictor

    return true;

  }

  template<typename T>
  bool error(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

#   include "generated/PoseDerivative_Err.cppready"

    return false;
  }

  template<typename T>
  bool errorJacobian(const Eigen::VectorXd &x, double **params,
      const Eigen::VectorXd& z, int wrt,
      Eigen::MatrixBase<T> const &const_ret) {

    Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

    const static int _OFF = -1;

    switch (wrt) {
      
    case -6: // jacobian wrt to Alpha
			{
#include "generated/PoseDerivative_JErrAlpha.cppready"
				return false; // "it is useless to evaluate me again"
				break;
			}
			case -5: // jacobian wrt to A
			{
#include "generated/PoseDerivative_JErrA.cppready"
				return false; // "it is useless to evaluate me again"
				break;
			}
			case -4: // jacobian wrt to W
			{
#include "generated/PoseDerivative_JErrW.cppready"
				return false;
				break;
			}
			case -3: // jacobian wrt to V
			{
#include "generated/PoseDerivative_JErrV.cppready"
				return false;
				break;
			}
			case -2: // jacobian wrt to Q
			{
#include "generated/PoseDerivative_JErrQ.cppready"
				return false;
				break;
			}
      case -1: // jacobian wrt to Q
			{
#include "generated/PoseDerivative_JErrP.cppready"
				return false;
				break;
			}

			case 0: // jacobian wrt to noises
			{
#include "generated/PoseDerivative_JErrNoises.cppready"
				return false; // it is not the identity matrix
				break;
			}
//    case 0: // jacobian wrt noises
//    {
      // it is the identity matrix
//#     include "generated/LiDARTieFeatures_JErrNoises.cppready"
//      return false;
 //     break;
 //   }
    }
  std::cout << "Assertion "<< wrt<< std::endl;
    assert(false);
    return false;
  }
};
} /* namespace ROAMfunctions */
#endif /* POSEDERIVATIVEM_H_ */
