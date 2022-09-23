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
  static const unsigned int _MEASUREMENT_SIZE = 0;

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

    err << 0,0,0;
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
        J = Eigen::MatrixXd::Zero(3,3);
				return false; // "it is useless to evaluate me again"
				break;
			}
			case -5: // jacobian wrt to A
			{
        J = Eigen::MatrixXd::Zero(3,3);
				return false; // "it is useless to evaluate me again"
				break;
			}
			case -4: // jacobian wrt to W
			{
        J = Eigen::MatrixXd::Zero(3,3);
				return false;
				break;
			}
			case -3: // jacobian wrt to V
			{
        J = Eigen::MatrixXd::Zero(3,3);
				return false;
				break;
			}
			case -2: // jacobian wrt to Q
			{
        J = Eigen::MatrixXd::Zero(3,4);
				return false;
				break;
			}
      case -1: // jacobian wrt to Q
			{
        J = Eigen::MatrixXd::Zero(3,3);
				return false;
				break;
			}

			case 0: // jacobian wrt to noises
			{
        J = Eigen::MatrixXd::Identity(3,3);
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
