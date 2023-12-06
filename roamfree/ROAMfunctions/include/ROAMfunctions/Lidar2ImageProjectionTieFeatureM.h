#ifndef LIDAR2IMAGEPROJECTIONTIEFEATUREM_H
#define LIDAR2IMAGEPROJECTIONTIEFEATUREM_H

#include <Eigen/Dense>

namespace ROAMfunctions {

class LiDAR2ImageProjectionTieFeatureM {

public:

    static const bool _usedComponents[];

    static const std::string _paramsNames[];
    static const int _nParams = 8;

    static const unsigned int _ORDER = 1;

    static const unsigned int _ERROR_SIZE = 2;
    static const unsigned int _NOISE_SIZE = 5;
    static const unsigned int _MEASUREMENT_SIZE = 5;

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

        Eigen::Map<Eigen::Vector4d> lqOS(params[0]);
        Eigen::Map<Eigen::Vector3d> lSO(params[1]);
        Eigen::Map<Eigen::Vector4d> cqOS(params[2]);
        Eigen::Map<Eigen::Vector3d> cSO(params[3]);

        Eigen::Map<Eigen::Vector3d> cm(params[4]);
        Eigen::Map<Eigen::Vector4d> rd(params[5]);
        Eigen::Map<Eigen::Vector2d> td(params[6]);
        Eigen::Map<Eigen::Vector2d> sk(params[7]);

        Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

        const static int _OFF = -1;

    #   include "generated/LiDAR2ImageTieFeature_Err.cppready"

      return false;
    }

    template<typename T>
    bool errorJacobian(const Eigen::VectorXd &x, double **params,
        const Eigen::VectorXd& z, int wrt,
        Eigen::MatrixBase<T> const &const_ret) {

        Eigen::Map<Eigen::Vector4d> lqOS(params[0]);
        Eigen::Map<Eigen::Vector3d> lSO(params[1]);
        Eigen::Map<Eigen::Vector4d> cqOS(params[2]);
        Eigen::Map<Eigen::Vector3d> cSO(params[3]);

        Eigen::Map<Eigen::Vector3d> cm(params[4]);
        Eigen::Map<Eigen::Vector4d> rd(params[5]);
        Eigen::Map<Eigen::Vector2d> td(params[6]);
        Eigen::Map<Eigen::Vector2d> sk(params[7]);

      Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);

      const static int _OFF = -1;

      switch (wrt) {
      case -10: // jacobian wrt DispQ
      {
  #     include "generated/LiDAR2ImageTieFeature_JErrQprev.cppready"
        return true;
        break;
      }
      case -7: // jacobian wrt Disp
      {
  #     include "generated/LiDAR2ImageTieFeature_JErrDisp.cppready"
        return true;
        break;
      }
      case -2: // jacobian wrt Q
      {
  #     include "generated/LiDAR2ImageTieFeature_JErrQ.cppready"
        return true;
        break;
      }
      case 0: // jacobian wrt noises
      {
        // it is the identity matrix
  //#     include "generated/LiDAR2ImageTieFeature_JErrNoises.cppready"
        return false;
        break;
      }

      case 1: // jacobian wrt lqOS
      {
#			include "generated/LiDAR2ImageTieFeature_JErrQL2B.cppready"
        return true;
        break;
      }
      case 2: // jacobian wrt lSO
      {
#     include "generated/LiDAR2ImageTieFeature_JErrtL2B.cppready"
        return true;
        break;
      }
      case 3: // jacobian wrt cqOS
      {
#     include "generated/LiDAR2ImageTieFeature_JErrQC2B.cppready"
        return true;
        break;
      }
      case 4: // jacobian wrt cSO
      {
#     include "generated/LiDAR2ImageTieFeature_JErrtC2B.cppready"
        return true;
        break;
      }

      case 5: // jacobian wrt camera matrix
      {
#			include "generated/LiDAR2ImageTieFeature_JErrCM.cppready"
        return true;
        break;
      }
      case 6: // jacobian wrt radial distortion
      {
#     include "generated/LiDAR2ImageTieFeature_JErrRD.cppready"
        return true;
        break;
      }
      case 7: // jacobian wrt tangential distortion
      {
#     include "generated/LiDAR2ImageTieFeature_JErrTD.cppready"
        return true;
        break;
      }
      case 8: // jacobian wrt skew
      {
#     include "generated/LiDAR2ImageTieFeature_JErrSKEW.cppready"
        return true;
        break;
      }
      }

      assert(false);
      return false;
    }



};

} // namespace ROAMfunctions

#endif // LIDAR2IMAGEPROJECTIONTIEFEATUREM_H
