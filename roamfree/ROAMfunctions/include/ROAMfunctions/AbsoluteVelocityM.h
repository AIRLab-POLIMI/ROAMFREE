
#ifndef ABSOLUTEVELOCITYM_H_
#define ABSOLUTEVELOCITYM_H_

#include <string>
#include <Eigen/Dense>
#include <iostream>
namespace ROAMfunctions {

class AbsoluteVelocityM {

public:
    static const bool _usedComponents[];
    static const std::string _paramsNames[];
    static const int _nParams = 0;

    static const unsigned int _ORDER = 1;
    static const unsigned int _ERROR_SIZE = 3;
    static const unsigned int _NOISE_SIZE = 3;
    static const unsigned int _MEASUREMENT_SIZE = 3;

    const std::string* getParamsList() {
    return _paramsNames;
    }

    const int getNParams() {
    return _nParams;
    }

    bool predict(const Eigen::VectorXd &x, double **params, const Eigen::VectorXd& z, double dt, Eigen::VectorXd &xhat) {
        const static int _OFF = -1;

        #include "generated/AbsoluteVelocity_predictor.cppready"
        return true;
    }

    template<typename T>
    bool error(const Eigen::VectorXd &x, double **params, const Eigen::VectorXd& z, Eigen::MatrixBase<T> const &const_ret) {

        Eigen::MatrixBase<T> & err = const_cast<Eigen::MatrixBase<T>&>(const_ret);

        const static int _OFF = -1;

        #include "generated/AbsoluteVelocity_Err.cppready"

        // std::cerr<<"x: "<<x<<"\n"<<std::endl;
        // std::cerr<<"z: "<<z<<"\n"<<std::endl;
        // std::cerr<<"err: "<<err<<"\n"<<std::endl;
        // std::cerr<<"\n\n"<<std::endl;
        return false;
    }

    template<typename T>
    bool errorJacobian(const Eigen::VectorXd &x, double **params, const Eigen::VectorXd& z, int wrt, Eigen::MatrixBase<T> const &const_ret) {

        
        Eigen::MatrixBase<T> & J = const_cast<Eigen::MatrixBase<T>&>(const_ret);
        const static int _OFF = -1;

        switch (wrt)
        {

        case -12: {
            #include "generated/AbsoluteVelocity_JErrWprev.cppready"
            return true;
            break;
        }     
            
        case -11: {
            #include "generated/AbsoluteVelocity_JErrVprev.cppready"
            return true;
            break;
        }
            
        case -10: {
            #include "generated/AbsoluteVelocity_JErrQprev.cppready"
            return true;
            break;
        }
            
        case -6: {
            #include "generated/AbsoluteVelocity_JErrAlpha.cppready"
            return true;
            break;
        }
            
        case -5: {
            #include "generated/AbsoluteVelocity_JErrA.cppready"
            return true;
            break;
        }
            
        case -4: {
            #include "generated/AbsoluteVelocity_JErrW.cppready"
            return true;
            break;
        }
            
        case -3: {
            #include "generated/AbsoluteVelocity_JErrV.cppready"
            return true;
            break;
        }
            
        case -2: {
            #include "generated/AbsoluteVelocity_JErrQ.cppready"
            return true;
            break;
        }
            
        case 0: {
            #include "generated/AbsoluteVelocity_JErrNoises.cppready"
            return true;
            break; 
        }
              
        }

        assert(false);
        return false;
    }

};
} /* namespace ROAMfunctions */

#endif /* ABSOLUTEVELOCITYM_H_ */