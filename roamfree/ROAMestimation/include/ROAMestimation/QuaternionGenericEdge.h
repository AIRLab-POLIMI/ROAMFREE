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
 * QuaternionGenericEdge.h
 *
 *  Created on: Mar 20, 2013
 *      Author: davide
 */

#ifndef QUATERNIONGENERICEDGE_H_
#define QUATERNIONGENERICEDGE_H_

#include <sstream>
#include <map>

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>

#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimizable_graph.h"

#include "Enums.h"

#include "ROAMfunctions/AllFunctions.h"
#include "ROAMfunctions/AllVariables.h"

#include "ROAMmath/MatricesOperations.h"

#include "ROAMutils/StringUtils.h"

// old sparse type for _JxArgsXxY temp jacobian matrices
//#include "../ROAMmath/SparsePopulator.h"

#include "GenericVertex.h"
#include "GenericEdge.h"
#include "GenericEdgeInterface.h"
#include "EstimationEdgeCollectInterface.h"
#include "ParameterVerticesManager.h"
#include "ParameterTemporaries.h"

#include "AugmentedStateJacobianCalculator.h"
#include "AugmentedStateCalculator.h"

namespace ROAMestimation {

// TODO: comments on the class hierarchy
//       There should be a class which fix the Eigen::VectorXd template
//       from which this one inherits. In this way a lot of methods can be made independent on the
//       MT template. But it is not sure it is possible since most fields needs the MT::_ERROR_SIZE template
//

template<typename MT>
class QuaternionGenericEdge: public ROAMestimation::GenericEdge<MT::_ERROR_SIZE>,
		public EstimationEdgeCollectInterface {
protected:
	// function object containing implementations
	MT _F;

	// machinery for jacobian computation
	Eigen::MatrixXd *_JErrx[AUGSTATE_N_COMPONENTS];
	bool isNotTheIdentity[AUGSTATE_N_COMPONENTS];

	// TODO: If these matrices are reused, sparse stuff is not a good idea because the sparsity patter changes
	//       so at the end they are full. Find a better solution because I end up multiplying zeros.
	//
	// ROAMmath::SparsePopulator

	Eigen::Matrix<double, 3, 1> _JxArgs3x1;
	Eigen::Matrix<double, 4, 1> _JxArgs4x1;
	Eigen::Matrix<double, 3, 6> _JxArgs3x6;
	Eigen::Matrix<double, 4, 6> _JxArgs4x6;

	// temporary variables to hold covariance and J wrt noises

	Eigen::Matrix<double, MT::_ERROR_SIZE, MT::_ERROR_SIZE> _tmpCov;
	Eigen::Matrix<double, MT::_ERROR_SIZE, MT::_NOISE_SIZE> _tmpJErrNoises;

	// using various stuff...

	USINGDIRECTIVES;

public:

	QuaternionGenericEdge() :
	GenericEdge<MT::_ERROR_SIZE>(_F.getNParams()), _JxArgs3x6(
			3, 6), _JxArgs4x6(4, 6), _JxArgs3x1(3, 1), _JxArgs4x1(4, 1) {

		// resize all(almost) that needs to be resized
		_measurement.resize(MT::_MEASUREMENT_SIZE);
		_noiseCov.resize(MT::_NOISE_SIZE, MT::_NOISE_SIZE);

		// preallocate the components of _JErrx, only the used ones
		// e.g. if the function does not depend on acceleration, we don't need to
		// allocate space for the Jacobian of the error with respect to the acceleration.
		//
		// .. and initialize isNotTheIdentity array
		//
		// components 1,7 are quaternions, size 4, the rest are size 3

		for (int k = 0; k < AUGSTATE_N_COMPONENTS; k++) {
			if (_F._usedComponents[k] == true) {
				_JErrx[k] = new Eigen::MatrixXd((int) MT::_ERROR_SIZE, (k == 1 || k == 7) ? 4 : 3);
			}

			isNotTheIdentity[k] = true;
		}

#ifdef DEBUG_BUILD
		// in debug build we want to have nice logs with zeros in non-used variables
		// e.g. if the error function does not reference the acceleration components of _x
		// these will not be initialized and could contain the famous x.xxxe300 nice doubles..
		_x.setZero();
#endif

	}

	virtual ~QuaternionGenericEdge() {
		// delete the preallocated jacobians matrices
		for (int k = 0; k < AUGSTATE_N_COMPONENTS; k++) {
			if (_F._usedComponents[k] == true) {
				delete _JErrx[k];
			}
		}
	}

	void collect(GenericVertex<ROAMfunctions::SE3V> *xtm2,
			GenericVertex<ROAMfunctions::SE3V> *xtm1,
			GenericVertex<ROAMfunctions::SE3V> *xt, double tstamp,
			GenericVertex<ROAMfunctions::Eucl1DV> *dt1,
			GenericVertex<ROAMfunctions::Eucl1DV> *dt2, const std::string &name,
			std::map<std::string, boost::shared_ptr<ParameterVerticesManager> > &params) {

		// --- set name and timestamp
		_name = name;
		_tstamp = tstamp;

		// required vertices: x(t-1), x(t), x(t+1), dt1, dt2, s(O)[0], s(O)[1], s(O)[2], qOS[0], qOS[1], qOS[2]
		// here the displacement and misalignment parameters are divided in their components so that
		// we can separately fix & unfix each one depending on the situations (e.g.. 2d roaming)

		// each sensor has the default displacement and misalignment vertices
		// and eventually other parameters
		// we require that there exist 6 Eucl1D vertices in the map (maybe shared) with names
		// <sensor_name>_SO<n> and <sensor_name>_qOS<n>

		// populate the parameter name vector (C++11)
		std::vector<std::string> to_serach = {"SOx", "SOy", "SOz", "qOSx", "qOSy",
			"qOSz"};

		const std::string *argNames = _F.getParamsList();

		for (int k = 0; k < _F.getNParams(); k++) {
			to_serach.push_back(argNames[k]);
		}

		// --- collect the parameter vertices managers

		for (int k = 0; k < to_serach.size(); k++) {
			std::stringstream s;
			s << _name << "_" << to_serach[k];

			std::map<std::string, boost::shared_ptr<ParameterVerticesManager> >::const_iterator it =
			params.find(s.str());

			if (it != params.end()) {
				// allocate the parameter descriptor
				_params.emplace_back(it->second, _tstamp);// N.B. C++11

			} else {
				std::cerr << "[Sensor " << _name << "] Error: Parameter vertex '"
				<< s.str() << "' not found in graph" << std::endl;
			}
		}

		// --- now count how many parameter vertices we have
		int vertexCount = 0;
		for (std::vector<ParameterTemporaries>::const_iterator it = _params.begin();
				it != _params.end(); ++it) {
			vertexCount += it->p->getWindowSize();
		}

		// resize this edge so that it can hold the parameter vertices
		resize(2 * MT::_ORDER + 1 + vertexCount);

		// --- add the parameter vertices at the proper positions in the _vertices vector

		if (MT::_ORDER == 2) {
			_vertices[3] = dt1;
			_vertices[4] = xtm2;
		}
		if (MT::_ORDER > 0) {
			_vertices[1] = dt2;
			_vertices[2] = xtm1;
		}

		_vertices[0] = xt;

		int freeVertexPosition = 2 * MT::_ORDER + 1;

		for (std::vector<ParameterTemporaries>::const_iterator it = _params.begin();
				it != _params.end(); ++it) {

		  //
		  it->p->getVerticesPointers(_tstamp, _vertices, freeVertexPosition);
		  freeVertexPosition += it->p->getWindowSize();
		  //*/

		  /* old code with getVertices
			std::map<double, g2o::OptimizableGraph::Vertex *>::const_iterator vit =
			it->p->getVertices(_tstamp);

			for (int k = 0; k < it->p->getWindowSize(); k++) {
				g2o::OptimizableGraph::Vertex * ov = vit->second;
				assert(ov != NULL); // debug check for consistency in the getVertices window.

				_vertices[freeVertexPosition] = ov;// set the vertex

				freeVertexPosition++;
				vit++;
			}
			//*/
		}

#		ifdef DEBUG_BUILD
		// check that I have no duplicates in the vertex set
		for (int i = 0; i < freeVertexPosition-1; i++) {
			for (int j = i+1; j< freeVertexPosition; j++) {
				assert( _vertices[i]!= _vertices[j]);
			}
		}
#		endif

		// --- resize the _jacobianOplus matrices (only for the vertices which are not fixed)
		for (int k = 0; k < _vertices.size(); k++) {
			g2o::OptimizableGraph::Vertex *ov =
			static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[k]);

			//TODO: this memory optimization had to be removed otherwise I cannot easily unfix a vertex during optimization

			//if (!ov->fixed()) {
			_jacobianOplus[k].resize(MT::_ERROR_SIZE, ov->dimension());
			//}
		}
	}

	/**
	 * \brief computes all the stuff that has to be updated only once  per iteration
	 */

	//TODO: find a way to call this method only once per vertex change
	void afterVertexUpdate() {

		// std::cerr << "[QuaternionGenericEdge] Info: afterVertexUpdate()" << std::endl;

		handleParamTemporaries();

		AugmentedStateCalculator augmentedState(_params, _x, _F._usedComponents);
		switch(MT::_ORDER)
		{
			case 0:
			{
				const Eigen::VectorXd &x2 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
				augmentedState.calculate(x2);
				break;
			}
			case 1:
			{
				const Eigen::VectorXd &x2 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
				const Eigen::VectorXd &x1 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[2])->estimate();
				const double &Dt12 = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate()(0);
				augmentedState.calculate(x1, x2, Dt12);
				break;
			}
			case 2:
			{
				const Eigen::VectorXd &x2 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
				const Eigen::VectorXd &x1 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[2])->estimate();
				const Eigen::VectorXd &x0 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[4])->estimate();
				const double &Dt12 = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate()(0);
				const double &Dt01 = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[3])->estimate()(0);
				augmentedState.calculate(x0, x1, x2, Dt01, Dt12);
				break;
			}
		}

		// --- update the edge information matrix
		// compute the jacobian of the error with respect to the noises

		// if the method returns false the jacobian wrt noises is the identity matrix
		// so there is no need to perform multiplications and inversions
		if (_F.errorJacobian(_x, _paramsPtrs, _measurement, 0, _tmpJErrNoises)) {
			_tmpCov.noalias() = _tmpJErrNoises * _noiseCov * _tmpJErrNoises.transpose();
			ROAMmath::inv(_tmpCov, _information);
		}
	}

	void computeError() {

#   ifdef DEBUG_COMPARE_WITH_NUMERIC_JACOBIANS
		// this is needed in the g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::linearizeOplus()
		afterVertexUpdate();
#   endif

		_F.error(_x, _paramsPtrs, _measurement, _error);

		/* debug
		 std::cerr << "---- " << writeDebugInfo() << std::endl;
		 std::cerr << static_cast<GenericVertex<SE3V> *>(_vertices[0])->estimate().transpose() << std::endl;
		 if (MT::_ORDER > 0) {
		 std::cerr << static_cast<GenericVertex<SE3V> *>(_vertices[2])->estimate().transpose() << std::endl;
		 }
		 std::cerr << _x.transpose() << std::endl;

		 std::cerr << _error.transpose() << std::endl;
		 std::cerr << _measurement.transpose() << std::endl;
		 std::cerr << "chi2" << chi2() << std::endl;
		 //*/
	}

	void linearizeOplus() {
		/* debug
		 std::cerr << " ------------------- " << getCategory() << std::endl;
		 //*/

		// --- update the relevant components of J_err wrt augmented state
		for (int k = 0; k < AUGSTATE_N_COMPONENTS; k++) {
			if (_F._usedComponents[k] && isNotTheIdentity[k]) {

				// if it returns false, it means "it is useless to evaluate me again"
				isNotTheIdentity[k] = _F.errorJacobian(_x, _paramsPtrs, _measurement,
						-(k + 1), *_JErrx[k]);

				/* debug
				 std::cerr << "J_err wrt augx_i, i=" << k << std::endl << *_JErrx[k]
				 << std::endl;
				 //*/
			}
		}

		// now we have to evaluate sum_i( J_err/augState_i * J_augState_i/vertex_j ) for each j not fixed
		// which contributes to the augmented state computation

		// for vertices 0-(2*MT::_ORDER+1 + 6) (the ones from which we compute the augmented state) compute their jacobian
		// remember that we assume that S(O) and qOS are time-invariant parameters

		for (int v = 0; v < 2 * MT::_ORDER + 1 + 6; v++) {

			g2o::OptimizableGraph::Vertex *ov =
			static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[v]);

			// but only if they are not fixed, otherwise who cares..
			if (!ov->fixed()) {

				_jacobianOplus[v].setZero(); // very important since here we compute a summation

				for (int k = 0; k < AUGSTATE_N_COMPONENTS; k++) { //k is the current component of the augmented state
					if (_F._usedComponents[k]) { // if this is false, the product is zero so it is useless to sum

						/* debug
						 std::cerr << "computing J_err wrt augx_i, i=" << k
						 << "* J_augx_i wrt x_j, j=" << v << std::endl;
						 //*/

						// here we have to map the current value of v to the correct value the computeJacobianBlock expects
						switch (ov->dimension()) { // this is for the dimension of temporaries
							case 1:
							// cols = 1
							if (k == 1 || k == 7) { // with respect to some quaternion, rows = 4
								computeJacobianBlock(k, v, _JxArgs4x1);
								_jacobianOplus[v].noalias() += (*_JErrx[k]) * _JxArgs4x1;
							} else {
								// rows = 3
								computeJacobianBlock(k, v, _JxArgs3x1);
								_jacobianOplus[v].noalias() += (*_JErrx[k]) * _JxArgs3x1;
							}
							break;

							case 6:
							// cols = 6
							if (k == 1 || k == 7) { // with respect to q or delta q, rows = 4
								computeJacobianBlock(k, v, _JxArgs4x6);
								_jacobianOplus[v].noalias() += (*_JErrx[k]) * _JxArgs4x6;
							} else {
								// rows = 3
								computeJacobianBlock(k, v, _JxArgs3x6);
								_jacobianOplus[v].noalias() += (*_JErrx[k]) * _JxArgs3x6;
							}
							break;

							default:
							assert(false);

						}
					}
				}
			}

			// some debug
			//std::cout << v << std::endl << _jacobianOplus[v] << std::endl;
		}

		// here we come to the non default parameters of the function

		int cur_vertex = 2 * MT::_ORDER + 1 + 6;

		for (int k = 6; k < _params.size(); k++) {
			for (int h = 0; h < _params[k].p->getWindowSize(); h++, cur_vertex++) {

				// if the vertex is fixed avoid wasting time

				if (!static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[cur_vertex])->fixed()) {

					// sixth descriptor in _params is the first non default parameter (indexed with 1 in errorJacobian)
					// TODO: if the jacobian is the identity matrix there is no need to update this stuff every time
					_F.errorJacobian(_x, _paramsPtrs, _measurement, k - 5,
							_jacobianOplus[cur_vertex]);

					// there is the nice property that post-multiplying to J(err) wrt parameter k
					// with J(k) wrt vertex k^i does not change the dimension of the matrix _jacobianOplus[k]
					// thus I don't need a temporary.

					Eigen::MatrixXd & pJ = _params[k].jacobians[h];

					// apparently Eigen cannot handle this case by themselves
					if (pJ.rows() == 1 && pJ.cols() == 1) {
						_jacobianOplus[cur_vertex] *= pJ(0, 0);
					} else {
						_jacobianOplus[cur_vertex] *= _params[k].jacobians[h];
					}

					/*
					 std::cout << "J " << k << " w.r.t v " << h << ":" << std::endl;
					 std::cout << _params[k].jacobians[h] << std::endl;
					 //*/
				}
			}
		}

		/* debug
		 std::cerr << "---- " << writeDebugInfo() << std::endl;
		 for (int k = 0; k < _vertices.size(); k++) {
		 if (!static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[k])->fixed()) {
		 std::cout << "analytic jacobian wrt " << k << "th vertex" << std::endl;
		 std::cout << _jacobianOplus[k] << std::endl;
		 }
		 }

		 //*/

#   ifdef DEBUG_COMPARE_WITH_NUMERIC_JACOBIANS
		// debug, save analytic jacobians to compute differences

		std::vector<Eigen::MatrixXd> tmp_J;
		for (int k = 0; k < _jacobianOplus.size(); k++) {
			tmp_J.emplace_back(_jacobianOplus[k].rows(), _jacobianOplus[k].cols());
			tmp_J.back() = _jacobianOplus[k];
		}

		// compute numeric jacobians
		g2o::BaseMultiEdge<MT::_ERROR_SIZE, Eigen::VectorXd>::linearizeOplus();

		// compare with the analytic ones and report differences
		for (unsigned int i = 0; i < _vertices.size(); i++) {
			if (!static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[i])->fixed()) {
				Eigen::MatrixXd diff = (tmp_J[i] - _jacobianOplus[i]).cwiseAbs();

				if (diff.maxCoeff() > 1) {
					std::cerr << "[QuaternionGenericEdge] Edge: " << writeDebugInfo() << std::endl;
					std::cerr << "Error: substantial difference between " << i << "th argument analytic and numeric jacobians." << std::endl;

					std::cerr << _name << ": J analytic - J numeric (max: " << diff.maxCoeff() << ")" << std::endl;
					std::cerr << diff << std::endl;

					std::cerr << " J numeric: " << std::endl;
					std::cerr << _jacobianOplus[i] << std::endl;

					std::cerr << " J analytic: " << std::endl;
					std::cerr << tmp_J[i] << std::endl;

					//assert(false);
				}
			}
		}

#   endif

		/* debug jacobians
		 for (unsigned int i = 0; i < _vertices.size(); i++) {
		 if (!static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[i])->fixed()) {
		 std::cout << _name << ": Jacobian with respect to " << i
		 << "th argument:" << std::endl;
		 std::cout << _jacobianOplus[i] << std::endl;
		 }
		 }
		 //*/
	}

	void predictNextState() {

		handleParamTemporaries();

		// call the predict method

		const Eigen::VectorXd &x1 =
		static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[2])->estimate();
		Eigen::VectorXd &x2 =
		static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();

		const double &Dt12 =
		static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate()(
				0);

		assert(_F.predict(x1, _paramsPtrs, _measurement, Dt12, x2));
	}

	inline
	int getOrder() const {
		return MT::_ORDER;
	}

	const bool *getUsedComponents() const {
		return _F._usedComponents;
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

	template<typename Derived>
	void computeJacobianBlock(int y, int x, Eigen::MatrixBase<Derived> &J) {

		bool calculated = false;
		AugmentedStateJacobianCalculator<Derived, MT::_ORDER> jacobianBlock(_params, y, x, J);
		switch(MT::_ORDER)
		{
			case 0:
			{
				const Eigen::VectorXd &x2 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
				calculated = jacobianBlock.calculate(x2);
				break;
			}
			case 1:
			{
				const Eigen::VectorXd &x2 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
				const Eigen::VectorXd &x1 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[2])->estimate();
				const double &Dt12 = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate()(0);
				calculated = jacobianBlock.calculate(x1, x2, Dt12);
				break;
			}
			case 2:
			{
				const Eigen::VectorXd &x2 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[0])->estimate();
				const Eigen::VectorXd &x1 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[2])->estimate();
				const Eigen::VectorXd &x0 = static_cast<GenericVertex<ROAMfunctions::SE3V> *>(_vertices[4])->estimate();
				const double &Dt12 = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[1])->estimate()(0);
				const double &Dt01 = static_cast<GenericVertex<ROAMfunctions::Eucl1DV> *>(_vertices[3])->estimate()(0);
				calculated = jacobianBlock.calculate(x0, x1, x2, Dt01, Dt12);
				break;
			}
		}

		if(calculated == false)
		{
			// something really wrong has happened
			std::cerr << "[Sensor " << _name << "] Error: trying to compute the " << y
			<< "," << x << "block of the Jacobian matrix" << std::endl;
			assert(false);
		}

	}

	std::string writeDebugInfo() const {
		std::stringstream s;

		g2o::OptimizableGraph::Vertex *x2, *x1, *x0;

		x0 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[0]);

		s << _name << "{" << _frameCounter << "}[" << ROAMutils::StringUtils::writeNiceTimestamp(_tstamp) << "](" << x0->id();

		if (MT::_ORDER > 0) {
			x1 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[2]);
			s << "," << x1->id();
		}

		if (MT::_ORDER > 1) {
			x2 = static_cast<g2o::OptimizableGraph::Vertex *>(_vertices[4]);
			s << "," << x2->id();
		}

		s << ")";

		return s.str();
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
;

}
/* namespace ROAMfunctions */
#endif /* QUATERNIONGENERICEDGE_H_ */

