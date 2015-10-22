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
 * FactorGraphFilterImpl.cpp
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#include "FactorGraphFilterImpl.h"

#include <fenv.h>
#include <time.h>

#include <utility>

#include "PriorEdges/AllPriors.h"
#include "ROAMfunctions/AllVariables.h"
#include "ROAMfunctions/AllFunctions.h"

#include "ROAMutils/StringUtils.h"

#include "ROAMlog/GraphLogger.h"

#include "QuaternionGenericEdge.h"
#include "GenericEdgeInterface.h"
#include "EstimationEdgeCollectInterface.h"
#include "g2oSolverFactory.h"
#include "ConstantParameter.h"
#include "LimitedBandwithEuclideanParameter.h"
#include "LinearlyInterpolatedEuclideanParameter.h"
#include "ParameterBlender.h"
#include "PoseVertexMetadata.h"
#include "MeasurementEdgeMetadata.h"
#include "PoseVertexWrapperImpl.h"
#include "ParameterWrapperImpl.h"
#include "MeasurementEdgeWrapperImpl.h"
#include "GenericLinearConstraintFactory.h"

#include "SpatialIndex/SpatialIndex.h"

/*
 #ifndef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
 #define DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES 1
 #endif
 */

using namespace std;

namespace ROAMestimation {

FactorGraphFilter_Impl::FactorGraphFilter_Impl() {
  initSolver();
}

FactorGraphFilter_Impl::~FactorGraphFilter_Impl() {
  if (_logger != NULL) {
    delete _logger;
  }

  delete _spatialIndex;

  delete _optimizer;
}

void FactorGraphFilter_Impl::initSolver() {

  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  _optimizer = g2oSolverFactory::getNewSolver();
  _optimizer->setVerbose(DEBUG_G2O_OPTIMIZER_VERBOSE);

  // default solver is GaussNewton
  setSolverMethod(GaussNewton);

  // the initial pose has not been set
  _initialPoseSet = false;

  // by default, we go for dead reckoning
  _deadReckoning = true;

  // no logging by default
  _lowLevelLogging = false;
  _logger = NULL;

  // init the spatial index

  _spatialIndex = new SpatialIndex;
}

void FactorGraphFilter_Impl::setSolverMethod(SolverMethod method) {
  //decouple from g2o Enum for method
  switch (method) {
  case GaussNewton:
    _optimizer->setMethod(g2o::SparseOptimizer::GaussNewton);
    break;

  case LevenbergMarquardt:
    _optimizer->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
    _optimizer->setUserLambdaInit(0.);
    break;
  }
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::setInitialPose(
    const Eigen::VectorXd &x0, double t) {

  PoseVertex *v = addPose_i(t);

  v->setEstimate(x0);

  _initialPoseSet = true;

  for (map<string, struct Sensor>::iterator it = _sensors.begin();
      it != _sensors.end(); ++it) {

    // only for order 1 and 2, otherwise a measurement for the first pose will be discarded
    if (it->second.order != 0) {
      it->second.last = v;
    }
  }

  return PoseVertexWrapper_Ptr(new PoseVertexWrapper_Impl(v));
}

bool FactorGraphFilter_Impl::addSensor(const string& name, MeasTypes type,
    bool isMaster, bool isSequential) {
// retrieve the sensor descriptor
  map<string, struct Sensor>::iterator it = _sensors.find(name);

  if (it != _sensors.end()) {
    cerr << "[FactorGraphFilter] Error: Sensor '" << name
        << "' already present." << endl;
    return false;
  }

// retrieve the sensor order according to its type

  struct Sensor s;

  switch (type) {
  case AbsolutePosition:
    s.order = AbsolutePositionM::_ORDER;
    break;
  case AbsolutePose:
    s.order = AbsolutePoseM::_ORDER;
    break;
  case LinearVelocity:
    s.order = LinearVelocityM::_ORDER;
    break;
  case AngularVelocity:
    s.order = AngularVelocityM::_ORDER;
    break;
  case LinearAcceleration:
    s.order = AccelerationM::_ORDER;
    break;
  case AckermannOdometer:
    s.order = AckermannM::_ORDER;
    break;
  case AckermannConstraint:
    s.order = AckermannKinematicNoInputsM::_ORDER;
    break;
  case TriskarOdometer:
    s.order = TriskarKinematicM::_ORDER;
    break;
  case DifferentialDriveOdometer:
    s.order = DifferentialDriveKinematicM::_ORDER;
    break;
  case GenericOdometer:
    s.order = GenericOdometerM::_ORDER;
    break;
  case Displacement:
    s.order = DisplacementM::_ORDER;
    break;
  case VectorField:
    s.order = VectorFieldM::_ORDER;
    break;
  case VectorFieldAsCompass:
    s.order = VectorFieldAsCompassM::_ORDER;
    break;
  case IMUintegralDeltaP:
    s.order = IMUImtegralDeltaPM::_ORDER;
    break;
  case IMUintegralDeltaQ:
    s.order = IMUImtegralDeltaQM::_ORDER;
    break;
  case FixedFeaturePosition:
    s.order = FixedFeaturePositionM::_ORDER;
    break;
  case FixedFeaturePose:
    s.order = FixedFeaturePositionM::_ORDER;
    break;
  case ImagePlaneProjection:
    s.order = ImagePlaneProjectionM::_ORDER;
    break;
  case FramedHomogeneousPoint:
    s.order = FramedHomogeneousPointM::_ORDER;
    break;
  case RectangularObject:
    s.order = RectangularObjectM::_ORDER;
    break;
  case AnchoredRectangularObject:
    s.order = AnchoredRectangularObjectM::_ORDER;
    break;
  case AnchoredRectangularObjectFirst:
    s.order = AnchoredRectangularObjectM::_ORDER;
    break;
  case PlanarConstraint:
    s.order = PlanarConstraintM::_ORDER;
    break;
  default:
    cerr << "[FactorGraphFilter] Error: unknown measurement type" << endl;
    break;
    return false;
  }

  s.name = name;

  s.type = type;
  s.isMaster = isMaster;
  s.isSequential = isSequential;

  s.last = NULL;
  s.secondlast = NULL;

  _sensors[name] = s;

// default sensors are not robustified
  setRobustKernel(name, false, 0);

  return true;
}

bool FactorGraphFilter_Impl::setSensorFrame(const string& sensor,
    const Eigen::VectorXd& s) {
// retrieve the sensor descriptor
  map<string, struct Sensor>::iterator sensor_it = _sensors.find(sensor);

  if (sensor_it == _sensors.end()) {
    cerr << "[FactorGraphFilter] Error: Sensor '" << sensor << "' undefined."
        << endl;
    return false;
  }

  const string suffixes[] =
      { "_SOx", "_SOy", "_SOz", "_qOSx", "_qOSy", "_qOSz" };

  for (int k = 0; k < 6; k++) {
    addConstantParameter(sensor + suffixes[k], s(k > 2 ? k + 1 : k), true);
  }

  return true;
}

bool FactorGraphFilter_Impl::shareSensorFrame(const string& from,
    const string &to) {

// retrieve the sensor descriptor
  map<string, struct Sensor>::iterator sensor_it = _sensors.find(from);

  if (sensor_it == _sensors.end()) {
    cerr << "[FactorGraphFilter] Error: Sensor '" << from << "' undefined."
        << endl;
    return false;
  }

  const string suffixes[] =
      { "_SOx", "_SOy", "_SOz", "_qOSx", "_qOSy", "_qOSz" };

  for (int k = 0; k < 6; k++) {
    auto toshare = _params.find(from + suffixes[k]);

    if (toshare == _params.end()) {
      cerr << "[FactorGraphFilter] Error: Parameter'" << (from + suffixes[k])
          << "' undefined." << endl;
      return false;
    }

    _params[to + suffixes[k]] = toshare->second;
  }

  return true;
}

bool FactorGraphFilter_Impl::shareParameter(const string &from,
    const string &to) {
  auto toshare = _params.find(from);

  if (toshare == _params.end()) {
    cerr << "[FactorGraphFilter] Error: Parameter'" << from << "' undefined."
        << endl;
    return false;
  }

  _params[to] = toshare->second;

  return true;
}

bool FactorGraphFilter_Impl::setRobustKernel(const string& sensor, bool enabled,
    double huberWidth) {

// retrieve the sensor descriptor
  map<string, struct Sensor>::iterator sensor_it = _sensors.find(sensor);

  if (sensor_it == _sensors.end()) {
    cerr << "[FactorGraphFilter] Error: Sensor '" << sensor << "' undefined."
        << endl;
    return false;
  }

  sensor_it->second.robustified = enabled;
  sensor_it->second.huberWidth = huberWidth;

  return true;
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addConstantParameter(
    ParameterTypes type, const string& name, double t,
    const Eigen::VectorXd& x0, bool isFixed) {
  _params[name] = boost::shared_ptr<ParameterVerticesManager>(
      new ConstantParameter(_optimizer, type, name, t, x0));
  _params[name]->setFixed(isFixed);
  return ParameterWrapper_Ptr(new ParameterWrapper_Impl(_params[name].get()));
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addConstantParameter(
    ParameterTypes type, const string& name, const Eigen::VectorXd& x0,
    bool isFixed) {
  return addConstantParameter(type, name, 0.0, x0, isFixed);
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addConstantParameter(
    const string &name, double x0, bool isFixed) {
  Eigen::VectorXd eigen_x0(1);
  eigen_x0 << x0;
  return addConstantParameter(Euclidean1D, name, eigen_x0, isFixed);
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::poseVertexAsParameter(
    PoseVertexWrapper_Ptr pose, const string& name) {

  if (!pose) {
    cerr << "[FactorGraphFilter] Error: Null PoseVertexWrapper_Ptr" << endl;
    return ParameterWrapper_Ptr();
  }

  PoseVertex *pv = boost::static_pointer_cast<PoseVertexWrapper_Impl>(pose)->_v;

  PoseMapIterator pv_it = _poses.find(pv->getTimestamp());
  if (pv_it == _poses.end() || pv_it->second != pv) {
    cerr
        << "[FactorGraphFilter] Error: referenced pose vertex does not exist in the active pose window"
        << endl;
    return ParameterWrapper_Ptr();
  }

//pv->setCategory(name);

  _params[name] = boost::shared_ptr<ParameterVerticesManager>(
      new ConstantParameter(_optimizer, SE3, name, pv));

  return ParameterWrapper_Ptr(new ParameterWrapper_Impl(_params[name].get()));
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addLimitedBandwithParameter(
    ParameterTypes type, const string& name, const Eigen::VectorXd& x0,
    bool isFixed, double bandwith, int a) {

  if (type != Euclidean3D && type != Euclidean1D && type != Euclidean2D) {
    cerr
        << "[FactorGraphFilter] Error: currently, only EuclideanXD limited bandwidth parameters are supported"
        << endl;
    return ParameterWrapper_Ptr();
  }

  _params[name] = boost::shared_ptr<ParameterVerticesManager>(
      new LimitedBandwithEuclideanParameter(bandwith, a, _optimizer, type, name,
          x0));
  _params[name]->setFixed(isFixed);

  return ParameterWrapper_Ptr(new ParameterWrapper_Impl(_params[name].get()));
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addLimitedBandwithParameter(
    const std::string& name, double x0, bool isFixed, double bandwidth, int a) {
  Eigen::VectorXd tmp(1);
  tmp << x0;

  return addLimitedBandwithParameter(Euclidean1D, name, tmp, isFixed, bandwidth,
      a);
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addLinearlyInterpolatedParameter(
    ParameterTypes type, const std::string& name, const Eigen::VectorXd& x0,
    bool isFixed, double spacing) {

  if (type != Euclidean3D && type != Euclidean1D && type != Euclidean2D) {
    cerr
        << "[FactorGraphFilter] Error: currently, only EuclideanXD linearly interpolated parameters are supported"
        << endl;
    return ParameterWrapper_Ptr();
  }

  _params[name] = boost::shared_ptr<ParameterVerticesManager>(
      new LinearlyInterpolatedEuclideanParameter(spacing, _optimizer, type,
          name, x0));
  _params[name]->setFixed(isFixed);

  return ParameterWrapper_Ptr(new ParameterWrapper_Impl(_params[name].get()));
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addLinearlyInterpolatedParameter(
    const std::string& name, double x0, bool isFixed, double spacing) {
  Eigen::VectorXd tmp(1);
  tmp << x0;

  return addLinearlyInterpolatedParameter(Euclidean1D, name, tmp, isFixed,
      spacing);
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::addParameterBlender(
    ParameterTypes type, const std::string &name,
    ParameterWrapperVector_Ptr toblend) {

  std::vector<ParameterVerticesManager *> pv;
  for (auto pit = toblend->begin(); pit != toblend->end(); ++pit) {
    ParameterVerticesManager *pvm = boost::static_pointer_cast<
        ParameterWrapper_Impl>(*pit)->_param;
    pv.push_back(pvm);
  }

  _params[name] = boost::shared_ptr<ParameterVerticesManager>(
      new ParameterBlender(_optimizer, type, name, pv));

  return ParameterWrapper_Ptr(new ParameterWrapper_Impl(_params[name].get()));
}

MeasurementEdgeWrapper_Ptr FactorGraphFilter_Impl::addPriorOnPose(
    PoseVertexWrapper_Ptr pose, const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& cov) {

  PoseVertex *p = boost::static_pointer_cast<PoseVertexWrapper_Impl>(pose)->_v;

//test that p is an existing and active vertex
  if (p != NULL) {
    if (_poses.count(p->getTimestamp()) != 1) {
      cerr
          << "[FactorGraphFilter] Error: the provided pose vertex does not exist in the graph!"
          << endl;

      return MeasurementEdgeWrapper_Ptr();
    }
  }

  addPriorOnPose_i(p, x0, cov);

  return MeasurementEdgeWrapper_Ptr();
}

MeasurementEdgeWrapper_Ptr FactorGraphFilter_Impl::addPriorOnConstantParameter(
    PriorEdgeTypes type, const string &name, const Eigen::VectorXd &x0,
    const Eigen::MatrixXd &cov) {

  ParameterVerticesManager *parameter = getParameterByName_i(name);

  if (parameter == NULL) {
    cerr << "[FactorGraphFilter] Error: parameter " << name
        << " does not exist " << endl;

    return MeasurementEdgeWrapper_Ptr();
  }

  ConstantParameter *cnst_par = dynamic_cast<ConstantParameter *>(parameter);
  if (cnst_par == NULL) {
    cerr << "[FactorGraphFilter] Error: parameter " << name
        << " is not constant. " << endl;

    return MeasurementEdgeWrapper_Ptr();
  }

  g2o::OptimizableGraph::Vertex *v = cnst_par->getVertices(0.0)->second;

  BasePriorEdgeInterface *priorif;

  switch (type) {

  case Euclidean1DPrior:
    priorif = new Eucl1DPriorEdge;
    break;
  case Euclidean3DPrior:
    priorif = new Eucl3DPriorEdge;
    break;
  case SE3Prior:
    priorif = new SE3PriorEdge;
    break;
  case FHPPriorOnHomogeneousPoint:
    priorif = new FHPPriorOnHomogeneousPointEdge;
    break;
  }

  g2o::OptimizableGraph::Edge *edge = priorif->getg2oOptGraphPointer();

  edge->vertices()[0] = v;

  priorif->setMeasurement(x0);
  priorif->setNoiseCov(cov);

  _optimizer->addEdge(edge);

#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr << "[FactorGraphFilter] Info: adding prior edge on " << v->id()
  << endl;
#   endif

// TODO: return the pointer to the edge. Problem, BasePriorEdgeInterface is not connected with GenericEdgeInterface
  return MeasurementEdgeWrapper_Ptr();

}

MeasurementEdgeWrapper_Ptr FactorGraphFilter_Impl::addPriorOnTimeVaryingParameter(
    PriorEdgeTypes type, const std::string& name, double t,
    const Eigen::VectorXd& x0, const Eigen::MatrixXd& cov) {

  ParameterVerticesManager *parameter = getParameterByName_i(name);

  if (parameter == NULL) {
    cerr << "[FactorGraphFilter] Error: parameter " << name
        << " does not exist " << endl;

    return MeasurementEdgeWrapper_Ptr();
  }

  LinearlyInterpolatedEuclideanParameter *li_par;
  LimitedBandwithEuclideanParameter *lbw_par;

  g2o::OptimizableGraph::Vertex *v;

//TODO: this exploits friendship (should we modify?)
  if ((li_par =
      dynamic_cast<LinearlyInterpolatedEuclideanParameter *>(parameter)) != NULL) {
    v = li_par->getVertexNearestTo(t)->getg2oOptGraphPointer();
  } else if ((lbw_par =
      dynamic_cast<LimitedBandwithEuclideanParameter *>(parameter)) != NULL) {
    v = lbw_par->getVertexNearestTo(t)->getg2oOptGraphPointer();
  } else {
    cerr << "[FactorGraphFilter] Error: parameter " << name
        << " is not time varying. " << endl;

    return MeasurementEdgeWrapper_Ptr();
  }

  BasePriorEdgeInterface *priorif;

  switch (type) {

  case Euclidean1DPrior:
    priorif = new Eucl1DPriorEdge;
    break;
  case Euclidean3DPrior:
    priorif = new Eucl3DPriorEdge;
    break;
  case SE3Prior:
    priorif = new SE3PriorEdge;
    break;
  case FHPPriorOnHomogeneousPoint:
    priorif = new FHPPriorOnHomogeneousPointEdge;
    break;
  }

  g2o::OptimizableGraph::Edge *edge = priorif->getg2oOptGraphPointer();

  edge->vertices()[0] = v;

  priorif->setMeasurement(x0);
  priorif->setNoiseCov(cov);

  _optimizer->addEdge(edge);

#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr << "[FactorGraphFilter] Info: adding prior edge on " << v->id()
  << endl;
#   endif

// TODO: return the pointer to the edge. Problem, BasePriorEdgeInterface is not connected with GenericEdgeInterface
  return MeasurementEdgeWrapper_Ptr();
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::addPose(double t) {
  PoseVertex *v = addPose_i(t);

  return PoseVertexWrapper_Ptr(v != NULL ? new PoseVertexWrapper_Impl(v) : NULL);
}

PoseVertex *FactorGraphFilter_Impl::addPose_i(double t) {

// check if we have a pose vertex whose timestamp is too similar with respect to this
  PoseVertex *existing_v = getNearestPoseByTimestamp_i(t);
  if (existing_v != NULL && fabs(existing_v->getTimestamp() - t) < 1e-6) {
    cerr
        << "[FactorGraphFilter] Error: there exist a pose vertex which is within 1e-6s from t"
        << endl;
    return NULL;
  }

// initialize the pose vertex
  PoseVertex *v = new PoseVertex;

  v->setCategory("PoseSE3(W)");
  v->setTimestamp(t);

  v->setUserData(new PoseVertexMetadata);

// add the pose vertex to the g2o graph and to my poses collection
  _poses[t] = v;
  _optimizer->addVertex(v);

// update parameter vertices windows
  map<string, boost::shared_ptr<ParameterVerticesManager> >::const_iterator it =
      _params.begin();

  while (it != _params.end()) {

    PoseVertex *first, *last;
    first = getNewestPose_i();
    last = getOldestPose_i();

    it->second->updateVertexSet(last->getTimestamp(), first->getTimestamp());

    ++it;
  };

# ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr << "[FactorGraphFilter] Info: adding Pose vertex at t="
  << ROAMutils::StringUtils::writeNiceTimestamp(t) << ", id " << v->id()
  << endl;
# endif

  return v;
}

MeasurementEdgeWrapperVector_Ptr FactorGraphFilter_Impl::addSequentialMeasurement(
    const string& sensorName, double timestamp, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& cov) {

// retrieve the sensor descriptor
  map<string, struct Sensor>::iterator sensor_it = _sensors.find(sensorName);

  if (sensor_it == _sensors.end()) {
    cerr << "[FactorGraphFilter] Error: Sensor '" << sensorName
        << "' undefined." << endl;
    return MeasurementEdgeWrapperVector_Ptr();
  }

  struct Sensor &sensor = sensor_it->second;

# ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr << "[FactorGraphFilter] Info: new measurement for sensor "
  << sensorName << " at t="
  << ROAMutils::StringUtils::writeNiceTimestamp(timestamp) << endl;
# endif

  MeasurementEdgeWrapperVector_Ptr ret(new MeasurementEdgeWrapperVector);

  if (sensor.isMaster == true) {

    // if we still do not have an initial pose the measurement will be deferred
    if (_initialPoseSet == false) {
      cerr
          << "[FactorGraphFilter] Error: master measurement provided with no initial pose set."
          << endl;
      return ret;
    }

    // call with a back inserted iterator for this list
    list<GenericEdgeInterface *> inserted;

    addMasterSequentialMeasurement_i(back_inserter(inserted), sensor, timestamp,
        z, cov);

    // for each edge, instantiate a wrapper and stuff it into ret
    for (list<GenericEdgeInterface *>::iterator it = inserted.begin();
        it != inserted.end(); ++it) {
      ret->push_back(
          MeasurementEdgeWrapper_Ptr(new MeasurementEdgeWrapper_Impl(*it)));
    }
  } else {

    GenericEdgeInterface *inserted;

    inserted = addNonMasterSequentialMeasurement_i(sensor, timestamp, z, cov);

    ret->push_back(
        MeasurementEdgeWrapper_Ptr(new MeasurementEdgeWrapper_Impl(inserted)));
  }

  return ret;
}

template<typename OutputIterator>
void FactorGraphFilter_Impl::addMasterSequentialMeasurement_i(
    OutputIterator outIter, struct Sensor &sensor, double timestamp,
    const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) {

// add the new pose vertex
  PoseVertex *newPose = addPose_i(timestamp);

  if (newPose == NULL) {
    cerr
        << "[FactorGraphFilter] Error: Could not add the new measurement for master sensor '"
        << sensor.name << endl;
    return;
  }

// pick the needed pose vertices
// some of these might be null. It is handled in the lower level function

  PoseVertex *th, *last, *secondlast;

  th = newPose;
  last = sensor.last;
  secondlast = sensor.secondlast;

  sensor.secondlast = last;
  sensor.last = th;

  GenericEdgeInterface *edge = addMeasurement_i(sensor, timestamp, z, cov, th,
      last, secondlast);

  if (edge != NULL) {
    // predictor
    edge->predictNextState();

    handleDeferredMeasurements_i(outIter);

    (*outIter)++ = edge;
  }
}

bool FactorGraphFilter_Impl::checkSensorOrder(const struct Sensor &sensor,
    const PoseVertex* v2, const PoseVertex* v1, const PoseVertex* v0) {
  return !(v2 == NULL || (sensor.order >= 1 && v1 == NULL)
      || (sensor.order == 2 && v0 == NULL));
}

bool FactorGraphFilter_Impl::marginalizeNodes(
    vector<PoseVertexWrapper*>& nodes) {

// 0 - make a set with the vertices wrapped by the PoseVertexWrappers
//     in the meanwhile, check also that all the nodes being supplied appear in the _poses map

  set<g2o::HyperGraph::Vertex *> nodeSet;

  for (vector<PoseVertexWrapper *>::const_iterator it = nodes.begin();
      it != nodes.end(); ++it) {
    PoseVertex *pose = static_cast<PoseVertexWrapper_Impl *>(*it)->_v;

    nodeSet.insert(pose);

    //poses are unique given the timestamp, it is used as a key
    PoseMapIterator p_it = _poses.find((*it)->getTimestamp());
    if (p_it == _poses.end()) {
      cerr
          << "[FactorGraphFilter] Error: unknown or no longer existing vertex supplied with timestamp "
          << (*it)->getTimestamp() << endl;
      return false;
    }

    // ensure that the pose accessed by timestamp is actually the same one
    assert(p_it->second != pose);
  }

# ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  if (nodes.size() != nodeSet.size()) {
    cerr
    << "[FactorGraphFilter] Warning: the provided vector contains duplicate pose nodes!"
    << endl;
  }
# endif

  return marginalizeNodes_i(nodeSet);
}

void FactorGraphFilter_Impl::handlePriorsOnOldestPose() {
// we add priors only in dead reckoning mode
  if (_deadReckoning == true) {
    PoseVertex *p = getOldestPose_i();

    assert(p != NULL);

    // search if one of the p edges is a prior

    for (g2o::OptimizableGraph::EdgeSet::iterator it = p->edges().begin();
        it != p->edges().end(); ++it) {
      SE3PriorEdge *prior = dynamic_cast<SE3PriorEdge *>(*it);
      if (prior != NULL) {
        //it already has a prior. good. I'm done
        return;
      }
    }

    // if this point is reached, no prior has been set in the vertex. Create it.

    addPriorOnPose_i(p, p->estimate(), Eigen::MatrixXd::Identity(6, 6));

  }
}

SE3PriorEdge* FactorGraphFilter_Impl::addPriorOnPose_i(PoseVertex* p,
    const Eigen::VectorXd& x0, const Eigen::MatrixXd& cov) {

  SE3PriorEdge *prior = new SE3PriorEdge;

  prior->vertices()[0] = p;

  prior->setMeasurement(x0);
  prior->setNoiseCov(cov);

  _optimizer->addEdge(prior);

#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr << "[FactorGraphFilter] Info: adding prior edge on " << p->id()
  << endl;
#   endif

  return prior;
}

void FactorGraphFilter_Impl::setDeadReckoning(bool deadReckoning) {
  _deadReckoning = deadReckoning;
}

bool FactorGraphFilter_Impl::marginalizeNodes_i(
    set<g2o::HyperGraph::Vertex *>& nodeSet) {

  assert(nodeSet.size() != 0);

// 1 - get involved nodes and edges

  set<g2o::HyperGraph::Vertex *> markovBlanket;
  set<g2o::HyperGraph::Edge *> fallingEdges;

  GenericLinearConstraintFactory::getMarkovBlanket(nodeSet, markovBlanket,
      fallingEdges);

// 2 - check that every one of the fallingEdges has already been employed in an estimation

  for (set<g2o::HyperGraph::Edge *>::const_iterator it = fallingEdges.begin();
      it != fallingEdges.end(); ++it) {
    g2o::OptimizableGraph::Edge *oe =
        static_cast<g2o::OptimizableGraph::Edge *>(*it);
    MeasurementEdgeMetadata *meta =
        static_cast<MeasurementEdgeMetadata *>(oe->userData());

    // only MeasurementEdges have metadata
    // note that insertedAterLastEstimate would be always false
    // for edges such as priors or GLC

    if (meta != NULL && meta->insertedAterLastEstimate == true) {

      GenericEdgeInterface *e = dynamic_cast<GenericEdgeInterface *>(oe);

      cerr
          << "[FactorGraphFilter] Warning: one of the edges which are going to be removed has never been employed in estimation: "
          << e->writeDebugInfo() << endl;

      // TODO: in this case I might just remove it from the fallingEdge set..
      return false;
    }
  }

// 3 - create the constraint

  GenericLinearConstraint *edge = GenericLinearConstraintFactory::buildGLC(
      nodeSet, markovBlanket, fallingEdges);

  if (edge == NULL) {
# 	ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr << "[FactorGraphFilter] Error: error during node marginalization."
    << endl;
#		endif

    return false;
  }

  _optimizer->addEdge(edge);

// 4 - search the falling edges for the dt vertices and remove them

  for (set<g2o::HyperGraph::Edge *>::iterator it = fallingEdges.begin();
      it != fallingEdges.end(); ++it) {

    // only GenericEdges may have dt vertices
    GenericEdgeInterface *ge = dynamic_cast<GenericEdgeInterface *>(*it);
    if (ge != NULL && ge->getOrder() >= 0) { // it returns -1 for edges for which order is not meaningful and thus no dt

      if (ge->getOrder() >= 1) {
        DtVertex *dt12 = NULL;

#				ifdef DEBUG_BUILD
        dt12 = dynamic_cast<DtVertex *>( (*it)->vertices()[1] );
        assert(dt12 != NULL);
#				else
        dt12 = static_cast<DtVertex *>((*it)->vertices()[1]);
#				endif

        _optimizer->removeVertex(dt12);
      }

      if (ge->getOrder() == 2) {
        DtVertex *dt01 = NULL;

#				ifdef DEBUG_BUILD
        dt01 = dynamic_cast<DtVertex *>( (*it)->vertices()[3] );
        assert(dt01 != NULL);
#				else
        dt01 = static_cast<DtVertex *>((*it)->vertices()[3]);
#				endif

        _optimizer->removeVertex(dt01);
      }
    }
  }

// 5 - search falling nodes in sensor descriptors

  for (auto s_it = _sensors.begin(); s_it != _sensors.end(); ++s_it) {
    if (nodeSet.find(s_it->second.secondlast) != nodeSet.end()) {
      s_it->second.secondlast = NULL;
    }

    if (nodeSet.find(s_it->second.last) != nodeSet.end()) {
      s_it->second.secondlast = NULL;
      s_it->second.last = NULL;
    }
  }

// 6 - remove the nodes (this causes the fallinEdges to be deleted as well)
//     and remove them from _poses as well

  for (set<g2o::HyperGraph::Vertex *>::iterator it = nodeSet.begin();
      it != nodeSet.end(); ++it) {

    PoseVertex *p = dynamic_cast<PoseVertex *>(*it);

    if (p != NULL) {
      int erased = _poses.erase(p->getTimestamp());

      assert(erased == 1);
    }

    _optimizer->removeVertex(static_cast<g2o::OptimizableGraph::Vertex *>(*it));

    // TODO: remove the node from the spatial index
  }

  /* 7 - deal with parameter vertices

   the parameters vertices have been already handled.
   The ones that have to fall go into the nodeSet argument

   // first empty the parameter vertices trash
   for (auto it = _params.begin(); it != _params.end(); ++it) {

   // first update the vertex set so all the parameter vertices that could fall will be trashed

   PoseVertex *first, *last;
   first = getNewestPose_i();
   last = getOldestPose_i();

   it->second->updateVertexSet(last->getTimestamp(), first->getTimestamp());

   // now empty the trash

   it->second->emptyTrash();
   }
   //*/

  return true;
}

bool FactorGraphFilter_Impl::forgetNodes_i(
    set<g2o::HyperGraph::Vertex*>& nodeSet) {

// be tolerant with respect to empty nodeSets
  if (nodeSet.size() == 0) {
    return true;
  }

// 1 - get involved nodes and edges

  set<g2o::HyperGraph::Vertex *> markovBlanket;
  set<g2o::HyperGraph::Edge *> fallingEdges;

  GenericLinearConstraintFactory::getMarkovBlanket(nodeSet, markovBlanket,
      fallingEdges);

// 2 - search the falling edges for the dt vertices and remove them

  for (set<g2o::HyperGraph::Edge *>::iterator it = fallingEdges.begin();
      it != fallingEdges.end(); ++it) {

    // only GenericEdges may have dt vertices
    GenericEdgeInterface *ge = dynamic_cast<GenericEdgeInterface *>(*it);
    if (ge != NULL && ge->getOrder() >= 0) { // it returns -1 for edges for which order is not meaningful and thus no dt

      if (ge->getOrder() >= 1) {
        DtVertex *dt12 = NULL;

#				ifdef DEBUG_BUILD
        dt12 = dynamic_cast<DtVertex *>( (*it)->vertices()[1] );
        assert(dt12 != NULL);
#				else
        dt12 = static_cast<DtVertex *>((*it)->vertices()[1]);
#				endif

        _optimizer->removeVertex(dt12);
      }

      if (ge->getOrder() == 2) {
        DtVertex *dt01 = NULL;

#				ifdef DEBUG_BUILD
        dt01 = dynamic_cast<DtVertex *>( (*it)->vertices()[3] );
        assert(dt01 != NULL);
#				else
        dt01 = static_cast<DtVertex *>((*it)->vertices()[3]);
#				endif

        _optimizer->removeVertex(dt01);
      }
    }
  }

// 3 - search falling nodes in sensor descriptors

  for (auto s_it = _sensors.begin(); s_it != _sensors.end(); ++s_it) {
    if (nodeSet.find(s_it->second.secondlast) != nodeSet.end()) {
      s_it->second.secondlast = NULL;
    }

    if (nodeSet.find(s_it->second.last) != nodeSet.end()) {
      s_it->second.secondlast = NULL;
      s_it->second.last = NULL;
    }
  }

// 4 - remove the nodes (this causes the fallinEdges to be deleted as well)
//     and remove them from _poses as well

  for (set<g2o::HyperGraph::Vertex *>::iterator it = nodeSet.begin();
      it != nodeSet.end(); ++it) {

    int erased = _poses.erase(static_cast<PoseVertex *>(*it)->getTimestamp());
    assert(erased == 1); // one and only one has been erased

    _optimizer->removeVertex(static_cast<g2o::OptimizableGraph::Vertex *>(*it));

    // TODO: remove the node from the spatial index
  }

  /* 5 - deal with parameter vertices

   the parameters vertices have been already handled.
   The ones that have to fall go into the nodeSet argument


   // first empty the parameter vertices trash
   for (auto it = _params.begin(); it != _params.end(); ++it) {

   // first update the vertex set so all the parameter vertices that could fall will be trashed

   PoseVertex *first, *last;
   first = getNewestPose_i();
   last = getOldestPose_i();

   it->second->updateVertexSet(last->getTimestamp(), first->getTimestamp());

   // now empty the trash

   it->second->emptyTrash();
   }
   //*/

  return true;
}

GenericEdgeInterface* FactorGraphFilter_Impl::addNonMasterSequentialMeasurement_i(
    struct Sensor& sensor, double timestamp, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& cov) {

// if the measurement is more recent with respect to the newest vertex we defer it
  PoseVertex *newest = getNewestPose_i();
  if (newest == NULL || newest->getTimestamp() < timestamp) {
    deferMeasurement(sensor, timestamp, z, cov);
    return NULL;
  }

// get the pose which is nearer to the provided timestamp
  PoseVertex *th = getNearestPoseByTimestamp_i(timestamp, false);
  assert(th != NULL); //it should be impossible since I have at least a pose

// if the nearest pose is the same as the last employed for this sensor
  if (th == sensor.last) {
#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr
    << "[FactorGraphFilter] FixMe: two measurements for the same time slot ("
    << sensor.name << "). Discarding the second." << endl;
#   endif

    return NULL;
  }

// if this measurement is before the first pose I have
  if (sensor.last != NULL && th->getTimestamp() < sensor.last->getTimestamp()) {
#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr << "[FactorGraphFilter] FixMe: out of order readings for sequential sensor "
    << sensor.name << ". Discarding this one." << endl;
#   endif

    return NULL;
  }

// collect the poses from the sensor descriptor
  PoseVertex *last, *secondlast;

  last = sensor.last;
  secondlast = sensor.secondlast;

// advance the slots in sensor descriptor
  sensor.secondlast = last;
  sensor.last = th;

// check if I have enough vertices wrt the sensor order
  if (!checkSensorOrder(sensor, th, last, secondlast)) {
    deferMeasurement(sensor, timestamp, z, cov);
    return NULL;
  }

// try to add the measurement (there exist more lower level failure cases)
  GenericEdgeInterface *edge = addMeasurement_i(sensor, timestamp, z, cov, th,
      last, secondlast);

  return edge;
}

MeasurementEdgeWrapper_Ptr FactorGraphFilter_Impl::addMeasurement(
    const string& sensorName, double timestamp, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& cov, PoseVertexWrapper_Ptr v2,
    PoseVertexWrapper_Ptr v1, PoseVertexWrapper_Ptr v0) {

// retrieve the sensor descriptor
  map<string, struct Sensor>::iterator sensor_it = _sensors.find(sensorName);

  if (sensor_it == _sensors.end()) {
    cerr << "[FactorGraphFilter] Error: Sensor '" << sensorName
        << "' undefined." << endl;
    return MeasurementEdgeWrapper_Ptr();
  }

  struct Sensor &sensor = sensor_it->second;

// access the g2o pose vertices
  PoseVertex *g2oV2 = (
      v2 ? boost::static_pointer_cast<PoseVertexWrapper_Impl>(v2)->_v : NULL);
  PoseVertex *g2oV1 = (
      v1 ? boost::static_pointer_cast<PoseVertexWrapper_Impl>(v1)->_v : NULL);
  PoseVertex *g2oV0 = (
      v0 ? boost::static_pointer_cast<PoseVertexWrapper_Impl>(v0)->_v : NULL);

// test if every vertex appear in _poses

  if (g2oV2 != NULL && testExistance(g2oV2) == false) {
    cerr << "[FactorGraphFilter] Error: Vertex v2 not in graph" << endl;
    return MeasurementEdgeWrapper_Ptr();
  }
  if (g2oV1 != NULL && testExistance(g2oV1) == false) {
    cerr << "[FactorGraphFilter] Error: Vertex v1 not in graph" << endl;
    return MeasurementEdgeWrapper_Ptr();
  }
  if (g2oV0 != NULL && testExistance(g2oV0) == false) {
    cerr << "[FactorGraphFilter] Error: Vertex v0 not in graph" << endl;
    return MeasurementEdgeWrapper_Ptr();
  }

  GenericEdgeInterface *e = addMeasurement_i(sensor, timestamp, z, cov, g2oV2,
      g2oV1, g2oV0);

  return MeasurementEdgeWrapper_Ptr(
      e != NULL ? new MeasurementEdgeWrapper_Impl(e) : NULL);
}

GenericEdgeInterface *FactorGraphFilter_Impl::addMeasurement_i(
    struct Sensor &sensor, double timestamp, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& cov, PoseVertex* v2, PoseVertex* v1,
    PoseVertex* v0) {

// check if the user has provided enough vertices
  if (!checkSensorOrder(sensor, v2, v1, v0)) {
    cerr << "[FactorGraphFilter] Error: Not enough vertices provided for '"
        << sensor.name << "', which is of order " << sensor.order << endl;

    return NULL;
  }

// create the edge
  GenericEdgeInterface *e = NULL;

  switch (sensor.type) {
  case AbsolutePosition:
    e = new QuaternionGenericEdge<AbsolutePositionM>;
    break;
  case AbsolutePose:
    e = new QuaternionGenericEdge<AbsolutePoseM>;
    break;
  case LinearVelocity:
    e = new QuaternionGenericEdge<LinearVelocityM>;
    break;
  case AngularVelocity:
    e = new QuaternionGenericEdge<AngularVelocityM>;
    break;
  case LinearAcceleration:
    e = new QuaternionGenericEdge<AccelerationM>;
    break;
  case AckermannOdometer:
    e = new QuaternionGenericEdge<AckermannM>;
    break;
  case AckermannConstraint:
    e = new QuaternionGenericEdge<AckermannKinematicNoInputsM>;
    break;
  case TriskarOdometer:
    e = new QuaternionGenericEdge<TriskarKinematicM>;
    break;
  case DifferentialDriveOdometer:
    e = new QuaternionGenericEdge<DifferentialDriveKinematicM>;
    break;
  case GenericOdometer:
    e = new QuaternionGenericEdge<GenericOdometerM>;
    break;
  case Displacement:
    e = new QuaternionGenericEdge<DisplacementM>;
    break;
  case IMUintegralDeltaP:
    e = new QuaternionGenericEdge<IMUImtegralDeltaPM>;
    break;
  case IMUintegralDeltaQ:
    e = new QuaternionGenericEdge<IMUImtegralDeltaQM>;
    break;
  case VectorField:
    e = new QuaternionGenericEdge<VectorFieldM>;
    break;
  case VectorFieldAsCompass:
    e = new QuaternionGenericEdge<VectorFieldAsCompassM>;
    break;
  case FixedFeaturePosition:
    e = new QuaternionGenericEdge<FixedFeaturePositionM>;
    break;
  case FixedFeaturePose:
    e = new QuaternionGenericEdge<FixedFeaturePoseM>;
    break;
  case ImagePlaneProjection:
    e = new QuaternionGenericEdge<ImagePlaneProjectionM>;
    break;
  case FramedHomogeneousPoint:
    e = new QuaternionGenericEdge<FramedHomogeneousPointM>;
    break;
  case RectangularObject:
    e = new QuaternionGenericEdge<RectangularObjectM>;
    break;
  case AnchoredRectangularObject:
    e = new QuaternionGenericEdge<AnchoredRectangularObjectM>;
    break;
  case AnchoredRectangularObjectFirst:
    e = new QuaternionGenericEdge<AnchoredRectangularObjectFirstM>;
    break;
  case PlanarConstraint:
    e = new QuaternionGenericEdge<PlanarConstraintM>;
    break;
  default:
    cerr << "[FactorGraphFilter] Error: unknown measurement type" << endl;
    return NULL;
    break;
  }

# ifdef DEBUG_BUILD
// perform some checks on the provided vector and matrices
  if (z.rows() != e->measurementDimension() || z.cols() != 1) {
    cerr
    << "[FactorGraphFilter] Error: wrong measurement vector size provided: ("
    << z.rows() << "x" << z.cols() << ") instead of ("
    << e->measurementDimension() << "x1)" << endl;

    delete e;
    return NULL;
  }

  if (cov.rows() != e->noiseDimension() || cov.cols() != e->noiseDimension()) {
    cerr
    << "[FactorGraphFilter] Error: wrong covariance matrix size provided: ("
    << cov.rows() << "x" << cov.cols() << ") instead of ("
    << e->noiseDimension() << "x" << e->noiseDimension() << ")"
    << endl;

    delete e;
    return NULL;
  }
# endif

// add dt vertices (the needed ones)
  DtVertex *dt12 = NULL;
  if (sensor.order > 0) {
    dt12 = addDt(v2, v1);
  }

  DtVertex *dt01 = NULL;
  if (sensor.order > 1) {
    dt01 = addDt(v1, v0);
  }

// call the collect method, which substantially loads the vertices into the edge
  EstimationEdgeCollectInterface *cif =
      dynamic_cast<EstimationEdgeCollectInterface *>(e);
  assert(cif != NULL);

  cif->collect(v0, v1, v2, timestamp, dt01, dt12, sensor.name, _params);

// fill in the measurement and the covariance
  e->setMeasurement_GE(z);
  e->setNoiseCov(cov);
  e->setFrameCounter(0); // TODO: implement frame counter handling

// set robustification according to sensor configuration

  g2o::OptimizableGraph::Edge *oe = *e; // conversion operator

// if the sensor is currently robustified, set these properties into the edge
  if (sensor.robustified) {
    oe->setRobustKernel(true);
    oe->setHuberWidth(sensor.huberWidth);
  }

// create the metadata
  oe->setUserData(new MeasurementEdgeMetadata);

// insert the edge into the g2o graph
  _optimizer->addEdge(oe);

#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  Eigen::IOFormat basicFormat(3, Eigen::DontAlignCols, ", ", ", ", "", "", "{",
      "}");

  cerr
  << "[FactorGraphFilter] Info: inserting low level measurement, ids: ("
  << v2->id();
  if (v1 != NULL) {
    cerr << ", " << v1->id();
  }
  if (v0 != NULL) {
    cerr << ", " << v0->id();
  }
  cerr << ") and z: " << z.format(basicFormat) << endl;
#   endif

  return e;
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::getNearestPoseByTimestamp(
    double t, bool onlyBefore) {

  PoseVertex *nearest = getNearestPoseByTimestamp_i(t, onlyBefore);

  return PoseVertexWrapper_Ptr(
      nearest != NULL ? new PoseVertexWrapper_Impl(nearest) : NULL);
}

PoseVertex* FactorGraphFilter_Impl::getNearestPoseByTimestamp_i(double t,
    bool onlyBefore) {

  if (_poses.size() == 0) {
    return NULL;
  }

// find the first pose after t
  PoseMapIterator after;
  after = _poses.upper_bound(t);

// if there are no poses prior the one which is just after t
  if (after == _poses.begin()) {

    // if I wanted only poses before t, well we don't have
    if (onlyBefore == true) {
      return NULL;
    } else {
      return after->second;
    }
  }

// otherwise, we have for sure a pose before
  PoseMapIterator before = after;
  --before;

// if there is no pose after t or the user wants only the poses before t
  if (after == _poses.end() || onlyBefore) {
    return before->second;
  }

// otherwise I have to chose based on which is nearer to t
  double t_before = before->second->getTimestamp();
  double t_after = after->second->getTimestamp();

// some checks to be sure everything is correct
  assert(t_before <= t);
  assert(t_after > t);

  if (t - t_before > t_after - t) {
    return after->second;
  } else {
    return before->second;
  }
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::getNewestPose() {
  PoseVertex *last = getNewestPose_i();

  return PoseVertexWrapper_Ptr(
      last != NULL ? new PoseVertexWrapper_Impl(last) : NULL);
}

PoseVertex * FactorGraphFilter_Impl::getNewestPose_i() {
  if (_poses.size() == 0) {
    return NULL;
  }

  PoseMapIterator last = --_poses.end();

  return last->second;
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::getOldestPose() {
  PoseVertex *last = getOldestPose_i();

  return PoseVertexWrapper_Ptr(
      last != NULL ? new PoseVertexWrapper_Impl(last) : NULL);
}

PoseVertex * FactorGraphFilter_Impl::getOldestPose_i() {
  if (_poses.size() == 0) {
    return NULL;
  }

  return _poses.begin()->second;
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::getNthPose(int n) {

  PoseVertex *pose = getNthPose_i(n);

  return PoseVertexWrapper_Ptr(
      pose != NULL ? new PoseVertexWrapper_Impl(pose) : NULL);
}

PoseVertexWrapper_Ptr FactorGraphFilter_Impl::getNthOldestPose(int n) {

  PoseVertex *pose = getNthOldestPose_i(n);

  return PoseVertexWrapper_Ptr(
      pose != NULL ? new PoseVertexWrapper_Impl(pose) : NULL);
}

PoseVertex *FactorGraphFilter_Impl::getNthPose_i(int n) {
  PoseVertex *pose = NULL;

  if (n < _poses.size()) {
    auto it = _poses.end();
    for (int k = 0; k <= n; k++) {
      --it;
    }

    pose = it->second;
  }

  return pose;
}

PoseVertex *FactorGraphFilter_Impl::getNthOldestPose_i(int n) {
  PoseVertex *pose = NULL;

  if (n < _poses.size()) {
    auto it = _poses.begin();
    for (int k = 0; k < n; k++) {
      ++it;
    }

    pose = it->second;
  }

  return pose;
}

GenericVertex<Eucl1DV>* FactorGraphFilter_Impl::addDt(const PoseVertex *v1,
    const PoseVertex *v2) {
  GenericVertex<Eucl1DV> *dt_v = new GenericVertex<Eucl1DV>;

// initialize the dt vertex
  dt_v->estimate()(0) = v1->getTimestamp() - v2->getTimestamp();
  dt_v->setFixed(true);
  dt_v->setCategory("Poses_dt");

  dt_v->setTimestamp(v1->getTimestamp());

  _optimizer->addVertex(dt_v);

# ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr << "[FactorGraphFilter] Info: adding dt vertex between " << v2->id()
  << " and " << v1->id() << ", Delta: "
  << ROAMutils::StringUtils::writeNiceTimestamp(dt_v->estimate()(0))
  << " with id=" << dt_v->id() << endl;
# endif

  return dt_v;
}

bool FactorGraphFilter_Impl::estimate(int nIterations) {

// we handle priors only in case of full estimation
  handlePriorsOnOldestPose();

// generate the set ot the edges which are involved in the optimization
// i.e., all the edges that are incident to pose nodes in the _poses map.
// this is usefull if the graph does contain also other nodes and edges

  g2o::HyperGraph::EdgeSet eset;
  for (auto pit = _poses.begin(); pit != _poses.end(); ++pit) {
    if (!pit->second->fixed()) {
      eset.insert(pit->second->edges().begin(), pit->second->edges().end());
    }
  }

  return estimate_i(eset, nIterations);
}

bool FactorGraphFilter_Impl::estimate(PoseVertexWrapperVector poses,
    int nIterations) {

  g2o::HyperGraph::EdgeSet eset;

  for (auto pit = poses.begin(); pit != poses.end(); ++pit) {

    const PoseVertexWrapper_Ptr &pw = *pit;

    if (!pw) {
      cerr
          << "[FactorGraphFilter] Error: Null PoseVertexWrapper_Ptr in pose set"
          << endl;
      return false;
    }

    PoseVertex * pv = boost::static_pointer_cast<PoseVertexWrapper_Impl>(pw)->_v;

    if (!testExistance(pv)) {
      cerr << "[FactorGraphFilter] Error: one pose does not belong to the graph"
          << endl;
      return false;
    }

    if (!pv->fixed()) {
      eset.insert(pv->edges().begin(), pv->edges().end());
    }
  }

  return estimate_i(eset, nIterations);
}

bool FactorGraphFilter_Impl::estimate_i(g2o::HyperGraph::EdgeSet &eset,
    int nIterations) {

// we don't do anything until we get the first measurements
  if (eset.size() == 0) {
#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr << "[FactorGraphFilter] Warning: estimation launched with an empty edgeset" << endl;
#   endif
    return true;
  }

// there might be prior edges incident to parameter nodes
// TODO: there might be a more efficient way to do this

// 1- get the active vertices
  set<g2o::HyperGraph::Vertex *> activeVertices;
  for (auto eit = eset.begin(); eit != eset.end(); ++eit) {

    // please exclude fixed vertices
    for (auto vit = (*eit)->vertices().begin(); vit != (*eit)->vertices().end();
        ++vit) {
      g2o::OptimizableGraph::Vertex *ov =
          static_cast<g2o::OptimizableGraph::Vertex *>(*vit);

      if (!ov->fixed()) {
        activeVertices.insert(ov);
      }
    }
  }

// 2- insert in the eset the prior edges which are incident to any of these vertices
  for (auto vit = activeVertices.begin(); vit != activeVertices.end(); ++vit) {
    for (auto eit = (*vit)->edges().begin(); eit != (*vit)->edges().end();
        ++eit) {
      if (dynamic_cast<BasePriorEdgeInterface *>(*eit) != NULL) {
        eset.insert(*eit);
      }
    }
  }

  /* ----------- TODO: LBW WORKAROUND for limited bandwidth parameters
   // the newest vertex has to be fixed if not enough observations are available for it

   for (auto it = _params.begin(); it != _params.end(); ++it) {
   boost::shared_ptr<LimitedBandwithEuclideanParameter> lbw =
   boost::dynamic_pointer_cast<LimitedBandwithEuclideanParameter>(
   it->second);

   if (lbw != NULL && lbw->fixed() == false) { // if the parameter, not the vertex, is fixed, don't mess around with vertices
   auto vit = lbw->getVertices(getNewestPose_i()->getTimestamp());

   for (int k = 0; k < lbw->getWindowSize() - 1; k++, ++vit)
   ; // go to the last vertex

   #     ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
   GenericVertexInterface *gv = dynamic_cast<GenericVertexInterface *>(vit->second);
   cerr << "[FactorGraphFilter] Info: " << it->first << "{"
   << ROAMutils::StringUtils::writeNiceTimestamp(gv->getTimestamp())
   << "} n edges: " << vit->second->edges().size();
   #     endif

   if (vit->second->edges().size() < 2) {
   vit->second->setFixed(true);

   #       ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
   cerr << " so FIXING it";
   #       endif
   } else {
   vit->second->setFixed(false);
   }

   #     ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
   cerr << endl;
   #     endif

   }
   }

   // --- end of LBW WORKAROUND */

// stuff for estimation time statistics
  static ofstream ftStats("/tmp/roamfree/timeStats.txt"); // TODO: put the same folder as for _logger
  ftStats.precision(6);

  double tStart = g2o::get_time();

// if logging is enabled, write the current factor graph to a file
  if (_lowLevelLogging == true) {
    ofstream f("/tmp/roamfree/graph.txt"); // TODO: put the same folder as for _logger
    assert(f.is_open());
    f << writeFactorGraph();
    f.close();

    /* debug graph in dot, useless
     ofstream fdot("/tmp/roamfree/graph.dot"); // TODO: put the same folder as for _logger
     fdot << writeFactorGraphToDot();
     fdot.close();
     //*/
  }

// ----------- END

  _optimizer->initializeOptimization(eset);

  if (_lowLevelLogging == true) {
    ofstream fid("debug/Hstruct.txt"); // TODO: put the same folder as for _logger

    if (fid.is_open()) {
      fid << writeVertexIdMap();
      fid.close();
    }
  }

  double tInitialized = g2o::get_time();

// run the optimization

  bool ret = _optimizer->optimize(nIterations) || nIterations == 0;
  if (ret == false) {
# 	ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr << "[FactorGraphFilter] Error: estimation failed." << endl;
# 	endif

    return false;
  }

  double tOptimized = g2o::get_time();

  updatePosesAndEdgesMetadata();

  if (nIterations > 0) {
    computeCovariances();

    _spatialIndex->build(_poses);
  }

  double tCovariancesAndSpatial = g2o::get_time();

  if (_lowLevelLogging == true) {
    assert(_logger != NULL);
    _logger->sync();
  }

  double tLogging = g2o::get_time();

// write stats

  ftStats << (tInitialized - tStart) << ", " << (tOptimized - tInitialized)
      << ", " << (tCovariancesAndSpatial - tOptimized) << ", "
      << (tLogging - tCovariancesAndSpatial) << endl;

  return true;
}

FactorGraphFilter_Impl::Measurement::Measurement(struct Sensor& sensor,
    double t, const Eigen::VectorXd& z, const Eigen::MatrixXd& cov) :
    sensor(sensor), tstamp(t), z(new Eigen::VectorXd(z)), cov(
        new Eigen::MatrixXd(cov)) {
}

bool FactorGraphFilter_Impl::marginalizeOldNodes(double l) {
  double oldestAllowedT = getNewestPose_i()->getTimestamp() - l;

  if (oldestAllowedT <= getOldestPose_i()->getTimestamp()) {
    return true;
  }

  double newestMarginalizedT = -numeric_limits<double>::infinity();

  set<g2o::HyperGraph::Vertex *> toMarginalize;

  PoseMapIterator it = _poses.begin();
  while (it->first < oldestAllowedT) {
    assert(it->first == it->second->getTimestamp());

    toMarginalize.insert(it->second);

    newestMarginalizedT = it->first;

    ++it;
  }

// insert into the nodeSet also the old parameter nodes
  for (auto pit = _params.begin(); pit != _params.end(); ++pit) {
    pit->second->prepareForPoseRemoval(-std::numeric_limits<double>::infinity(),
        newestMarginalizedT);

    const map<double, g2o::OptimizableGraph::Vertex *> &trash =
        pit->second->getTrash();

    for (auto vit = trash.begin(); vit != trash.end(); ++vit) {
      toMarginalize.insert(vit->second);
    }
  }

// do the marginalization
  if (!marginalizeNodes_i(toMarginalize)) {
    return false;
  }

// update internal data structures in patameters
  for (auto pit = _params.begin(); pit != _params.end(); ++pit) {
    pit->second->emptyTrash();
  }

  return true;
}

bool FactorGraphFilter_Impl::forgetOldNodes(double l) {

  double oldestAllowedT = getNewestPose_i()->getTimestamp() - l;

  set<g2o::HyperGraph::Vertex *> toForget;

  PoseMapIterator it = _poses.begin();
  while (it->first < oldestAllowedT) {
    assert(it->first == it->second->getTimestamp());

    toForget.insert(it->second);

    ++it;
  }

  return forgetNodes_i(toForget);
}

void FactorGraphFilter_Impl::deferMeasurement(struct Sensor& sensor, double t,
    const Eigen::VectorXd& z, const Eigen::MatrixXd& cov) {
  struct Measurement meas(sensor, t, z, cov);
  _deferred.insert(meas); // default copy constructor, TODO: replace with emplace as soon as we got gcc 4.8.x

#     ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
  cerr
  << "[FactorGraphFilter] Info: deferring insertion of measurement from "
  << sensor.name << " at time "
  << ROAMutils::StringUtils::writeNiceTimestamp(t) << endl;
#     endif

}

template<typename OutputIterator>
void FactorGraphFilter_Impl::handleDeferredMeasurements_i(
    OutputIterator outIter) {
// I have to iterate and delete through a multiset, eventually deleting elements
// I have to be careful since erasing by means of an iterator invalidates it

  multiset<struct Measurement>::iterator it2 = _deferred.begin();
  while (it2 != _deferred.end()
      && it2->tstamp < getNewestPose_i()->getTimestamp()) {

    struct Measurement toadd = *it2;

    if (toadd.sensor.order > _poses.size() - 1) {

#     ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
      cerr << "[FactorGraphFilter] Info: not enough poses in window"
      << endl;
#     endif

      ++it2;
      continue;
    }

#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr << "[FactorGraphFilter] Info: now insert of measurement from "
    << toadd.sensor.name << " at time "
    << ROAMutils::StringUtils::writeNiceTimestamp(toadd.tstamp)
    << endl;
#   endif

    _deferred.erase(it2++); // use it2 and then, after, increment it.

    *(outIter++) = addNonMasterSequentialMeasurement_i(toadd.sensor,
        toadd.tstamp, *toadd.z, *toadd.cov);
  }
}

void FactorGraphFilter_Impl::updatePosesAndEdgesMetadata() {

  for (auto pit = _optimizer->activeVertices().begin();
      pit != _optimizer->activeVertices().end(); pit++) {
    PoseVertex *pose = dynamic_cast<PoseVertex *>(*pit);

    if (pose != NULL) {
      PoseVertexMetadata *meta =
          static_cast<PoseVertexMetadata *>(pose->userData());

      meta->hasBeenEstimated = true;

      g2o::HyperGraph::EdgeSet::iterator weit;
      for (weit = pose->edges().begin(); weit != pose->edges().end(); ++weit) {
        GenericEdgeInterface *e = dynamic_cast<GenericEdgeInterface *>(*weit);

        if (e != NULL) {
          g2o::OptimizableGraph::Edge *oe = *e;

          MeasurementEdgeMetadata *meta =
              static_cast<MeasurementEdgeMetadata *>(oe->userData());

          meta->insertedAterLastEstimate = false;
        }
      }
    }
  }
}

PoseVertexWrapperVector_Ptr FactorGraphFilter_Impl::getNeighbourPosesByDistance(
    PoseVertexWrapper_Ptr sample, double d) {

// get the internal g2o vertex
  PoseVertex * v =
      boost::static_pointer_cast<PoseVertexWrapper_Impl>(sample)->_v;

// test if sample vertex appears appears in poses
  if (testExistance(v) == false) {
    cerr << "[FactorGraphFilter] Error: provided vertex not in graph" << endl;
    return PoseVertexWrapperVector_Ptr();
  }

  list<PoseVertex *> neighbours;
  _spatialIndex->rangeSearch(v->estimate(), d, back_inserter(neighbours));

// wrap the vertices and return
  PoseVertexWrapperVector_Ptr ret(new PoseVertexWrapperVector);

  for (list<PoseVertex *>::iterator it = neighbours.begin();
      it != neighbours.end(); ++it) {
    ret->push_back(PoseVertexWrapper_Ptr(new PoseVertexWrapper_Impl(*it)));
  }

  return ret;
}

bool FactorGraphFilter_Impl::testExistance(const PoseVertex* v) const {
  PoseMap::const_iterator it = _poses.find(v->getTimestamp());
  if (it->second == v) {
    return true;
  }
  return false;
}

void FactorGraphFilter_Impl::computeCovariances() {

  // iterate through the poses and collect the vertices for which I have to evaluate covariance

  list<PoseVertex *> active;
  for (PoseMapIterator it = _poses.begin(); it != _poses.end(); ++it) {

    PoseVertex *pose = it->second;
    PoseVertexMetadata *meta =
        static_cast<PoseVertexMetadata *>(pose->userData());

    if (meta->computeUncertainty == true && pose->fixed() == false) {
      active.push_back(pose);
    }
  }

  // iterate through the vertices and collect their tempIndex

  vector<pair<int, int> > blockIndices;
  for (list<PoseVertex *>::iterator it = active.begin(); it != active.end();
      ++it) {
    blockIndices.push_back(
        pair<int, int>((*it)->tempIndex(), (*it)->tempIndex()));
  }

  // iterate trough parameters and collect vertices for which covariances have
  // to be calculated

  set<g2o::OptimizableGraph::Vertex *> otherVertices;

  for (auto p_it = _params.begin(); p_it != _params.end(); ++p_it) {
    boost::shared_ptr<ParameterVerticesManager> p = p_it->second;
    if (p->fixed() == false && p->computeCovariance()) {
      for (auto v_it = p->_v.begin(); v_it != p->_v.end(); ++v_it) {
        g2o::OptimizableGraph::Vertex *v = v_it->second;
        if (v->tempIndex() >= 0) { //there might be parameters not involved in current estimation
          otherVertices.insert(v);

          blockIndices.push_back(
              pair<int, int>(v->tempIndex(), v->tempIndex()));
        }
      }
    }
  }

  /* ----------------------- TMP
   // Always compute covariance for parameters that are not fixed

   for (auto p_it = _params.begin(); p_it != _params.end(); ++p_it) {
   if (p_it->second->fixed() == false) {
   // it holds only for constant parameters (which are the only one available in bespin
   g2o::OptimizableGraph::Vertex *v = p_it->second->getVertices(0.0)->second;

   if (v->tempIndex() >= 0) { //there might be parameters not involved in current estimation
   otherVertices.insert(v);

   blockIndices.push_back(
   pair<int, int>(v->tempIndex(), v->tempIndex()));

   # 	  	ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
   GenericVertexInterface *gi = dynamic_cast<GenericVertexInterface *>(v);
   assert(gi != NULL);

   cerr << "[FactorGraphFilter] Info: computing covariance for "
   << p_it->first << "{"
   << ROAMutils::StringUtils::writeNiceTimestamp(gi->getTimestamp())
   << "}" << v->fixed() << " " << v->tempIndex() << endl;
   #		  endif
   }
   }
   }

   // ----------------------- */

// compute the marginals
  if (blockIndices.size() == 0) {
    // there are no marginals to compute
    return;
  }

  g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;
  _optimizer->computeMarginals(spinv, blockIndices);

// store the marginals into the vertices
// TODO: maybe it is possible to compute the marginals DIRECTLY into the vertices uncertainty storage
  for (list<PoseVertex *>::iterator it = active.begin(); it != active.end();
      ++it) {
    PoseVertex *pose = *it;
    pose->setUncertainty(*(spinv.block(pose->tempIndex(), pose->tempIndex())));
  }

// ----------------------- TMP
// do the same for the parameters

  for (auto v_it = otherVertices.begin(); v_it != otherVertices.end(); ++v_it) {
    (*v_it)->setUncertainty(
        (spinv.block((*v_it)->tempIndex(), (*v_it)->tempIndex()))->data());

#		ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    g2o::OptimizableGraph::Vertex *v = *v_it;
    GenericVertexInterface *gv = dynamic_cast<GenericVertexInterface *>(v);

    Eigen::Map<Eigen::MatrixXd> cov(v->uncertaintyData(), v->dimension(),
        v->dimension());

    cerr << "[FactorGraphFilter] Info: uncertainty for "
    << gv->getCategory() << ":" << endl << cov.format(CleanFmt)
    << endl;
#		endif
  }

// ----------------------- */

}

bool FactorGraphFilter_Impl::addMisalignmentGuard(const string& sensor) {
  GenericVertex<Eucl1DV> *v1 =
      static_cast<GenericVertex<Eucl1DV> *>(_params[sensor + "_qOSx"]->getVertices(
          0)->second);
  GenericVertex<Eucl1DV> *v2 =
      static_cast<GenericVertex<Eucl1DV> *>(_params[sensor + "_qOSy"]->getVertices(
          0)->second);
  GenericVertex<Eucl1DV> *v3 =
      static_cast<GenericVertex<Eucl1DV> *>(_params[sensor + "_qOSz"]->getVertices(
          0)->second);

  MisalignmentGuard *g = new MisalignmentGuard(v1, v2, v3);

  _optimizer->addPostIterationAction(g);

  return true;
}

double FactorGraphFilter_Impl::getWindowLenght() {
  if (_initialPoseSet == false) {
    return 0.0;
  }

  return getNewestPose_i()->getTimestamp() - getOldestPose_i()->getTimestamp();
}

MeasurementEdgeWrapperVector_Ptr FactorGraphFilter_Impl::handleDeferredMeasurements() {
  MeasurementEdgeWrapperVector_Ptr ret(new MeasurementEdgeWrapperVector);

// call with a back inserted iterator for this list
  list<GenericEdgeInterface *> inserted;

  handleDeferredMeasurements_i(back_inserter(inserted));

// for each edge, instantiate a wrapper and stuff it into ret
  for (list<GenericEdgeInterface *>::iterator it = inserted.begin();
      it != inserted.end(); ++it) {
    ret->push_back(
        MeasurementEdgeWrapper_Ptr(new MeasurementEdgeWrapper_Impl(*it)));
  }

  return ret;
}

ParameterWrapper_Ptr FactorGraphFilter_Impl::getParameterByName(
    const string& name) {

  ParameterVerticesManager *parameter = getParameterByName_i(name);

  if (parameter == NULL) {

#   ifdef DEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES
    cerr << "[FactorGraphFilter] Error: parameter " << name
    << " does not exist " << endl;
#		endif

    return ParameterWrapper_Ptr();

  } else {
    return ParameterWrapper_Ptr(new ParameterWrapper_Impl(parameter));
  }
}

ParameterVerticesManager* FactorGraphFilter_Impl::getParameterByName_i(
    const string& name) {
  auto parameter = _params.find(name);

  if (parameter == _params.end()) {
    return NULL;
  }

  return parameter->second.get();
}

string FactorGraphFilter_Impl::writeFactorGraph() {

  stringstream s;

  s << " --> PARAMETERS <--" << endl;

  for (auto pit = _params.begin(); pit != _params.end(); ++pit) {
    s << pit->first << (pit->second->fixed() ? " -- Fixed --:" : ":") << endl;

    for (auto vit = pit->second->_v.begin(); vit != pit->second->_v.end();
        ++vit) {

      GenericVertexInterface *gvi =
          dynamic_cast<GenericVertexInterface *>(vit->second);
      assert(gvi != NULL);

      s << "---- " << vit->second->id() << "["
          << ROAMutils::StringUtils::writeNiceTimestamp(gvi->getTimestamp())
          << "] " << (vit->second->fixed() ? " -- Fixed --:" : ":") << endl;

      for (auto eit = vit->second->edges().begin();
          eit != vit->second->edges().end(); ++eit) {
        s << "-------- " << writeEdge(*eit) << endl;
      }
    }
  }

  s << endl << " --> POSE GRAPH <--" << endl;
  for (PoseMapIterator pit = _poses.begin(); pit != _poses.end(); ++pit) {
    PoseVertex *v = pit->second;

    s << v->id() << "["
        << ROAMutils::StringUtils::writeNiceTimestamp(v->getTimestamp()) << "] "
        << (v->fixed() ? " -- Fixed --:" : ":") << endl;

    g2o::HyperGraph::EdgeSet::const_iterator weit;
    for (weit = v->edges().begin(); weit != v->edges().end(); ++weit) {

      s << "---- " << writeEdge(*weit) << endl;
    }
  }

  return s.str();
}

string FactorGraphFilter_Impl::writeEdge(g2o::HyperGraph::Edge * e) {
  stringstream s;

  GenericEdgeInterface *gei;
  BasePriorEdgeInterface *pei;
  GenericLinearConstraint *glc;

  if ((gei = dynamic_cast<GenericEdgeInterface *>(e)) != NULL) {
    s << gei->writeDebugInfo();
  } else if ((pei = dynamic_cast<BasePriorEdgeInterface *>(e)) != NULL) {
    s << pei->writeDebugInfo();
  } else if ((glc = dynamic_cast<GenericLinearConstraint *>(e)) != NULL) {
    s << glc->writeDebugInfo();
  } else {
    s << "Unknown (this is bad) ";
  }

  return s.str();
}

string FactorGraphFilter_Impl::writeFactorGraphToDot() {

  stringstream s;

  s << "graph {" << endl;
  s << "overlap=false;" << endl;

  for (PoseMapIterator pit = _poses.begin(); pit != _poses.end(); ++pit) {
    PoseVertex *v = pit->second;

    s << v->id() << "[label=\"" << "(" << v->id() << ") t="
        << ROAMutils::StringUtils::writeNiceTimestamp(v->getTimestamp())
        << (v->fixed() ? " Fixed" : "") << "\"];" << endl;

    g2o::HyperGraph::EdgeSet::const_iterator weit;
    for (weit = v->edges().begin(); weit != v->edges().end(); ++weit) {

      GenericEdgeInterface *e = dynamic_cast<GenericEdgeInterface *>(*weit);
      if (e != NULL) {
        s << e->getEdgeHash() << "[label=\"" << e->getCategory() << "\n"
            << ROAMutils::StringUtils::writeNiceTimestamp(e->getTimestamp())
            << "\", shape=box];";
        s << e->getEdgeHash() << "--" << v->id() << ";" << endl;
      }

      //TODO other information
      /*BasePriorEdgeInterface *p = dynamic_cast<BasePriorEdgeInterface *>(*weit);
       if (p != NULL) {
       s << "Prior ";
       }

       GenericLinearConstraint *glc =
       dynamic_cast<GenericLinearConstraint *>(*weit);
       if (glc != NULL) {
       s << glc->writeDebugInfo();
       }

       s << endl;*/
    }
  }

  s << "}" << endl;

  return s.str();
}

void FactorGraphFilter_Impl::setLowLevelLogging(bool lowLevelLogging,
    string folder) {
  if (lowLevelLogging == true) {
    if (_logger == NULL) {
      _logger = new ROAMlog::GraphLogger(folder, _optimizer);
    }
  }

  _lowLevelLogging = lowLevelLogging;
}

FactorGraphFilter_Impl::MisalignmentGuard::MisalignmentGuard(
    GenericVertex<Eucl1DV> *v1, GenericVertex<Eucl1DV> *v2,
    GenericVertex<Eucl1DV> *v3) :
    _v1(v1), _v2(v2), _v3(v3) {
}

g2o::HyperGraphAction* FactorGraphFilter_Impl::MisalignmentGuard::operator ()(
    const g2o::HyperGraph* graph,
    g2o::HyperGraphAction::Parameters* parameters) {

  double norm = pow(_v1->estimate()(0), 2) + pow(_v2->estimate()(0), 2)
      + pow(_v3->estimate()(0), 2);

  if (norm > 1.0) {
    _v1->estimate()(0) /= norm;
    _v2->estimate()(0) /= norm;
    _v3->estimate()(0) /= norm;
  }

  return this;
}

string FactorGraphFilter_Impl::writeVertexIdMap() {

  stringstream s;

  unsigned int cur = 0;

  for (auto it = _optimizer->indexMapping().begin();
      it != _optimizer->indexMapping().end(); ++it) {

    g2o::OptimizableGraph::Vertex *v = *it;
    GenericVertexInterface *gv = dynamic_cast<GenericVertexInterface *>(v);

    assert(gv != NULL);

    s << gv->getCategory() << "["
        << ROAMutils::StringUtils::writeNiceTimestamp(gv->getTimestamp())
        << "]: " << cur << " to " << cur + v->dimension() - 1 << endl;

    cur += v->dimension();
  }

  return s.str();
}

} /* namespace ROAMestimation */
