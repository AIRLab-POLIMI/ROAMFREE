/*
 Copyright (c) 2013-2016 Politecnico di Milano.
 All rights reserved. This program and the accompanying materials
 are made available under the terms of the GNU Lesser Public License v3
 which accompanies this distribution, and is available at
 https://www.gnu.org/licenses/lgpl.html

 Contributors:
 Davide A. Cucci (davide.cucci@epfl.ch)
 */

/* FactorGraphFilterImpl.h
 *
 *  Created on: Mar 12, 2014
 *      Author: davide
 */

#ifndef FACTORGRAPHFILTERIMPL_H_
#define FACTORGRAPHFILTERIMPL_H_

#include <map>
#include <set>
#include <string>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "FactorGraphFilter.h"

#include "types.h"
#include "Enums.h"

#include "GenericVertex.h"
#include "ParameterVerticesManager.h"

#include "PoseVertexWrapperImpl.h"
#include "MeasurementEdgeWrapperImpl.h"

#include "GenericEdgeInterface.h"

#include "Chi2TestAction.h"

namespace ROAMlog {
class GraphLogger;
}

namespace ROAMreject {
class RejectManager;
}

using namespace ROAMfunctions;

namespace ROAMestimation {

class SpatialIndex;
class SE3PriorEdge;

class FactorGraphFilter_Impl: public FactorGraphFilter {

  public:

    FactorGraphFilter_Impl();

    /* --------------------------- SOLVER LEVEL METHODS ---------------------------- */

    void setSolverMethod(SolverMethod method);

    void setChi2Threshold(double threshold);

    PoseVertexWrapper_Ptr setInitialPose(const Eigen::VectorXd &x0, double t);

    void setDeadReckoning(bool deadReckoning);

    void setLowLevelLogging(bool lowLevelLogging, std::string folder =
        "/tmp/roamfree");
    void setWriteGraph(bool writeGraph);
    void setWriteHessianStructure(bool writeHessianStructure);

    double getWindowLenght();

    double getChi2();

    std::map<std::string, EstimationStats> getEstimationStats();

    void writeFinalHessian();

    bool addPostIterationAction(g2o::HyperGraphAction* action);

    /* --------------------------- SENSOR LEVEL METHODS ---------------------------- */

    bool addSensor(const std::string &name, MeasTypes type, bool isMaster,
        bool isSequential);

    bool setSensorFrame(const std::string &name, const Eigen::VectorXd &s);

    bool shareSensorFrame(const std::string &from, const std::string &to);

    bool shareParameter(const std::string &from, const std::string &to);

    bool setRobustKernel(const std::string &sensor, bool enabled,
        double huberWidth);

    /* --------------------------- PARAMETER LEVEL METHODS ---------------------------- */

    ParameterWrapper_Ptr addConstantParameter(ParameterTypes type,
        const std::string &name, double t, const Eigen::VectorXd &x0,
        bool isFixed);
    ParameterWrapper_Ptr addConstantParameter(ParameterTypes type,
        const std::string &name, const Eigen::VectorXd &x0, bool isFixed);
    ParameterWrapper_Ptr addConstantParameter(const std::string &sensor,
        double x0, bool isFixed);
    ParameterWrapper_Ptr poseVertexAsParameter(PoseVertexWrapper_Ptr pose,
        const std::string &name);

    ParameterWrapper_Ptr addLimitedBandwithParameter(ParameterTypes type,
        const std::string &name, const Eigen::VectorXd &x0, bool isFixed,
        double bandwidth, int a);
    ParameterWrapper_Ptr addLimitedBandwithParameter(const std::string &name,
        double x0, bool isFixed, double bandwidth, int a);

    ParameterWrapper_Ptr addLinearlyInterpolatedParameter(ParameterTypes type,
        const std::string &name, const Eigen::VectorXd &x0, bool isFixed,
        double spacing);
    ParameterWrapper_Ptr addLinearlyInterpolatedParameter(
        const std::string &name, double x0, bool isFixed, double spacing);

    ParameterWrapper_Ptr addParameterBlender(ParameterTypes type,
        const std::string &name, ParameterWrapperVector_Ptr toblend);

    ParameterWrapper_Ptr getParameterByName(const std::string &name);
    
    /* --------------------------- POSES AND EDGES LEVEL METHODS ---------------------- */

    PoseVertexWrapper_Ptr addPose(double t);

    PoseVertexWrapper_Ptr addInterpolatingPose(double t, ParameterWrapper_Ptr delayParam, 
        const Eigen::MatrixXd &pseudoObsCov);

    MeasurementEdgeWrapper_Ptr addMeasurement(const std::string& sensorName,
        double timestamp, const Eigen::VectorXd &z, const Eigen::MatrixXd &cov,
        PoseVertexWrapper_Ptr v2, PoseVertexWrapper_Ptr v1 =
            PoseVertexWrapper_Ptr(), PoseVertexWrapper_Ptr v0 =
            PoseVertexWrapper_Ptr());

    bool removeMeasurement(MeasurementEdgeWrapper_Ptr edge);

    MeasurementEdgeWrapperVector_Ptr addSequentialMeasurement(
        const std::string& sensorName, double timestamp,
        const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

    MeasurementEdgeWrapperVector_Ptr handleDeferredMeasurements();

    bool marginalizeNodes(std::vector<PoseVertexWrapper *> &nodes);

    bool marginalizeOldNodes(double l);

    bool forgetOldNodes(double l);

    /* --------------------------- PRIOR CONTROL METHODS ------------------------------ */

    MeasurementEdgeWrapper_Ptr addPriorOnPose(PoseVertexWrapper_Ptr pose,
        const Eigen::VectorXd &x0, const Eigen::MatrixXd &cov);

    MeasurementEdgeWrapper_Ptr addPriorOnConstantParameter(PriorEdgeTypes type,
        const std::string &name, const Eigen::VectorXd &x0,
        const Eigen::MatrixXd &cov);

    MeasurementEdgeWrapper_Ptr addPriorOnTimeVaryingParameter(
        PriorEdgeTypes type, const std::string &name, double t,
        const Eigen::VectorXd &x0, const Eigen::MatrixXd &cov);

    /* --------------------------- POSES GETTERS -------------------------------------- */

    PoseVertexWrapper_Ptr getNearestPoseByTimestamp(double t, bool onlyBefore =
        false);

    std::pair<PoseVertexWrapper_Ptr,PoseVertexWrapper_Ptr> getNearestTwoPoseByTimestamp(
    double t);

    PoseVertexWrapperVector_Ptr getNeighbourPosesByDistance(
        PoseVertexWrapper_Ptr sample, double d);

    PoseVertexWrapper_Ptr getNewestPose();

    PoseVertexWrapper_Ptr getOldestPose();

    PoseVertexWrapper_Ptr getNthPose(int n);

    PoseVertexWrapper_Ptr getNthOldestPose(int n);

    /* --------------------------- ESTIMATION CONTROM METHODS ------------------------- */

    bool estimate(int nIterations);

    bool estimate(PoseVertexWrapperVector poses, int nIterations);
    
    void computeCrossCovariances();
    /* --------------------------- OTHER STUFF ---------------------------------------- */

    ~FactorGraphFilter_Impl();

  protected:

    /* --------------------------- STUFF FOR SOLVER -------------------------------- */

    void initSolver();

    bool _deadReckoning; //!< if we have full information on pose wrt world

    void handlePriorsOnOldestPose(); //!< manage the prior on the oldest pose in the graph
    void updatePosesAndEdgesMetadata(); //! after an estimation, set to false all the flags

    void computeCovariances();

    bool _stopFlag;
    Chi2TestAction *_stopAction;
    g2o::AutoIDSparseOptimizer *_optimizer;

    typedef PoseMap::iterator PoseMapIterator;

    PoseMap _poses; //!< the set of all the poses

    bool _initialPoseSet; //<! if the initial pose has been set or not

    bool _lowLevelLogging, _writeGraph, _writeHessianStructure;
    ROAMlog::GraphLogger *_logger; //!< the object which handles low level logging
    std::string _logFolder;

    SpatialIndex *_spatialIndex; //!< the object which maintains spatial informations about poses.    

    /* --------------------------- STUFF FOR SENSORS ------------------------------- */

    //! descriptor for a logical sensor
    struct Sensor {
        std::string name;

        MeasTypes type;
        bool isMaster;
        bool isSequential;

        bool robustified; //!< sensor level edge roubstification
        double huberWidth;

        int order; //!< the order of the sensor edge, 0 = pose/orientation only, 1 -> also speeds, 2 -> also accelerations

        GenericVertex<SE3V> *last, *secondlast; //!< these are kept by the addSequentialMeasurement and are ignored by the lower level addMeasurement method
    };

    std::map<std::string, struct Sensor> _sensors; //!< collection for the sensor descriptors

    /* --------------------------- STUFF FOR PARAMETERS ------------------------------- */

    //! collection for the parameter descriptors
    std::map<std::string, boost::shared_ptr<ParameterVerticesManager> > _params;
    

    /* --------------------------- STUFF FOR POSES AND MEASUREMENTS ------------------- */

    // these are some internal methods which do not return wrappers

    PoseVertex *addPose_i(double t);

    PoseVertex *addInterpolatingPose_i(double t, ParameterVerticesManager *delayParam, const Eigen::MatrixXd &pseudoObsCov);

    PoseVertex *getNewestPose_i();
    PoseVertex *getOldestPose_i();

    int _lastReturnedN_fromBack; // two variables to speed up calls of getNthPose_i
    PoseMapIterator _lastReturnedPose_fromBack;
    PoseVertex *getNthPose_i(int n);

    int _lastReturnedN_fromFront; // two variables to speed up calls of getNthPose_i
    PoseMapIterator _lastReturnedPose_fromFront;
    PoseVertex *getNthOldestPose_i(int n);

    PoseVertex *getNearestPoseByTimestamp_i(double t, bool onlyBefore = false);
    std::pair<PoseVertex*,PoseVertex*> getNearestTwoPoseByTimestamp_i(double t);

    bool checkSensorOrder(const struct Sensor &sensor, const PoseVertex *v2,
        const PoseVertex *v1, const PoseVertex *v0);

    GenericEdgeInterface *addMeasurement_i(struct Sensor &sensor,
        double timestamp, const Eigen::VectorXd &z, const Eigen::MatrixXd &cov,
        PoseVertex *v2, PoseVertex *v1 = NULL, PoseVertex *v0 = NULL);

    template<typename OutputIterator>
    void addMasterSequentialMeasurement_i(OutputIterator outiter,
        struct Sensor &sensor, double timestamp, const Eigen::VectorXd &z,
        const Eigen::MatrixXd &cov);

    GenericEdgeInterface *addNonMasterSequentialMeasurement_i(
        struct Sensor &sensor, double timestamp, const Eigen::VectorXd &z,
        const Eigen::MatrixXd &cov);

    ParameterVerticesManager *getParameterByName_i(const std::string &name);

    SE3PriorEdge *addPriorOnPose_i(PoseVertex *p, const Eigen::VectorXd &x0,
        const Eigen::MatrixXd &cov);

    bool marginalizeNodes_i(std::set<g2o::HyperGraph::Vertex *> &nodeSet);

    bool forgetNodes_i(std::set<g2o::HyperGraph::Vertex *> &nodeSet);

    bool estimate_i(g2o::HyperGraph::EdgeSet &eset, int nIterations);

    bool testExistance(const PoseVertex *v) const;

    //! descriptor to contain a measurement which cannot be inserted now
    struct Measurement {
        struct Sensor &sensor;
        double tstamp;
        boost::shared_ptr<Eigen::VectorXd> z;
        boost::shared_ptr<Eigen::MatrixXd> cov;

        inline bool operator<(const struct Measurement& rhs) const {
          return tstamp < rhs.tstamp;
        }

        Measurement(struct Sensor &sensor, double timestamp,
            const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);
    };

    //! container for deferred measurements
    std::multiset<struct Measurement> _deferred;

    void deferMeasurement(struct Sensor &sensor, double t,
        const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

    template<typename OutputIterator>
    void handleDeferredMeasurements_i(OutputIterator oiter);

    /* --------------------------- STUFF FOR DEBUG ------------------------------------ */

    std::string writeFactorGraph();
    std::string writeVertexIdMap(); // for each vertex it writes its indices in the hessian matrix
    std::string writeEdge(g2o::HyperGraph::Edge *e);

    /* --------------------------- FRIENDSHIP ----------------------------------------- */

  public:
    friend class ROAMreject::RejectManager;

};

} /* namespace ROAMestimation */

#endif /* FACTORGRAPHFILTERIMPL_H_ */
