// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O__AIS_OPTIMIZABLE_GRAPH_HH_
#define G2O__AIS_OPTIMIZABLE_GRAPH_HH_

#include <deque>
#include <set>
#include <iostream>
#include <list>
#include <limits>
#include <cmath>

#include "openmp_mutex.h"
#include "hyper_graph.h"

namespace g2o {

class HyperGraphAction;
struct SolverProperty;

/**
 @addtogroup g2o
 */
/**
 This is an abstract class that represents one optimization
 problem.  It specializes the general graph to contain special
 vertices and edges.  The vertices represent parameters that can
 be optimized, while the edges represent constraints.  This class
 also provides basic functionalities to handle the backup/restore
 of portions of the vertices.
 */
struct OptimizableGraph: public HyperGraph {

  enum ActionType {
    AT_PREITERATION, AT_POSTITERATION, AT_NUM_ELEMENTS, // keep as last element
  };

  typedef std::set<HyperGraphAction*> HyperGraphActionSet;

  // forward declarations
  class Vertex;
  class Edge;

  /**
   * \brief data packet for a vertex. Extend this class to store in the vertices
   * the potential additional information you need (e.g. images, laser scans, ...).
   */
  struct Data: public HyperGraph::HyperGraphElement {
    virtual ~Data() {
    }
    ;
    //! read the data from a stream
    virtual bool read(std::istream& is) = 0;
    //! write the data to a stream
    virtual bool write(std::ostream& os) const = 0;
  };

  /**
   * \brief order vertices based on their ID
   */
  struct VertexIDCompare {
    bool operator()(const Vertex* v1, const Vertex* v2) const {
      return v1->id() < v2->id();
    }
  };

  /**
   * \brief order edges based on the internal ID, which is assigned to the edge in addEdge()
   */
  struct EdgeIDCompare {
    bool operator()(const Edge* e1, const Edge* e2) const {
      return e1->internalId() < e2->internalId();
    }
  };

  //! vector container for vertices
  typedef std::vector<OptimizableGraph::Vertex*> VertexContainer;
  //! vector container for edges
  typedef std::vector<OptimizableGraph::Edge*> EdgeContainer;

  /**
   * \brief A general case Vertex for optimization
   */
  class Vertex: public HyperGraph::Vertex {
  private:
    friend struct OptimizableGraph;
  public:
    Vertex();

    //! returns a deep copy of the current vertex
    virtual Vertex* clone() const;

    //! the user data associated with this vertex
    const Data* userData() const {
      return _userData;
    }
    Data* userData() {
      return _userData;
    }

    void setUserData(Data* obs) {
      _userData = obs;
    }

    virtual ~Vertex();

    //! sets the node to the origin (used in the multilevel stuff)
    virtual void setToOrigin() = 0;

    //! get the element from the hessian matrix
    virtual const double& hessian(int i, int j) const = 0;
    virtual double& hessian(int i, int j) = 0;
    virtual double hessianDeterminant() const = 0;
    virtual double* hessianData() = 0;

    /** maps the internal matrix to some external memory location */
    virtual void mapHessianMemory(double* d) = 0;

    /**
     * copies the b vector in the array b_
     * @return the number of elements copied
     */
    virtual int copyB(double* b_) const = 0;

    //! get the b vector element
    virtual const double& b(int i) const = 0;
    virtual double& b(int i) = 0;
    //! return a pointer to the b vector associated with this vertex
    virtual double* bData() = 0;

    /**
     * set the b vector part of this vertex to zero
     */
    virtual void clearQuadraticForm() = 0;

    /**
     * updates the current vertex with the direct solution x += H_ii\b_ii
     * @return the determinant of the inverted hessian
     */
    virtual double solveDirect(double lambda = 0) = 0;

    /**
     * sets the initial estimate from an array of double
     * @return true on success
     */
    virtual bool setEstimateData(const double* estimate);

    /**
     * writes the estimater to an array of double
     * @returns true on success
     */
    virtual bool getEstimateData(double* estimate) const;

    /**
     * access the internal memory of the estimate
     * @returns NULL on failure
     */
    virtual double * accessEstimateData();

    /**
     * returns the dimension of the extended representation used by get/setEstimate(double*)
     * -1 if it is not supported
     */
    virtual int estimateDimension() const;

    /**
     * sets the initial estimate from an array of double
     * @return true on success
     */
    virtual bool setMinimalEstimateData(const double* estimate);

    /**
     * writes the estimater to an array of double
     * @returns true on success
     */
    virtual bool getMinimalEstimateData(double* estimate) const;

    /**
     * returns the dimension of the extended representation used by get/setEstimate(double*)
     * -1 if it is not supported
     */
    virtual int minimalEstimateDimension() const;

    //! backup the position of the vertex to a stack
    virtual void push() = 0;

    //! restore the position of the vertex by retrieving the position from the stack
    virtual void pop() = 0;

    //! pop the last element from the stack, without restoring the current estimate
    virtual void discardTop() = 0;

    //! return the stack size
    virtual int stackSize() const = 0;

    //! update the position of the node from the parameters in v
    virtual void oplus(double* v) = 0;

    //! temporary index of this node in the parameter vector obtained from linearization
    int tempIndex() const {
      return _tempIndex;
    }
    //! set the temporary index of the vertex in the parameter blocks
    void setTempIndex(int ti) {
      _tempIndex = ti;
    }

    //! true => this node is fixed during the optimization
    bool fixed() const {
      return _fixed;
    }
    //! true => this node should be considered fixed during the optimization
    void setFixed(bool fixed) {
      _fixed = fixed;
    }

    //! true => this node is marginalized out during the optimization
    bool marginalized() const {
      return _marginalized;
    }
    //! true => this node should be marginalized out during the optimization
    void setMarginalized(bool marginalized) {
      _marginalized = marginalized;
    }

    //! dimension of the estimated state belonging to this node
    int dimension() const {
      return _dimension;
    }

    //! sets the id of the node in the graph be sure that the graph keeps consistent after changing the id
    void setId(int id) {
      _id = id;
    }

    //! set the row of this vertex in the Hessian
    void setColInHessian(int c) {
      _colInHessian = c;
    }
    //! get the row of this vertex in the Hessian
    int colInHessian() const {
      return _colInHessian;
    }

    const OptimizableGraph* graph() const {
      return _graph;
    }

    //! sets the covariance/information of the node, used internally
    virtual void setUncertainty(double* c) = 0;
    //! return the pointer to the uncertainty (marginal covariance)
    virtual double* uncertaintyData() = 0;

    /**
     * lock for the block of the hessian and the b vector associated with this vertex, to avoid
     * race-conditions if multi-threaded.
     */
    void lockQuadraticForm() {
      _quadraticFormMutex.lock();
    }
    /**
     * unlock the block of the hessian and the b vector associated with this vertex
     */
    void unlockQuadraticForm() {
      _quadraticFormMutex.unlock();
    }

    //! read the vertex from a stream, i.e., the internal state of the vertex
    virtual bool read(std::istream& is) = 0;
    //! write the vertex to a stream
    virtual bool write(std::ostream& os) const = 0;

  protected:
    OptimizableGraph* _graph;
    Data* _userData;
    int _tempIndex;
    bool _fixed;
    bool _marginalized;
    int _dimension;
    int _colInHessian;
    OpenMPMutex _quadraticFormMutex;
  };

  class Edge: public HyperGraph::Edge {
  private:
    friend struct OptimizableGraph;
  public:
    Edge();
    virtual Edge* clone() const;

    //! the user data associated with this edge
    const Data* userData() const {
      return _userData;
    }
    Data* userData() {
      return _userData;
    }

    void setUserData(Data* obs) {
      _userData = obs;
    }

    /**
     * updates all temporaries which depend on connected vertices value
     */

    virtual void afterVertexUpdate();

    // computes the error of the edge and stores it in an internal structure
    virtual void computeError() = 0;

    //! sets the measurement from an array of double
    //! @returns true on success
    virtual bool setMeasurementData(const double* m);

    //! writes the measurement to an array of double
    //! @returns true on success
    virtual bool getMeasurementData(double* m) const;

    /**
     * access the internal memory of the measurement
     * @returns NULL on failure
     */
    virtual double * accessMeasurementData();

    //! returns the dimension of the measurement in the extended representation which is used
    //! by get/setMeasurement;
    virtual int measurementDimension() const;

    /**
     * sets the estimate to have a zero error, based on the current value of the state variables
     * returns false if not supported.
     */
    virtual bool setMeasurementFromState();

    /**
     * robustify the error of the edge using an robust kernel/M-estimator
     * this is only called if robustKernel==true
     */
    virtual void robustifyError() = 0;

    //! if true, error will be robustifed (not for computing Jacobians)
    bool robustKernel() const {
      return _robustKernel;
    }
    void setRobustKernel(bool rk) {
      _robustKernel = rk;
    }

    //! width of the robust huber kernel
    double huberWidth() const {
      return _huberWidth;
    }
    void setHuberWidth(double hw) {
      _huberWidth = hw;
    }

    //! return the weight used in the edge robustification so that original error can be computed again
    double currentHuberWeight() const {
    	return _currentHuberWeight;
    }

    //! returns the error vector cached after calling the computeError;
    virtual const double* errorData() const = 0;
    virtual double* errorData() = 0;

    //! returns the memory of the information matrix, usable for example with a Eigen::Map<MatrixXd>
    virtual const double* informationData() const = 0;
    virtual double* informationData() = 0;

    //! returns the memory of the jacobian of the error wrt ith vertex
    virtual const double* jacobianData(int i) const = 0;

    //! computes the chi2 based on the cached error value, only valid after computeError has been called.
    virtual double chi2() const = 0;

    /**
     * Linearizes the constraint in the edge.
     * Makes side effect on the vertices of the graph by changing
     * the parameter vector b and the hessian blocks ii and jj.
     * The off diagoinal block is accesed via _hessian.
     */
    virtual void constructQuadraticForm() = 0;

    /**
     * maps the internal matrix to some external memory location,
     * you need to provide the memory before calling constructQuadraticForm
     * @param d the memory location to which we map
     * @param i index of the vertex i
     * @param j index of the vertex j (j > i, upper triangular fashion)
     * @param rowMajor if true, will write in rowMajor order to the block. Since EIGEN is columnMajor by default, this results in writing the transposed
     */
    virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor) = 0;

    /**
     * Linearizes the constraint in the edge in the manifold space, and stores
     * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj (see base_edge).
     */
    virtual void linearizeOplus() = 0;

    /** set the estimate of the to vertex, based on the estimate of the from vertices in the edge. */
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
        OptimizableGraph::Vertex* to) = 0;

    /**
     * override in your class if it's possible to initialize the vertices in certain combinations.
     * The return value may correspond to the cost for initiliaizng the vertex but should be positive if
     * the initialization is possible and negative if not possible.
     */
    virtual double initialEstimatePossible(
        const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
      (void) from;
      (void) to;
      return -1.;
    }

    //! returns the level of the edge
    int level() const {
      return _level;
    }
    //! sets the level of the edge
    void setLevel(int l) {
      _level = l;
    }

    //! returns the dimensions of the error function
    int dimension() const {
      return _dimension;
    }

    virtual Vertex* createFrom() {
      return 0;
    }
    virtual Vertex* createTo() {
      return 0;
    }

    //! read the vertex from a stream, i.e., the internal state of the vertex
    virtual bool read(std::istream& is) = 0;
    //! write the vertex to a stream
    virtual bool write(std::ostream& os) const = 0;

    //! the internal ID of the edge
    long long internalId() const {
      return _internalId;
    }

  protected:
    Data* _userData;
    int _dimension;
    int _level;
    bool _robustKernel;
    double _huberWidth;
    double _currentHuberWeight;
    long long _internalId;

    /** Square root of huber cost function devided by delta
     *
     *  Let delta be the generalized 2-norm of the error e, thus
     *  delta = sqrt(e*Omega*e).
     *
     *  Let rho be the Huber cost function,
     *  rho(x) =  if |x|<b : x^2 | else: 2b|x|-b^2
     *  (Thus, b is the "width" of the quadratic component.)
     *
     *  This function computes "sqrt(rho(delta))/delta" which can
     *  be used as a weight to robustify the error e.
     *
     *  For details: See Hartley, Zisserman: "Multiple View Geometry in
     Computer Vision", 2nd edition, 2003, pp.616.
     */

    inline double sqrtOfHuberByNrm(double delta, double b) const {
      if (delta < b)
        return 1;
      return sqrt(2 * b * fabs(delta) - b * b) / delta;
    }

  };

  //! returns the vertex number <i>id</i> appropriately casted
  inline Vertex* vertex(int id) {
    return reinterpret_cast<Vertex*>(HyperGraph::vertex(id));
  }

  //! returns the vertex number <i>id</i> appropriately casted
  inline const Vertex* vertex(int id) const {
    return reinterpret_cast<const Vertex*>(HyperGraph::vertex(id));
  }

  //! empty constructor
  OptimizableGraph();
  virtual ~OptimizableGraph();

  //! adds all edges and vertices of the graph <i>g</i> to this graph.
  void addGraph(OptimizableGraph* g);

  /**
   * adds a new vertex. The new vertex is then "taken".
   * @return false if a vertex with the same id as v is already in the graph, true otherwise.
   */
  virtual bool addVertex(OptimizableGraph::Vertex* v, Data* userData = 0);

  /**
   * adds a new edge.
   * The edge should point to the vertices that it is connecting (setFrom/setTo).
   * @return false if the insertion does not work (incompatible types of the vertices/missing vertex). true otherwise.
   */
  virtual bool addEdge(OptimizableGraph::Edge* e);

  //! returns the chi2 of the current configuration
  double chi2() const;

  //! return the maximum dimension of all vertices in the graph
  int maxDimension() const;

  /**
   * iterates over all vertices and returns a set of all the vertex dimensions in the graph
   */
  std::set<int> dimensions() const;

  /**
   * carry out n iterations
   * @return the number of performed iterations
   */
  virtual int optimize(int iterations, bool online = false);

  //! called at the beginning of an iteration (argument is the number of the iteration)
  virtual void preIteration(int);
  //! called at the end of an iteration (argument is the number of the iteration)
  virtual void postIteration(int);

  //! add an action to be executed before each iteration
  bool addPreIterationAction(HyperGraphAction* action);
  //! add an action to be executed after each iteration
  bool addPostIterationAction(HyperGraphAction* action);

  //! remove an action that should no longer be execured before each iteration
  bool removePreIterationAction(HyperGraphAction* action);
  //! remove an action that should no longer be execured after each iteration
  bool removePostIterationAction(HyperGraphAction* action);

  //! push the estimate of all variables onto a stack
  virtual void push();
  //! pop (restore) the estimate of all variables from the stack
  virtual void pop();
  //! discard the last backup of the estimate for all variables by removing it from the stack
  virtual void discardTop();

  //! load the graph from a stream. Uses the Factory singleton for creating the vertices and edges.
  virtual bool load(std::istream& is, bool createEdges = true);
  bool load(const char* filename, bool createEdges = true);
  //! save the graph to a stream. Again uses the Factory system.
  virtual bool save(std::ostream& os, int level = 0) const;
  //! function provided for convenience, see save() above
  bool save(const char* filename, int level = 0) const;

  //! save a subgraph to a stream. Again uses the Factory system.
  bool saveSubset(std::ostream& os, HyperGraph::VertexSet& vset, int level = 0);

  //! save a subgraph to a stream. Again uses the Factory system.
  bool saveSubset(std::ostream& os, HyperGraph::EdgeSet& eset);

  //! push the estimate of a subset of the variables onto a stack
  virtual void push(HyperGraph::VertexSet& vset);
  //! pop (restore) the estimate a subset of the variables from the stack
  virtual void pop(HyperGraph::VertexSet& vset);
  //! ignore the latest stored element on the stack, remove it from the stack but do not restore the estimate
  virtual void discardTop(HyperGraph::VertexSet& vset);

  //! fixes/releases a set of vertices
  virtual void setFixed(HyperGraph::VertexSet& vset, bool fixed);

  /**
   * set the renamed types lookup from a string, format is for example:
   * VERTEX_CAM=VERTEX_SE3:EXPMAP,EDGE_PROJECT_P2MC=EDGE_PROJECT_XYZ:EXPMAP
   * This will change the occurance of VERTEX_CAM in the file to VERTEX_SE3:EXPMAP
   */
  void setRenamedTypesFromString(const std::string& types);

  /**
   * test whether a solver is suitable for optimizing this graph.
   * @param solverProperty the solver property to evaluate.
   * @param vertDims should equal to the set returned by dimensions() to avoid re-evaluating.
   */
  bool isSolverSuitable(const SolverProperty& solverProperty,
      const std::set<int>& vertDims = std::set<int>()) const;

protected:
  std::list<Vertex*> _taintedList, ///< this is the list of higher level vertices whose connectivity and information has to be recomputed
      _partialList; ///< this is the list of higher level vertices which need to be deleted.
  OptimizableGraph* _upperGraph, *_lowerGraph;
  std::map<std::string, std::string> _renamedTypesLookup;
  long long _nextEdgeId;
  std::vector<HyperGraphActionSet> _graphActions;

  // do not watch this. To be removed soon, or integrated in a nice way
  bool _edge_has_id;
};

/**
 @}
 */

} // end namespace

#endif
