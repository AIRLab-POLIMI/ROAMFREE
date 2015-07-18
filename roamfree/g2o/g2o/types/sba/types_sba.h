// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef G2O_SBA_TYPES
#define G2O_SBA_TYPES

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/math_groups/sbacam.h"
#include <Eigen/Geometry>
#include <iostream>

namespace g2o {

  using namespace Eigen;

/**
 * \brief Vertex encoding the intrinsics of the camera fx, fy, cx, xy, baseline;
 */

class VertexIntrinsics : public BaseVertex<4, Matrix<double, 5, 1> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexIntrinsics();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
      
    virtual void setToOrigin() {
      _estimate << 1., 1., 0.5, 0.5, 0.1;
    }
      
    virtual void oplus(double* update)
    {
      _estimate.head<4>() += Vector4d(update);
    }
 };

/**
 * \brief SBACam Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 * qw is assumed to be positive, otherwise there is an ambiguity in qx,qy,qz as a rotation
 */


  class VertexCam : public BaseVertex<6, SBACam>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexCam();
      
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOrigin() {
      _estimate = SBACam();
    }
      
    virtual void oplus(double* update)
    {
      /* if (_intrinsics){ */
      /* 	_estimate.setKcam(_intrinsics->estimate()[0], */
      /* 			  _intrinsics->estimate()[1], */
      /* 			  _intrinsics->estimate()[2], */
      /* 			  _intrinsics->estimate()[3], */
      /* 			  _intrinsics->estimate()[4]); */
      /* } */
      Vector6d v;
      for (int i=0; i<6; i++) 
	v[i]=update[i];
      _estimate.update(v);
    }

    VertexIntrinsics* _intrinsics;
 };

/**
 * \brief Point vertex, XYZ
 */
 class VertexPointXYZ : public BaseVertex<3, Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    VertexPointXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOrigin() {
      _estimate.fill(0.);
    }

    virtual void oplus(double* update_)
    {

      Vector3d update;
      for (int i=0; i<3; i++)
        update[i]=update_[i];

      _estimate += update;
    }


 protected:
};


// monocular projection
// first two args are the measurement type, second two the connection classes
 class EdgeProjectP2MC : public  BaseBinaryEdge<2, Vector2d, VertexPointXYZ, VertexCam> 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectP2MC();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {
      // from <Point> to <Cam>
      const VertexPointXYZ *point = static_cast<const VertexPointXYZ*>(_vertices[0]);
      const VertexCam *cam = static_cast<const VertexCam*>(_vertices[1]);

      // calculate the projection
      const Vector3d &pt = point->estimate();
      Vector4d ppt(pt(0),pt(1),pt(2),1.0);
      Vector3d p = cam->estimate().w2i * ppt;
      Vector2d perr;
      perr = p.head<2>()/p(2);
      //      std::cout << std::endl << "CAM   " << cam->estimate() << std::endl;
      //      std::cout << "POINT " << pt.transpose() << std::endl;
      //      std::cout << "PROJ  " << p.transpose() << std::endl;
      //      std::cout << "CPROJ " << perr.transpose() << std::endl;
      //      std::cout << "MEAS  " << _measurement.transpose() << std::endl;

      // error, which is backwards from the normal observed - calculated
      // _measurement is the measured projection
      _error = perr - _measurement;
      // std::cerr << _error.x() << " " << _error.y() <<  " " << chi2() << std::endl;
    }

    // jacobian
    virtual void linearizeOplus();

};

// stereo projection
// first two args are the measurement type, second two the connection classes
 class EdgeProjectP2SC : public  BaseBinaryEdge<3, Vector3d, VertexPointXYZ, VertexCam>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectP2SC();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {
      // from <Point> to <Cam>
      const VertexPointXYZ *point = static_cast<const VertexPointXYZ*>(_vertices[0]);
      VertexCam *cam = static_cast<VertexCam*>(_vertices[1]);

      // calculate the projection
      Vector3d kp;
      Vector4d pt;
      pt.head<3>() = point->estimate();
      pt(3) = 1.0;
      SBACam &nd = cam->estimate();
      nd.setTransform();
      nd.setProjection();
      nd.setDr();

      Vector3d p1 = nd.w2i * pt; 
      Vector3d p2 = nd.w2n * pt; 
      Vector3d pb(nd.baseline,0,0);

      double invp1 = 1.0/p1(2);
      kp.head<2>() = p1.head<2>()*invp1;

      // right camera px
      p2 = nd.Kcam*(p2-pb);
      kp(2) = p2(0)/p2(2);

      // std::cout << std::endl << "CAM   " << cam->estimate() << std::endl; 
      // std::cout << "POINT " << pt.transpose() << std::endl; 
      // std::cout << "PROJ  " << p1.transpose() << std::endl; 
      // std::cout << "PROJ  " << p2.transpose() << std::endl; 
      // std::cout << "CPROJ " << kp.transpose() << std::endl; 
      // std::cout << "MEAS  " << _measurement.transpose() << std::endl; 

      // error, which is backwards from the normal observed - calculated
      // _measurement is the measured projection
      _error = kp - _measurement;
    }

    // jacobian
    virtual void linearizeOplus();

};

// monocular projection with parameter calibration
// first two args are the measurement type, second two the connection classes
 class EdgeProjectP2MC_Intrinsics : public  BaseMultiEdge<2, Vector2d> 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeProjectP2MC_Intrinsics();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {
      // from <Point> to <Cam>, the intrinsics in KCam should be already set!
      const VertexPointXYZ *point = static_cast<const VertexPointXYZ*>(_vertices[0]);
      VertexCam *cam = static_cast<VertexCam*>(_vertices[1]);
      // calculate the projection
      const Vector3d &pt = point->estimate();
      Vector4d ppt(pt(0),pt(1),pt(2),1.0);
      Vector3d p = cam->estimate().w2i * ppt;
      Vector2d perr = p.head<2>()/p(2);
      _error = perr - _measurement;
    }

    // jacobian
    virtual void linearizeOplus();

};


/**
 * \brief 3D edge between two SBAcam
 */
 class EdgeSBACam : public BaseBinaryEdge<6, SE3Quat, VertexCam, VertexCam>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSBACam();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexCam* v1 = dynamic_cast<const VertexCam*>(_vertices[0]);
      const VertexCam* v2 = dynamic_cast<const VertexCam*>(_vertices[1]);
      SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error[0]=delta.translation().x();
      _error[1]=delta.translation().y();
      _error[2]=delta.translation().z();
      _error[3]=delta.rotation().x();
      _error[4]=delta.rotation().y();
      _error[5]=delta.rotation().z();
    }

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
};


/**
 * \brief edge between two SBAcam that specifies the distance between them
 */
 class EdgeSBAScale : public BaseBinaryEdge<1, double, VertexCam, VertexCam>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSBAScale();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexCam* v1 = dynamic_cast<const VertexCam*>(_vertices[0]);
      const VertexCam* v2 = dynamic_cast<const VertexCam*>(_vertices[1]);
      Vector3d dt=v2->estimate().translation()-v1->estimate().translation();
      _error[0] = _measurement - dt.norm();
    }
    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* to_);
};



} // end namespace

#endif // SBA_TYPES
