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

#include "types_sba.h"
#include <iostream>

#include "g2o/core/factory.h"

namespace g2o {

  using namespace std;
  void __attribute__ ((constructor)) init_sba_types(void)
  {
    //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;
    Factory* factory = Factory::instance();

    factory->registerType("VERTEX_CAM", new HyperGraphElementCreator<VertexCam>);
    factory->registerType("VERTEX_XYZ", new HyperGraphElementCreator<VertexPointXYZ>);
    factory->registerType("VERTEX_INTRINSICS", new HyperGraphElementCreator<VertexIntrinsics>);

    factory->registerType("EDGE_PROJECT_P2MC", new HyperGraphElementCreator<EdgeProjectP2MC>);
    factory->registerType("EDGE_PROJECT_P2MC_INTRINSICS", new HyperGraphElementCreator<EdgeProjectP2MC_Intrinsics>);
    factory->registerType("EDGE_PROJECT_P2SC", new HyperGraphElementCreator<EdgeProjectP2SC>);
    factory->registerType("EDGE_CAM", new HyperGraphElementCreator<EdgeSBACam>);
    factory->registerType("EDGE_SCALE", new HyperGraphElementCreator<EdgeSBAScale>);
  }

  // constructor
  VertexIntrinsics::VertexIntrinsics() 
  {
    estimate() << 1. , 1. , .5 , .5 , .1;
  }

  bool VertexIntrinsics::read(std::istream& is)
  {
    for (int i=0; i<5; i++)
      is >> estimate()[i];
    return true;
  }

  bool VertexIntrinsics::write(std::ostream& os) const
  {
    for (int i=0; i<5; i++)
      os << estimate()[i] << " ";
    return os.good();
  }

  // constructor
  VertexCam::VertexCam() 
  {
    _intrinsics = 0;
  }

  bool VertexCam::read(std::istream& is)
  {
    // first the position and orientation (vector3 and quaternion)
    Vector3d t;
    for (int i=0; i<3; i++){
      is >> t[i];
    }
    Vector4d rc;
    for (int i=0; i<4; i++) {
      is >> rc[i];
    }
    Quaterniond r;
    r.coeffs() = rc;
    r.normalize();


    // form the camera object
    SBACam cam(r,t);

    // now fx, fy, cx, cy, baseline
    double fx, fy, cx, cy, tx;

    // try to read one value
    is >>  fx;
    if (is.good()) {
      is >>  fy >> cx >> cy >> tx;
      cam.setKcam(fx,fy,cx,cy,tx);
    } else{
      is.clear();
      std::cerr << "cam not defined, using defaults" << std::endl;
      cam.setKcam(300,300,320,320,0.1);
    }

    // set the object
    estimate() = cam;

    //    std::cout << cam << std::endl;

    return true;
  }

  bool VertexCam::write(std::ostream& os) const
  {
    const SBACam &cam = estimate();

    // first the position and orientation (vector3 and quaternion)
    for (int i=0; i<3; i++)
      os << cam.translation()[i] << " ";
    for (int i=0; i<4; i++)
      os << cam.rotation().coeffs()[i] << " ";

    // now fx, fy, cx, cy, baseline
    os << cam.Kcam(0,0) << " ";
    os << cam.Kcam(1,1) << " ";
    os << cam.Kcam(0,2) << " ";
    os << cam.Kcam(1,2) << " ";
    os << cam.baseline << " ";
    return os.good();
  }

  EdgeSBACam::EdgeSBACam() :
    BaseBinaryEdge<6, SE3Quat, VertexCam, VertexCam>()
  {
  }

  bool EdgeSBACam::read(std::istream& is)
  {
    for (int i=0; i<7; i++)
      is >> measurement()[i];
    measurement().rotation().normalize();
    inverseMeasurement() = measurement().inverse();
    
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
	is >> information()(i,j);
	if (i!=j)
	  information()(j,i)=information()(i,j);
      }
    return true;
  }

  bool EdgeSBACam::write(std::ostream& os) const
  {
    for (int i=0; i<7; i++)
      os << measurement()[i] << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
	os << " " <<  information()(i,j);
      }
    return os.good();
  }

  void EdgeSBACam::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
  {
    VertexCam* from = static_cast<VertexCam*>(_vertices[0]);
    VertexCam* to = static_cast<VertexCam*>(_vertices[1]);
    if (from_.count(from) > 0)
      to->estimate() = ((SE3Quat) from->estimate()) * _measurement;
    else
      from->estimate() = ((SE3Quat) to->estimate() * _inverseMeasurement);
  }


  VertexPointXYZ::VertexPointXYZ() : BaseVertex<3, Vector3d>()
  {
  }

  bool VertexPointXYZ::read(std::istream& is)
  {
    Vector3d lv;
    for (int i=0; i<3; i++)
      is >> lv[i];
    estimate() = lv;
    return true;
  }

  bool VertexPointXYZ::write(std::ostream& os) const
  {
    Vector3d lv=estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }

  // point to camera projection, monocular
  EdgeProjectP2MC::EdgeProjectP2MC() :
  BaseBinaryEdge<2, Vector2d, VertexPointXYZ, VertexCam>()
  {
    information().setIdentity();
  }

  bool EdgeProjectP2MC::read(std::istream& is)
  {
    // measured keypoint
    for (int i=0; i<2; i++)
      is >> measurement()[i];
    // don't need this if we don't use it in error calculation (???)
    inverseMeasurement() = -measurement();
    // information matrix is the identity for features, could be changed to allow arbitrary covariances
    information().setIdentity();
    return true;
  }

  bool EdgeProjectP2MC::write(std::ostream& os) const
  {
    for (int i=0; i<2; i++)
      os  << measurement()[i] << " ";
    return os.good();
  }

  // point to camera projection, stereo
  EdgeProjectP2SC::EdgeProjectP2SC() :
    BaseBinaryEdge<3, Vector3d, VertexPointXYZ, VertexCam>()
  {
  }

  bool EdgeProjectP2SC::read(std::istream& is)
  {
    // measured keypoint
    for (int i=0; i<3; i++)
      is >> measurement()[i];
    // don't need this if we don't use it in error calculation (???)
    inverseMeasurement() = -measurement();
    // information matrix is the identity for features, could be changed to allow arbitrary covariances
    information().setIdentity();
    return true;
  }

  bool EdgeProjectP2SC::write(std::ostream& os) const
  {
    for (int i=0; i<3; i++)
      os  << measurement()[i] << " ";
    return os.good();
  }

/**
 * \brief Jacobian for stereo projection
 */
  void EdgeProjectP2SC::linearizeOplus()
  {
    VertexCam *vc = static_cast<VertexCam *>(_vertices[1]);
    SBACam &cam = vc->estimate();

    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[0]);
    Vector4d pt, trans;
    pt.head<3>() = vp->estimate();
    pt(3) = 1.0;
    trans.head<3>() = cam.translation();
    trans(3) = 1.0;

    // first get the world point in camera coords
    Eigen::Matrix<double,3,1> pc = cam.w2n * pt;

    // Jacobians wrt camera parameters
    // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    double px = pc(0);
    double py = pc(1);
    double pz = pc(2);
    double ipz2 = 1.0/(pz*pz);
    if (isnan(ipz2) ) 
      { 
	std::cout << "[SetJac] infinite jac" << std::endl;
	*(int *)0x0 = 0; 
      }

    double ipz2fx = ipz2*cam.Kcam(0,0); // Fx
    double ipz2fy = ipz2*cam.Kcam(1,1); // Fy
    double b      = cam.baseline; // stereo baseline

    Eigen::Matrix<double,3,1> pwt;

    // check for local vars
    pwt = (pt-trans).head<3>(); // transform translations, use differential rotation

    // dx
    Eigen::Matrix<double,3,1> dp = cam.dRdx * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,3) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,3) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,3) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    // dy
    dp = cam.dRdy * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,4) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,4) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,4) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    // dz
    dp = cam.dRdz * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,5) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,5) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,5) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -cam.w2n.col(0);        // dpc / dx
    _jacobianOplusXj(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = -cam.w2n.col(1);        // dpc / dy
    _jacobianOplusXj(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = -cam.w2n.col(2);        // dpc / dz
    _jacobianOplusXj(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = cam.w2n.col(0); // dpc / dx
    _jacobianOplusXi(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = cam.w2n.col(1); // dpc / dy
    _jacobianOplusXi(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = cam.w2n.col(2); // dpc / dz
    _jacobianOplusXi(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
  }


/**
 * \brief Jacobian for monocular projection
 */
  void EdgeProjectP2MC::linearizeOplus()
  {
    VertexCam *vc = static_cast<VertexCam *>(_vertices[1]);
    SBACam &cam = vc->estimate();

    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[0]);
    Vector4d pt, trans;
    pt.head<3>() = vp->estimate();
    pt(3) = 1.0;
    trans.head<3>() = cam.translation();
    trans(3) = 1.0;

    // first get the world point in camera coords
    Eigen::Matrix<double,3,1> pc = cam.w2n * pt;

    // Jacobians wrt camera parameters
    // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    double px = pc(0);
    double py = pc(1);
    double pz = pc(2);
    double ipz2 = 1.0/(pz*pz);
    if (isnan(ipz2) ) 
      { 
	std::cout << "[SetJac] infinite jac" << std::endl;
	*(int *)0x0 = 0; 
      }

    double ipz2fx = ipz2*cam.Kcam(0,0); // Fx
    double ipz2fy = ipz2*cam.Kcam(1,1); // Fy

    Eigen::Matrix<double,3,1> pwt;

    // check for local vars
    pwt = (pt-trans).head<3>(); // transform translations, use differential rotation

    // dx
    Eigen::Matrix<double,3,1> dp = cam.dRdx * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,3) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,3) = (pz*dp(1) - py*dp(2))*ipz2fy;
    // dy
    dp = cam.dRdy * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,4) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,4) = (pz*dp(1) - py*dp(2))*ipz2fy;
    // dz
    dp = cam.dRdz * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,5) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,5) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -cam.w2n.col(0);        // dpc / dx
    _jacobianOplusXj(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -cam.w2n.col(1);        // dpc / dy
    _jacobianOplusXj(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -cam.w2n.col(2);        // dpc / dz
    _jacobianOplusXj(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = cam.w2n.col(0); // dpc / dx
    _jacobianOplusXi(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = cam.w2n.col(1); // dpc / dy
    _jacobianOplusXi(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = cam.w2n.col(2); // dpc / dz
    _jacobianOplusXi(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
  }



  // point to camera projection, monocular
  EdgeProjectP2MC_Intrinsics::EdgeProjectP2MC_Intrinsics() :
    BaseMultiEdge<2, Vector2d>()
  {
    information().setIdentity();
    resize(3);
    _jacobianOplus[0].resize(2,3);
    _jacobianOplus[1].resize(2,6);
    _jacobianOplus[2].resize(2,4);
  }

/**
 * \brief Jacobian for monocular projection with intrinsics calibration
 */
  void EdgeProjectP2MC_Intrinsics::linearizeOplus()
  {
    VertexCam *vc = static_cast<VertexCam *>(_vertices[1]);
    SBACam &cam = vc->estimate();

    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[0]);

    //VertexIntrinsics *intr = static_cast<VertexIntrinsics *>(_vertices[2]);

    Vector4d pt, trans;
    pt.head<3>() = vp->estimate();
    pt(3) = 1.0;
    trans.head<3>() = cam.translation();
    trans(3) = 1.0;

    // first get the world point in camera coords
    Eigen::Matrix<double,3,1> pc = cam.w2n * pt;

    // Jacobians wrt camera parameters
    // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    double px = pc(0);
    double py = pc(1);
    double pz = pc(2);
    double ipz2 = 1.0/(pz*pz);
    if (isnan(ipz2) ) 
      { 
	std::cout << "[SetJac] infinite jac" << std::endl;
	*(int *)0x0 = 0; 
      }

    double ipz2fx = ipz2*cam.Kcam(0,0); // Fx
    double ipz2fy = ipz2*cam.Kcam(1,1); // Fy

    Eigen::Matrix<double,3,1> pwt;

    // check for local vars
    pwt = (pt-trans).head<3>(); // transform translations, use differential rotation

    // dx
    Eigen::Matrix<double,3,1> dp = cam.dRdx * pwt; // dR'/dq * [pw - t]
    _jacobianOplus[1](0,3) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[1](1,3) = (pz*dp(1) - py*dp(2))*ipz2fy;
    // dy
    dp = cam.dRdy * pwt; // dR'/dq * [pw - t]
    _jacobianOplus[1](0,4) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[1](1,4) = (pz*dp(1) - py*dp(2))*ipz2fy;
    // dz
    dp = cam.dRdz * pwt; // dR'/dq * [pw - t]
    _jacobianOplus[1](0,5) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[1](1,5) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -cam.w2n.col(0);        // dpc / dx
    _jacobianOplus[1](0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[1](1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -cam.w2n.col(1);        // dpc / dy
    _jacobianOplus[1](0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[1](1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -cam.w2n.col(2);        // dpc / dz
    _jacobianOplus[1](0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[1](1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = cam.w2n.col(0); // dpc / dx
    _jacobianOplus[0](0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[0](1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = cam.w2n.col(1); // dpc / dy
    _jacobianOplus[0](0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[0](1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = cam.w2n.col(2); // dpc / dz
    _jacobianOplus[0](0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplus[0](1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // Jacobians w.r.t the intrinsics
    _jacobianOplus[2].setZero();
    _jacobianOplus[2](0,0) = px/pz; // dx/dfx
    _jacobianOplus[2](1,1) = py/pz; // dy/dfy
    _jacobianOplus[2](0,2) = 1.;    // dx/dcx
    _jacobianOplus[2](1,3) = 1.;    // dy/dcy
  }

  bool EdgeProjectP2MC_Intrinsics::read(std::istream& is)
  {
    // measured keypoint
    for (int i=0; i<2; i++)
      is >> measurement()[i];
    // don't need this if we don't use it in error calculation (???)
    inverseMeasurement() = -measurement();
    // information matrix is the identity for features, could be changed to allow arbitrary covariances
    information().setIdentity();
    return true;
  }

  bool EdgeProjectP2MC_Intrinsics::write(std::ostream& os) const
  {
    for (int i=0; i<2; i++)
      os  << measurement()[i] << " ";
    return os.good();
  }


  // point to camera projection, stereo
  EdgeSBAScale::EdgeSBAScale() :
    BaseBinaryEdge<1, double, VertexCam, VertexCam>()
  {
  }
  
  bool EdgeSBAScale::read(std::istream& is)
  {
    is >> measurement();
    inverseMeasurement() = -measurement();
    information().setIdentity();
    is >> information()(0,0);
    return true;
  }

  bool EdgeSBAScale::write(std::ostream& os) const
  {
    os  << measurement() << " " << information()(0,0);
    return os.good();
  }

  void EdgeSBAScale::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
  {
    VertexCam* v1 = dynamic_cast<VertexCam*>(_vertices[0]);
    VertexCam* v2 = dynamic_cast<VertexCam*>(_vertices[1]);
    //compute the translation vector of v1 w.r.t v2
    if (from_.count(v1) == 1){
      SE3Quat delta = (v1->estimate().inverse()*v2->estimate());
      double norm =  delta.translation().norm();
      double alpha = _measurement/norm;
      delta.translation()*=alpha;
      v2->estimate()=v1->estimate()*delta;
    } else {
      SE3Quat delta = (v2->estimate().inverse()*v1->estimate());
      double norm =  delta.translation().norm();
      double alpha = _measurement/norm;
      delta.translation()*=alpha;
      v1->estimate()=v2->estimate()*delta;
    }
  }

} // end namespace
