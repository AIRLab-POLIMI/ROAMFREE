// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "edge_se2.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

  EdgeSE2::EdgeSE2() :
    BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>()
  {
  }

  bool EdgeSE2::read(std::istream& is)
  {
    Vector3d p;
    is >> p[0] >> p[1] >> p[2];
    measurement().fromVector(p);
    inverseMeasurement() = measurement().inverse();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2::write(std::ostream& os) const
  {
    Vector3d p = measurement().toVector();
    os << p.x() << " " << p.y() << " " << p.z();
    for (int i = 0; i < 3; ++i)
      for (int j = i; j < 3; ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  void EdgeSE2::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /* to */)
  {
    VertexSE2* fromEdge = static_cast<VertexSE2*>(_vertices[0]);
    VertexSE2* toEdge   = static_cast<VertexSE2*>(_vertices[1]);
    if (from.count(fromEdge) > 0)
      toEdge->estimate() = fromEdge->estimate() * _measurement;
    else
      fromEdge->estimate() = toEdge->estimate() * _inverseMeasurement;
  }

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeSE2::linearizeOplus()
  {
    const VertexSE2* vi = static_cast<const VertexSE2*>(_vertices[0]);
    const VertexSE2* vj = static_cast<const VertexSE2*>(_vertices[1]);
    double thetai = vi->estimate().rotation().angle();

    Vector2d dt = vj->estimate().translation() - vi->estimate().translation();
    double si=sin(thetai), ci=cos(thetai);

    _jacobianOplusXi(0, 0) = -ci; _jacobianOplusXi(0, 1) = -si; _jacobianOplusXi(0, 2) = -si*dt.x()+ci*dt.y();
    _jacobianOplusXi(1, 0) =  si; _jacobianOplusXi(1, 1) = -ci; _jacobianOplusXi(1, 2) = -ci*dt.x()-si*dt.y();
    _jacobianOplusXi(2, 0) =  0;  _jacobianOplusXi(2, 1) = 0;   _jacobianOplusXi(2, 2) = -1;

    _jacobianOplusXj(0, 0) = ci; _jacobianOplusXj(0, 1)= si; _jacobianOplusXj(0, 2)= 0;
    _jacobianOplusXj(1, 0) =-si; _jacobianOplusXj(1, 1)= ci; _jacobianOplusXj(1, 2)= 0;
    _jacobianOplusXj(2, 0) = 0;  _jacobianOplusXj(2, 1)= 0;  _jacobianOplusXj(2, 2)= 1;

    const SE2& rmean = inverseMeasurement();
    Matrix3d z = Matrix3d::Zero();
    z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
    z(2, 2) = 1.;
    _jacobianOplusXi = z * _jacobianOplusXi;
    _jacobianOplusXj = z * _jacobianOplusXj;
  }
#endif

  EdgeSE2WriteGnuplotAction::EdgeSE2WriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE2).name()){}

  HyperGraphElementAction* EdgeSE2WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE2* e =  static_cast<EdgeSE2*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertices()[0]);
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertices()[1]);
    *(params->os) << fromEdge->estimate().translation().x() << " " << fromEdge->estimate().translation().y()
      << " " << fromEdge->estimate().rotation().angle() << std::endl;
    *(params->os) << toEdge->estimate().translation().x() << " " << toEdge->estimate().translation().y()
      << " " << toEdge->estimate().rotation().angle() << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE2DrawAction::EdgeSE2DrawAction(): DrawAction(typeid(EdgeSE2).name()){}

  HyperGraphElementAction* EdgeSE2DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
							 HyperGraphElementAction::Parameters* /*params_*/){
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE2* e =  static_cast<EdgeSE2*>(element);
    VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertices()[0]);
    VertexSE2* toEdge   = static_cast<VertexSE2*>(e->vertices()[1]);
    glColor3f(0.5,0.5,0.8);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f(fromEdge->estimate().translation().x(),fromEdge->estimate().translation().y(),0.);
    glVertex3f(toEdge->estimate().translation().x(),toEdge->estimate().translation().y(),0.);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
