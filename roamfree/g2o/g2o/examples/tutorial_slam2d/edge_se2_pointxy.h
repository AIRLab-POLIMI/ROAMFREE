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

#ifndef G2O_TUTORIAL_EDGE_SE2_POINT_XY_H
#define G2O_TUTORIAL_EDGE_SE2_POINT_XY_H

#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {

  namespace tutorial {

    class EdgeSE2PointXY : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          EdgeSE2PointXY();

        void computeError()
        {
          const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
          const VertexPointXY* l2 = static_cast<const VertexPointXY*>(_vertices[1]);
          _error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    };

  } // end namespace
} // end namespace

#endif
