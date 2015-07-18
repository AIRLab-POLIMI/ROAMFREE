// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "g2o_qglviewer.h"

#include "primitives.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_graph_action.h"

#include <iostream>
using namespace std;

namespace g2o {

namespace {

  /**
   * \brief helper for setting up a camera for qglviewer
   */
  class StandardCamera : public qglviewer::Camera
  {
    public:
      StandardCamera() : _standard(true) {};

      float zNear() const {
        if (_standard) 
          return 0.001; 
        else 
          return Camera::zNear(); 
      }

      float zFar() const
      {  
        if (_standard) 
          return 10000.0; 
        else 
          return Camera::zFar();
      }

      const bool& standard() const {return _standard;}
      bool& standard() {return _standard;}

    private:
      bool _standard;
  };

} // end anonymous namespace

G2oQGLViewer::G2oQGLViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WFlags flags) :
  QGLViewer(parent, shareWidget, flags),
  graph(0), _drawActions(0), _drawList(0)
{
}

G2oQGLViewer::~G2oQGLViewer()
{
}

void G2oQGLViewer::draw()
{
  if (! graph)
    return;

  if (_drawActions == 0) {
    _drawActions = HyperGraphActionLibrary::instance()->actionByName("draw");
    assert(_drawActions);
  }
  
  if (_updateDisplay) {
    _updateDisplay = false;
    glNewList(_drawList, GL_COMPILE_AND_EXECUTE);
    applyAction(graph, _drawActions);
    glEndList();
  } else {
    glCallList(_drawList); 
  }
}

void G2oQGLViewer::init()
{
  QGLViewer::init();

 //glDisable(GL_LIGHT0);
 //glDisable(GL_LIGHTING);

  setBackgroundColor(QColor::fromRgb(51, 51, 51));

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  //glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString::null);

  // mouse bindings
  setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

  // keyboard shortcuts
  setShortcut(CAMERA_MODE, 0);
  setShortcut(EXIT_VIEWER, 0);
  //setShortcut(SAVE_SCREENSHOT, 0);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  delete oldcam;

  // getting a display list
  _drawList = glGenLists(1);
}

void G2oQGLViewer::setUpdateDisplay(bool updateDisplay)
{
  _updateDisplay = updateDisplay;
}

} // end namespace
