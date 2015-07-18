#include "viewer_vertex_decorator.h"

#include <algorithm>
#include <iostream>

namespace g2o {

ViewerVertexDecorator::ViewerVertexDecorator() :
    _displayEstimate(false) {
}

ViewerVertexDecorator::ViewerVertexDecorator(const ViewerVertexDecorator &vd) {
  _displayEstimate = vd.displayEstimate();
  //_category = vd.accessCategory();
}

bool ViewerVertexDecorator::displayEstimate() const {
  return _displayEstimate;
}
void ViewerVertexDecorator::setDisplayEstimate(bool displayEstimate) {
  _displayEstimate = displayEstimate;
}

}
