#ifndef VIEWER_VERTEX_DECORATOR_H
#define VIEWER_VERTEX_DECORATOR_H

#include <string>

namespace g2o {

class ViewerVertexDecorator {

public:

  ViewerVertexDecorator();
  ViewerVertexDecorator(const ViewerVertexDecorator &vd);

  //! true => the current estimate will be displayed in viewer
  bool displayEstimate() const;
  void setDisplayEstimate(bool displayEstimate);

protected:
  bool _displayEstimate;
  //std::string _category;
};

}

#endif
