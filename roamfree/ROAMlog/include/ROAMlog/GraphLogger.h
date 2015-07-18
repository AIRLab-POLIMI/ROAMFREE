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
 * GraphLogger.h
 *
 *  Created on: Jun 4, 2013
 *      Author: davide
 */

#ifndef GRAPHLOGGER_H_
#define GRAPHLOGGER_H_

#include <string>
#include <map>

#include "SlotRandomLogger.h"

namespace g2o {
  class SparseOptimizer;
}

namespace ROAMlog {

class GraphLogger {

protected:
  std::string _basepath;
  g2o::SparseOptimizer *_optimizer;

  // TODO: remove pointers here, find a way to construct elements directly into the map
  std::map<std::string, SlotRandomLogger *> _loggers;

public:
  GraphLogger(const std::string &basepath, g2o::SparseOptimizer *opt);
  virtual ~GraphLogger();

  void sync();
};

} /* namespace ROAMlog */
#endif /* GRAPHLOGGER_H_ */
