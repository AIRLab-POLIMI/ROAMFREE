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
 * SlotRandomLogger.h
 *
 *  Created on: Jun 4, 2013
 *      Author: davide
 */

#ifndef SLOTRANDOMLOGGER_H_
#define SLOTRANDOMLOGGER_H_

#include <map>
#include <fstream>

#include "ROAMestimation/GenericVertexInterface.h"
#include "ROAMestimation/GenericEdgeInterface.h"
#include "ROAMestimation/PriorEdges/BasePriorEdgeInterface.h"

namespace ROAMlog {

class SlotRandomLogger {

protected:
  std::map<double, int> _slots;
  int _freeSlot;
  int _precision;

  int _issue;

  std::ofstream _f;

public:
  SlotRandomLogger(const std::string &file, int prec);
  ~SlotRandomLogger();

  template <typename Obj>
  void logObject(Obj &o) {
    // search the timestap in the map

    std::pair<std::map<double, int>::iterator, bool> ret = _slots.insert(std::pair<double, int>(o.getTimestamp(),0));
    if (ret.second) {
      ret.first->second=_freeSlot++;
    }

    int curslot = ret.first->second;
    int nfields = 2 + getToLogVectorSize(o);


    int slotSize = (8+_precision)*(nfields) + 2*(nfields-1)+1;

    _f.seekp(curslot * slotSize);

    _f << std::setw(8+_precision) << o.getTimestamp() << ", " << std::setw(8+_precision) << _issue;

    for (int k = 0; k < getToLogVectorSize(o); k++) {
      _f << ", " << std::setw(8+_precision) << getToLogComponent(k, o);
    }

    _f << "\n";

  }

  template <typename Obj>
  inline int getToLogVectorSize(Obj &o) const {
    return 0;
  }

  template <typename Obj>
  double getToLogComponent(int i, Obj &o) {
    return 0;
  }

  inline void flush() {
    _f.flush();
    _issue++;
  }
};

// --- specialization for the object I really want to log

template <>
int SlotRandomLogger::getToLogVectorSize(ROAMestimation::GenericVertexInterface &o) const;

template <>
int SlotRandomLogger::getToLogVectorSize(ROAMestimation::GenericEdgeInterface &o) const;

template <>
int SlotRandomLogger::getToLogVectorSize(ROAMestimation::BasePriorEdgeInterface &o) const;

template <>
double SlotRandomLogger::getToLogComponent(int i, ROAMestimation::GenericVertexInterface &o);

template <>
double SlotRandomLogger::getToLogComponent(int i, ROAMestimation::GenericEdgeInterface &o);

template <>
double SlotRandomLogger::getToLogComponent(int i, ROAMestimation::BasePriorEdgeInterface &o);

} /* namespace ROAMlog */

#endif /* SLOTRANDOMLOGGER_H_ */
