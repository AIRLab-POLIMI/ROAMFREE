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
 * Eucl2DV.h
 *
 *  Created on: Jan 25, 2013
 *      Author: davide
 */

#ifndef EUCL2DV_H_
#define EUCL2DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Eucl2DV {
};

class Eucl2DVI: public Variable {
public:
  virtual ~Eucl2DVI();
};

template<>
struct VariableTraits<Eucl2DV> {

  static const unsigned int _INTERNAL_SIZE = 2;
  static const unsigned int _INCREMENT_SIZE = 2;

  typedef Eucl2DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* EUCL2DV_H_ */
