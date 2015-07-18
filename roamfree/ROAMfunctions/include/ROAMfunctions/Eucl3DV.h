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
 * Eucl3DV.h
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#ifndef EUCL3DV_H_
#define EUCL3DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Eucl3DV {
};

class Eucl3DVI: public Variable {
public:
  virtual ~Eucl3DVI();
};

template<>
struct VariableTraits<Eucl3DV> {

  static const unsigned int _INTERNAL_SIZE = 3;
  static const unsigned int _INCREMENT_SIZE = 3;

  typedef Eucl3DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* EUCL3DV_H_ */
