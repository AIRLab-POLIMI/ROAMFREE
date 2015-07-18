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
 * Eucl1DV.h
 *
 *  Created on: Jan 7, 2013
 *      Author: davide
 */

#ifndef EUCL1DV_H_
#define EUCL1DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Eucl1DV {
};

class Eucl1DVI: public Variable {
public:
  virtual ~Eucl1DVI();
};

template<>
struct VariableTraits<Eucl1DV> {

  static const unsigned int _INTERNAL_SIZE = 1;
  static const unsigned int _INCREMENT_SIZE = 1;

  typedef Eucl1DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* EUCL1DV_H_ */
