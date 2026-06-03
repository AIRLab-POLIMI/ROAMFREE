/*

Contributors:
    Simon Gilgien (simon.gilgien@epfl.ch)
*/

/*
 * Eucl6DV.h
 *
 *  Created on: Apr 20, 2026
 *      Author: simon
 */

#ifndef EUCL6DV_H_
#define EUCL6DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Eucl6DV {
};

class Eucl6DVI: public Variable {
public:
  virtual ~Eucl6DVI();
};

template<>
struct VariableTraits<Eucl6DV> {

  static const unsigned int _INTERNAL_SIZE = 6;
  static const unsigned int _INCREMENT_SIZE = 6;

  typedef Eucl6DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* EUCL6DV_H_ */
