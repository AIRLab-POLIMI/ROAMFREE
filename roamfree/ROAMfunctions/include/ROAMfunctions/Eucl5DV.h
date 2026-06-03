/*

Contributors:
    Simon Gilgien (simon.gilgien@epfl.ch)
*/

/*
 * Eucl5DV.h
 *
 *  Created on: Apr 22, 2026
 *      Author: simon
 */

#ifndef EUCL5DV_H_
#define EUCL5DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Eucl5DV {
};

class Eucl5DVI: public Variable {
public:
  virtual ~Eucl5DVI();
};

template<>
struct VariableTraits<Eucl5DV> {

  static const unsigned int _INTERNAL_SIZE = 5;
  static const unsigned int _INCREMENT_SIZE = 5;

  typedef Eucl5DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* EUCL5DV_H_ */
