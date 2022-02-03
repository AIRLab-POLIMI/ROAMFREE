/*

Contributors:
    Kenneth Joseph Paul (kenneth.josephpaul@epfl.ch)    
*/

/*
 * Eucl4DV.h
 *
 *  Created on: Jul 17, 2019
 *      Author: kenneth
 */

#ifndef EUCL4DV_H_
#define EUCL4DV_H_

#include "Variable.h"

namespace ROAMfunctions {

class Eucl4DV {
};

class Eucl4DVI: public Variable {
public:
  virtual ~Eucl4DVI();
};

template<>
struct VariableTraits<Eucl4DV> {

  static const unsigned int _INTERNAL_SIZE = 4;
  static const unsigned int _INCREMENT_SIZE = 4;

  typedef Eucl4DVI VariableI;
};

} /* namespace ROAMfunctions */
#endif /* EUCL4DV_H_ */
