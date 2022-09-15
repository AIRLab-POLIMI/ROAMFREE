#include "PoseDerivativeM.h"

namespace ROAMfunctions {                        // x      q      v      w      a    alpha
const bool PoseDerivativeM::_usedComponents[] = {true, true,  true,  true, true, true, false, false, false, false, false, false};

const std::string PoseDerivativeM::_paramsNames[] = {};

} /* namespace ROAMfunctions */
