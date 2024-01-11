#include "Lidar2ImageProjectionTieFeatureM.h"

namespace ROAMfunctions {


//POSITION, ORIENTATION, LINEARVELOCITY, ANGULARVELOCITY, ACCELERATION, ANGULARACCELERATION, DELTA_POSITION, DELTA_ORIENTATION, IMUINT_DELTAPOSE, PREVIOUS_ORIENTATION, PREVIOUS_LINEARVELOCITY, PREVIOUS_ANGULARVELOCITY
const bool LiDAR2ImageProjectionTieFeatureM::_usedComponents[] = {false, true, false, false,false, false, true, false, false, true, false, false};

const std::string LiDAR2ImageProjectionTieFeatureM::_paramsNames[] = {"lqOS", "lSO", "cqOS", "cSO", "CM", "RD", "TD", "SKEW"};

} /* namespace ROAMfunctions */

