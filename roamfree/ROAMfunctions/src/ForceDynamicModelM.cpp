#include "ForceDynamicModelM.h"

namespace ROAMfunctions
{                                                    // x      q      v      w      a    alpha
    const bool ForceDynamicModelM::_usedComponents[] = {false, true,  false,  true, true, true, false, false, false, false, false, false};

    const std::string ForceDynamicModelM::_paramsNames[] = {"Ibd","Ibod"};
} /* namespace ROAMfunctions */