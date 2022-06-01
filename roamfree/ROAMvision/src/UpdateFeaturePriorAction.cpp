#include <iostream>

#include "g2o/core/optimizable_graph.h"

#include "EuclideanFeatureHandler.h"

#include "UpdateFeaturePriorAction.h"

namespace ROAMvision {

UpdateFeaturePriorAction::UpdateFeaturePriorAction(ROAMvision::EuclideanFeatureHandler *featureHandler) : _featureHandler(featureHandler) { 

}

g2o::HyperGraphAction* UpdateFeaturePriorAction::operator()(const g2o::HyperGraph* graph, Parameters* parameters) {

    std::cerr << "[UpdateFeaturePriorAction] Updating feature priors" << std::endl;

    _featureHandler->updateFeaturePriors();

    return 0;
}

}