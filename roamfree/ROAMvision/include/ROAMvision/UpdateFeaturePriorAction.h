#include "g2o/core/hyper_graph_action.h"

namespace ROAMvision {

class EuclideanFeatureHandler;

class UpdateFeaturePriorAction : public g2o::HyperGraphAction {

public:

    UpdateFeaturePriorAction(ROAMvision::EuclideanFeatureHandler *featureHandler);

    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0);

protected:

    ROAMvision::EuclideanFeatureHandler *_featureHandler;

};

}