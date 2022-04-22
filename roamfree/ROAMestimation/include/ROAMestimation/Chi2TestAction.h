#include "g2o/core/hyper_graph_action.h"

namespace ROAMestimation {

class Chi2TestAction : public g2o::HyperGraphAction {

public:

    Chi2TestAction(bool *stopFlag);

    void reset();

    void setChi2threshold(double threshold);    

    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0);

protected:

    double chi2threshold;

    bool *stopFlag;
    double lastChi2;
};

}