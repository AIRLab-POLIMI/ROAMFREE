#include <iostream>

#include "g2o/core/optimizable_graph.h"

#include "Chi2TestAction.h"

namespace ROAMestimation {

Chi2TestAction::Chi2TestAction(bool *stopFlag) : stopFlag(stopFlag), chi2threshold(0.0) { 
    reset();
}

void Chi2TestAction::setChi2threshold(double threshold) {
    chi2threshold = threshold;
}

void Chi2TestAction::reset() {
    lastChi2 = std::numeric_limits<double>::infinity();
}

g2o::HyperGraphAction* Chi2TestAction::operator()(const g2o::HyperGraph* graph, Parameters* parameters) {
    double chi2 = ((g2o::OptimizableGraph *)graph)->chi2();

    // std::cerr << "last: " << std::fixed << lastChi2 << " current: " << std::fixed << chi2 << " delta: " << std::fixed << (lastChi2 - chi2) << std::endl;

    if (lastChi2 > chi2 && (lastChi2 - chi2) < chi2threshold) {            
        //std::cerr << "Early stopping :)" << std::endl;
        *stopFlag = true;
    }

    lastChi2 = chi2;

    return 0;
}

}