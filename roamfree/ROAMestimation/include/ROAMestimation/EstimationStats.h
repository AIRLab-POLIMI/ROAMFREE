/*
 * EstimationStats.h
 *
 *  Created on: Apr 25, 2022
 *      Author: davide
 */

#include <map>


namespace ROAMestimation {

struct EstimationStats {

    EstimationStats(int N, int dimension, double chi2) : N(N), dimension(dimension), chi2(chi2) {}

    int N;
    int dimension;
    double chi2;    
};

}