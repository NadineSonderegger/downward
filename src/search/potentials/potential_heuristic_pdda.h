#ifndef POTENTIALS_POTENTIAL_HEURISTIC_PDDA_H
#define POTENTIALS_POTENTIAL_HEURISTIC_PDDA_H

#include "../heuristic.h"

#include <memory>

namespace potentials {
class PotentialFunctionFeatures;

/*
  Use an internal potential function to evaluate a given state.
*/
class PotentialHeuristicPDDA : public Heuristic {
    std::unique_ptr<PotentialFunctionFeatures> function1;
    std::unique_ptr<PotentialFunctionFeatures> function2;

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit PotentialHeuristicPDDA(
        std::unique_ptr<PotentialFunctionFeatures> function1,
        std::unique_ptr<PotentialFunctionFeatures> function2,
        const std::shared_ptr<AbstractTask> &transform,
        bool cache_estimates, const std::string &description,
        utils::Verbosity verbosity);
};
}

#endif
