#ifndef POTENTIALS_POTENTIAL_HEURISTIC_INFINITY_H
#define POTENTIALS_POTENTIAL_HEURISTIC_INFINITY_H

#include "../heuristic.h"

#include <memory>

namespace potentials {
class PotentialFunctionFeatures;

/*
  Use an internal potential function to evaluate a given state.
*/
class PotentialHeuristicInfinity : public Heuristic {
    std::unique_ptr<PotentialFunctionFeatures> function;

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit PotentialHeuristicInfinity(
        std::unique_ptr<PotentialFunctionFeatures> function,
        const std::shared_ptr<AbstractTask> &transform,
        bool cache_estimates, const std::string &description,
        utils::Verbosity verbosity);
};
}

#endif
