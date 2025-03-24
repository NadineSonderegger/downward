#include "potential_heuristic.h"

#include "potential_function.h"

#include "../plugins/plugin.h"

using namespace std;

namespace potentials {
PotentialHeuristicPDDA::PotentialHeuristic(
    unique_ptr<PotentialFunctionFeatures> function,
    const shared_ptr<AbstractTask> &transform, bool cache_estimates,
    const string &description, utils::Verbosity verbosity)
    : Heuristic(transform, cache_estimates, description, verbosity),
      function(move(function)) {
}


int PotentialHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    if (function2->get_value(state) >= 0)
    {
        return max(0, function1->get_value(state));
    }
    
    return std::numeric_limits<int>::max();
}
}

