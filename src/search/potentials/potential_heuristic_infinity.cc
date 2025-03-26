#include "potential_heuristic_infinity.h"

#include "potential_function_features.h"

#include "../plugins/plugin.h"
#include <limits>

using namespace std;

namespace potentials {
PotentialHeuristicInfinity::PotentialHeuristicInfinity(
    unique_ptr<PotentialFunctionFeatures> function,
    const shared_ptr<AbstractTask> &transform, bool cache_estimates,
    const string &description, utils::Verbosity verbosity)
    : Heuristic(transform, cache_estimates, description, verbosity),
      function(move(function)) {
}


int PotentialHeuristicInfinity::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    return max(0, function->get_value(state));
}
}

