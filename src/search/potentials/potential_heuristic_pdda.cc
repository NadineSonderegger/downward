#include "potential_heuristic_pdda.h"

#include "potential_function_features.h"

#include "../plugins/plugin.h"
#include <limits>

using namespace std;

namespace potentials {
PotentialHeuristicPDDA::PotentialHeuristicPDDA(
    unique_ptr<PotentialFunctionFeatures> function1,
    unique_ptr<PotentialFunctionFeatures> function2,
    const shared_ptr<AbstractTask> &transform, bool cache_estimates,
    const string &description, utils::Verbosity verbosity)
    : Heuristic(transform, cache_estimates, description, verbosity),
      function1(move(function1)),
      function2(move(function2)) {
}


int PotentialHeuristicPDDA::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    if (function2->get_value(state) <= 0)
    {
        return max(0, function1->get_value(state));
    }
    
    return std::numeric_limits<int>::max();
}
}

