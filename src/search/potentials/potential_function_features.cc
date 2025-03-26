#include "potential_function_features.h"

#include "../task_proxy.h"

#include "../utils/collections.h"
#include "../utils/hash.h"

#include <cmath>
#include <limits>

using namespace std;

namespace potentials {
    PotentialFunctionFeatures::PotentialFunctionFeatures(
    utils::HashMap<vector<pair<int, int>>, int> &&feature_potentials)
    : feature_potentials(move(feature_potentials)) {
}

int PotentialFunctionFeatures::get_value(const State &state) const {
    int heuristic_value = 0.0;


    for (const auto &feature : feature_potentials) {
        const vector<pair<int, int>> &feature_set = feature.first;
        bool all_in_state = true;

        for (const auto &atom : feature_set) {
            int var = atom.first;
            int value = atom.second;
            if ( state[var].get_value() != value) { // could use state_features.count(atom) == 0 instead
                all_in_state = false;
                break;
            }
        }
        
        if (all_in_state) {
            heuristic_value += feature.second;
        }
    }
    return heuristic_value;
}
}

// unordered map
// copy once