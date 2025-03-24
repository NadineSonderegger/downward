#include "potential_function_features.h"

#include "../task_proxy.h"

#include "../utils/collections.h"

#include <cmath>

using namespace std;

namespace potentials {
    PotentialFunctionFeatures::PotentialFunctionFeatures(
    const unordered_map<vector<pair<int, int>>, int> &feature_potentials)
    : feature_potentials(feature_potentials) {
}

void generate_subsets(const vector<pair<int, int>> &features, vector<vector<pair<int, int>>> &subsets) {
    int n = features.size();
    for (int i = 1; i < (1 << n); ++i) {
        vector<pair<int, int>> subset;
        for (int j = 0; j < n; ++j) {
            if (i & (1 << j)) {
                subset.push_back(features[j]);
            }
        }
        subsets.push_back(subset);
    }
}

int PotentialFunctionFeatures::get_value(const State &state) const {
    int heuristic_value = 0.0;
    for (FactProxy fact : state) {
        int var_id = fact.get_variable().get_id();
        int value = fact.get_value();
    }
    return heuristic_value;
}
}

// unordered map
// copy once