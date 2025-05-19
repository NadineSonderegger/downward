#include "potential_function_features.h"

#include "../task_proxy.h"

#include "../utils/hash.h"

#include <limits>
#include <vector>

using namespace std;

namespace potentials {
    PotentialFunctionFeatures::PotentialFunctionFeatures(
    utils::HashMap<vector<pair<int, int>>, int> &&feature_potentials_){
        feature_potentials.reserve(feature_potentials_.size());
        for (const auto &feature : feature_potentials_){
            feature_potentials.push_back(feature);
        }
}

int PotentialFunctionFeatures::get_value(const State &state) const {
    int heuristic_value = 0.0;
    state.unpack();
    vector<int> state_values = state.get_unpacked_values();

    for (const auto &feature : feature_potentials) {
        const vector<pair<int, int>> &feature_set = feature.first;
        bool all_in_state = true;

        for (const auto &atom : feature_set) {
            int var = atom.first;
            int value = atom.second;
            if ( state_values[var] != value) {
                all_in_state = false;
                break;
            }
        }
        
        if (all_in_state) {
            if (feature.second == numeric_limits<int>::max())
            {
                /*
                cout << "got infinity" << endl;
                for (const auto &atom : feature_set) {
                    cout << "considering feature " << atom.first << ", " << atom.second << endl;
                }
                cout << "end infinity." << endl;
                */
                return numeric_limits<int>::max();
            }
            /*
            for (const auto &atom : feature_set) {
                cout << "considering feature " << atom.first << ", " << atom.second << endl;
            }
            cout << " with weight " << feature.second << endl;
            cout << "  is in state" << endl; 
            */
            heuristic_value += feature.second;
        }
        else{
            // cout << "  is not in state" << endl;
        }
    }
    cout << heuristic_value << endl;
    return heuristic_value;
}
}
