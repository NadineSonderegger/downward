#ifndef POTENTIALS_POTENTIAL_FUNCTION_H
#define POTENTIALS_POTENTIAL_FUNCTION_H

#include <vector>
#include "../utils/hash.h"

class State;

namespace potentials {
/*
  A potential function calculates the sum of potentials in a given state.

  We decouple potential functions from potential heuristics to avoid the
  overhead that is induced by evaluating heuristics whenever possible.
*/
class PotentialFunctionFeatures {
    const utils::HashMap<std::vector<std::pair<int, int>>, int> feature_potentials;

public:
    explicit PotentialFunctionFeatures(
        utils::HashMap<std::vector<std::pair<int, int>>, int> &&fact_potentials);
    ~PotentialFunctionFeatures() = default;

    int get_value(const State &state) const;
};
}

#endif
