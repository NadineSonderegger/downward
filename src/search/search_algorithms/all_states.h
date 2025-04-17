#ifndef SEARCH_ALGORITHMS_ALL_STATES_H
#define SEARCH_ALGORITHMS_ALL_STATES_H

#include "../open_list.h"
#include "../search_algorithm.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;
class OpenListFactory;

namespace plugins {
class Feature;
}

namespace all_states {
class AllStates : public SearchAlgorithm {
    std::shared_ptr<Evaluator> h_evaluator;

    void generate_states(int index,
        const std::vector<VariableProxy> &variables,
        std::vector<int> &current,
        const std::function<bool(const std::vector<int> &)> &callback,
        bool &stop_flag);

    bool is_goal_state(const State &s, const GoalsProxy &goals);

protected:
    virtual SearchStatus step() override;

public:
    explicit AllStates(
        const std::shared_ptr<Evaluator> &h_eval);
    virtual void print_statistics() const{};
    };

}

#endif
