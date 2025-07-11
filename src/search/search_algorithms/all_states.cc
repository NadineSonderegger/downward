#include "all_states.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../pruning_method.h"

#include "../plugins/options.h"
#include "../task_utils/successor_generator.h"
#include "../utils/logging.h"

#include "search_common.h"
#include "../plugins/plugin.h"

#include <cassert>
#include <memory>

using namespace std;

namespace all_states {
AllStates::AllStates(
    const shared_ptr<Evaluator> &h_eval)
    : SearchAlgorithm(NORMAL, 0, 0, " All states ", utils::Verbosity::NORMAL),
      h_evaluator(h_eval)      
      {
}

SearchStatus AllStates::step() {

    VariablesProxy variables = task_proxy.get_variables();
    std::vector<VariableProxy> variable_list;
    for (VariableProxy var : variables)
        variable_list.push_back(var);

    GoalsProxy goal = task_proxy.get_goals();
    
        
    std::vector<int> current(variables.size());
    bool stop_flag = false;

    State initial_state = task_proxy.get_initial_state();
    EvaluationContext eval_context_current(initial_state, 0, false, &statistics);
    int h_current = eval_context_current.get_evaluator_value_or_infinity(h_evaluator.get());
    if(h_current == numeric_limits<int>::max()){
        return FAILED;
    }

    generate_states(0, variable_list, current, [&](const std::vector<int> &state) -> bool {

        State s = task_proxy.create_state(std::vector<int>(state));

        EvaluationContext eval_context_current(s, 0, false, &statistics);
        int h_current = eval_context_current.get_evaluator_value_or_infinity(h_evaluator.get());
        //cout << "heuristic value current :" << h_current << endl;

        if (is_goal_state(s,goal))
        {
            // cout << "is a goal state" << endl;
            return true;
        }
        

        if(h_current == numeric_limits<int>::max()){
            return true;
        }

        vector<OperatorID> applicable_ops;
        successor_generator.generate_applicable_ops(s, applicable_ops);
        
        for (OperatorID op_id : applicable_ops) {
            OperatorProxy op = task_proxy.get_operators()[op_id];
    
            State succ_state = s.get_unregistered_successor(op);

            /* cout << "Successor State: ";
            for (FactProxy fact : succ_state){
                int val = fact.get_value();
                cout << val << " ";
            }
            cout << "\n"; */ 

            EvaluationContext eval_context_succ(succ_state, 0, false, &statistics);
            int h_succ = eval_context_succ.get_evaluator_value_or_infinity(h_evaluator.get());
            //cout << "heuristic value succ:" << h_succ << endl;
            if(h_succ < h_current){

                /*cout << "State: ";
                for (int val : state){
                    cout << val << " ";
                }
                cout << "\n";  */
                //cout << "Found improving successor: " << h_succ << " < " << h_current << std::endl;
                return true;
            }
        }

        cout << "Found no improving successor: "  << endl;
        cout << "State: ";
        for (int val : state){
            cout << val << " ";
        }
        cout << "\n"; 
        cout << "heuristic value current :" << h_current << endl;

        stop_flag = true;
        return false;  
        
    }, stop_flag);

    if (stop_flag){
        cout << "return FAILED" << endl;
        return FAILED;
    }
    cout << "Infinity-DDA property fullfilled" << endl;
    exit(0);  
    return SOLVED;

}

bool AllStates::is_goal_state(const State &s, const GoalsProxy &goals) {
    for (std::size_t i = 0; i < goals.size(); ++i) {
        FactProxy goal_fact = goals[i];
        if (s[goal_fact.get_variable()].get_value() != goal_fact.get_value()){
            return false;
        }
    }
    return true;
}

void AllStates::generate_states(int index,
    const std::vector<VariableProxy> &variables,
    std::vector<int> &current,
    const std::function<bool(const std::vector<int> &)> &callback,
    bool &stop_flag) {

    if (stop_flag)
        return;

    if (index == static_cast<int>(variables.size())) {
        if (!callback(current)) {
            stop_flag = true;
        }
        return;
    }

    int domain_size = variables[index].get_domain_size();
    for (int value = 0; value < domain_size; ++value) {
        current[index] = value;
        generate_states(index + 1, variables, current, callback, stop_flag);
        if (stop_flag)
            return;
    }
}


class AllStatesSearchFeature
    : public plugins::TypedFeature<SearchAlgorithm, all_states::AllStates> {
public:
    AllStatesSearchFeature() : TypedFeature("all_states") {
        document_title("All States");
        document_synopsis("");

        add_option<shared_ptr<Evaluator>>("h_eval", "evaluator");
    }

    virtual shared_ptr<all_states::AllStates>
    create_component(const plugins::Options &opts) const override {
        return make_shared<all_states::AllStates>(
                opts.get<shared_ptr<Evaluator>>("h_eval")
            );
    }
};

static plugins::FeaturePlugin<AllStatesSearchFeature> _plugin;
}
