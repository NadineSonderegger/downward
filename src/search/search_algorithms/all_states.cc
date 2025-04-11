#include "all_states.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../plugins/options.h"
#include "../task_utils/successor_generator.h"
#include "../utils/logging.h"

#include "search_common.h"
#include "../plugins/plugin.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional>
#include <set>

using namespace std;

namespace all_states {
AllStates::AllStates(
    const shared_ptr<OpenListFactory> &open, bool reopen_closed,
    const shared_ptr<Evaluator> &f_eval,
    const vector<shared_ptr<Evaluator>> &preferred,
    const shared_ptr<PruningMethod> &pruning,
    const shared_ptr<Evaluator> &lazy_evaluator, OperatorCost cost_type,
    int bound, double max_time, const string &description,
    utils::Verbosity verbosity)
    : SearchAlgorithm(
          cost_type, bound, max_time, description, verbosity),
      reopen_closed_nodes(reopen_closed),
      open_list(open->create_state_open_list()),
      f_evaluator(f_eval),     // default nullptr
      preferred_operator_evaluators(preferred),
      lazy_evaluator(lazy_evaluator),     // default nullptr
      pruning_method(pruning) {
    if (lazy_evaluator && !lazy_evaluator->does_cache_estimates()) {
        cerr << "lazy_evaluator must cache its estimates" << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
}

void AllStates::initialize() {
    
    log << "Conducting best first search"
        << (reopen_closed_nodes ? " with" : " without")
        << " reopening closed nodes, (real) bound = " << bound
        << endl;
    assert(open_list);

    set<Evaluator *> evals;
    open_list->get_path_dependent_evaluators(evals);

    /*
      Collect path-dependent evaluators that are used for preferred operators
      (in case they are not also used in the open list).
    */
    for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators) {
        evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the f_evaluator.
      They are usually also used in the open list and will hence already be
      included, but we want to be sure.
    */
    if (f_evaluator) {
        f_evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the lazy_evaluator
      (in case they are not already included).
    */
    if (lazy_evaluator) {
        lazy_evaluator->get_path_dependent_evaluators(evals);
    }

    path_dependent_evaluators.assign(evals.begin(), evals.end());

}

void AllStates::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
}

SearchStatus AllStates::step() {

    const auto &variables = task_proxy.get_variables();

    /*

    notes:

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];

        State succ_state = state_registry.get_successor_state(s, op);
        statistics.inc_generated();
    }
    */

    return IN_PROGRESS;
}

void AllStates::dump_search_space() const {
    search_space.dump(task_proxy);
}

void AllStates::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void AllStates::update_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

void add_all_states_options_to_feature(
    plugins::Feature &feature, const string &description) {
    add_search_pruning_options_to_feature(feature);
    add_search_algorithm_options_to_feature(feature, description);
}

tuple<shared_ptr<PruningMethod>, shared_ptr<Evaluator>, OperatorCost,
      int, double, string, utils::Verbosity>
get_all_states_arguments_from_options(const plugins::Options &opts) {
    return tuple_cat(
        get_search_pruning_arguments_from_options(opts),
        make_tuple(opts.get<shared_ptr<Evaluator>>(
                       "lazy_evaluator", nullptr)),
        get_search_algorithm_arguments_from_options(opts)
        );
}



class AllStatesSearchFeature
    : public plugins::TypedFeature<SearchAlgorithm, all_states::AllStates> {
public:
    AllStatesSearchFeature() : TypedFeature("all_states") {
        document_title("All States");
        document_synopsis("");

        add_list_option<shared_ptr<Evaluator>>("evals", "evaluators");
        add_list_option<shared_ptr<Evaluator>>(
            "preferred",
            "use preferred operators of these evaluators", "[]");
        add_option<int>(
            "boost",
            "boost value for preferred operator open lists", "0");
        all_states::add_all_states_options_to_feature(
            *this, "all_states");
    }

    virtual shared_ptr<all_states::AllStates>
    create_component(const plugins::Options &opts) const override {
        return plugins::make_shared_from_arg_tuples<all_states::AllStates>(
            search_common::create_greedy_open_list_factory(
                opts.get_list<shared_ptr<Evaluator>>("evals"),
                opts.get_list<shared_ptr<Evaluator>>("preferred"),
                opts.get<int>("boost")
                ),
            false,
            nullptr,
            opts.get_list<shared_ptr<Evaluator>>("preferred"),
            all_states::get_all_states_arguments_from_options(opts)
            );
    }
};

static plugins::FeaturePlugin<AllStatesSearchFeature> _plugin;
}
