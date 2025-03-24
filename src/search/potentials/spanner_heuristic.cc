#include "potential_function_features.h"
#include "potential_heuristic.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/system.h"

using namespace std;

namespace potentials{
static unique_ptr<PotentialFunctionFeatures> create_spanner_potential_function(const TaskProxy &task_proxy){
    VariablesProxy variables = task_proxy.get_variables();

    for (VariableProxy var : variables) {
    }
    // fill fact_potentials with values
    return make_unique<PotentialFunctionFeatures>(feature_potentials);
}
class SpannerHeuristicFeature
    : public plugins::TypedFeature<Evaluator, PotentialHeuristic> {
public:
    SpannerHeuristicFeature() : TypedFeature("spanner_heuristic") {
        add_heuristic_options_to_feature(*this, "spanner heuristic");
    }

    virtual shared_ptr<PotentialHeuristic>
    create_component(const plugins::Options &opts) const override {
        return make_shared<PotentialHeuristic>(
            create_spanner_potential_function(
                TaskProxy(*opts.get<shared_ptr<AbstractTask>>("transform"))
            ),
            opts.get<shared_ptr<AbstractTask>>("transform"),
            opts.get<bool>("cache_estimates"),
            opts.get<string>("description"),
            opts.get<utils::Verbosity>("verbosity")
            );
    }
};

static plugins::FeaturePlugin<SpannerHeuristicFeature> _plugin;
}