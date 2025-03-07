#include "potential_function.h"
#include "potential_heuristic.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/system.h"

using namespace std;

namespace potentials{
static unique_ptr<PotentialFunction> create_spanner_potential_function(const TaskProxy &task_proxy){
    VariablesProxy variables = task_proxy.get_variables();
    vector<vector<double>> fact_potentials(variables.size());
    for (VariableProxy var : variables) {
        fact_potentials[var.get_id()].resize(var.get_domain_size());
    }
    // fill fact_potentials with values
    return make_unique<PotentialFunction>(fact_potentials);
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