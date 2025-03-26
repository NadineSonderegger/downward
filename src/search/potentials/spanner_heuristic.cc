#include "potential_function_features.h"
#include "potential_heuristic_infinity.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/system.h"
#include "../utils/hash.h"

using namespace std;

namespace potentials{
static unique_ptr<PotentialFunctionFeatures> create_spanner_potential_function(const TaskProxy &task_proxy){
    VariablesProxy variables = task_proxy.get_variables();
    utils::HashMap<vector<pair<int, int>>, int> feature_potentials;

    for (VariableProxy var : variables) {
        int var_id = var.get_id();
        for (int value = 0; value < var.get_domain_size(); ++value) {
            string fact_name = var.get_fact(value).get_name();

            if (fact_name.find("at(spanner") != string::npos) {
                feature_potentials[{{var_id, value}}] = 1;
            }
            else if (fact_name.find("loose(nut") != string::npos) {
                feature_potentials[{{var_id, value}}] = 1;

                // also check for non-usable spanner 
            } 
            else if (fact_name.find("at(bob, location") != string::npos) {
                
                // extract number of location
            }

            // fill fact_potentials with values
        }
    }
    return make_unique<PotentialFunctionFeatures>(move(feature_potentials));
}
class SpannerHeuristicFeature
    : public plugins::TypedFeature<Evaluator, PotentialHeuristicInfinity> {
public:
    SpannerHeuristicFeature() : TypedFeature("spanner_heuristic") {
        add_heuristic_options_to_feature(*this, "spanner heuristic");
    }

    virtual shared_ptr<PotentialHeuristicInfinity>
    create_component(const plugins::Options &opts) const override {
        return make_shared<PotentialHeuristicInfinity>(
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