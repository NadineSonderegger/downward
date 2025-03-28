#include "potential_function_features.h"
#include "potential_heuristic_infinity.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/system.h"
#include "../utils/hash.h"

#include <regex>

using namespace std;

namespace potentials{
static unique_ptr<PotentialFunctionFeatures> create_spanner_potential_function(const TaskProxy &task_proxy){
    VariablesProxy variables = task_proxy.get_variables();
    utils::HashMap<vector<pair<int, int>>, int> feature_potentials;

    for (VariableProxy var : variables) {
        for (VariableProxy other_var : variables) {

            int var_id = var.get_id();

            for (int value = 0; value < var.get_domain_size(); ++value) {
                string fact_name = var.get_fact(value).get_name();

                smatch match;
                if (regex_search(fact_name, match, regex(R"(at\(agent, location(\d+)\))"))) { // agent at location i

                    int agent_location = stoi(match[1].str());  
                    feature_potentials[{{var.get_id(), value}}] = var.get_domain_size() - agent_location; // = m-i

                    for (int spanner_value = 0; spanner_value < other_var.get_domain_size(); ++spanner_value) {
                        string spanner_fact = other_var.get_fact(spanner_value).get_name();
                        smatch spanner_match;

                        if (regex_search(spanner_fact, spanner_match, regex(R"(at\(spanner\d+, location(\d+)\))"))) {
                            int spanner_location = stoi(spanner_match[1].str());  
    
                            if (spanner_location < agent_location) {  // condition: k < i
                                feature_potentials[{{var_id, value}, {other_var.get_id(), spanner_value}}] = std::numeric_limits<int>::max();
                            }
                        }
                    }
                } 
                else if (regex_search(fact_name, match, regex(R"(at\(spanner\d+, location\d+\))"))) { // spanner at location 
                    feature_potentials[{{var_id, value}}] = 1;
                }
                else if (regex_search(fact_name, match, regex(R"(loose\(nut(\d+)\))"))) { // loose nut
                    int nut_id = stoi(match[1].str());  // extract nut number
                    feature_potentials[{{var_id, value}}] = 1;

                    for (int other_value = 0; other_value < other_var.get_domain_size(); ++other_value) {
                        string other_fact_name = other_var.get_fact(other_value).get_name();
                        smatch other_match;
                        if (regex_search(other_fact_name, other_match, regex(R"(NegatedAtom useable\(spanner(\d+)\))"))) {
                            int spanner_id = stoi(other_match[1].str());  // extract spanner number
                            if (nut_id == spanner_id) {  // ensure numbers match
                                feature_potentials[{{var_id, value}, {other_var.get_id(), other_value}}] = std::numeric_limits<int>::max();
                            }
                        }
                    }
                    
                }
            }
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