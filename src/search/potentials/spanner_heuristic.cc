#include "potential_function_features.h"
#include "potential_heuristic_infinity.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/hash.h"

#include <regex>

using namespace std;

namespace potentials{
static unique_ptr<PotentialFunctionFeatures> create_spanner_potential_function(const TaskProxy &task_proxy){
    VariablesProxy variables = task_proxy.get_variables();
    utils::HashMap<vector<pair<int, int>>, int> feature_potentials;

    regex agent_at(R"(at\(agent, ([a-zA-Z]+)(\d*)\))");
    regex spanner_at(R"(at\(spanner\d+, ([a-zA-Z]+)(\d*)\))");
    regex nut_loose(R"(loose\(nut(\d+)\))");
    regex spanner_not_usable(R"(NegatedAtom useable\(spanner(\d+)\))");

    for (VariableProxy var : variables) {
        int var_id = var.get_id();

        for (int value = 0; value < var.get_domain_size(); ++value) {
            string fact_name = var.get_fact(value).get_name();

            smatch match;

            if (regex_search(fact_name, match, agent_at)) { // agent at some location
                string loc_name = match[1].str(); // place name
                string loc_number_str = match[2].str(); // number
                
                int agent_location = -1;
                if (loc_name == "shed") {
                    agent_location = 0;
                } else if (loc_name == "gate") {
                    agent_location = var.get_domain_size(); // gate = m
                } else if (loc_name == "location" && !loc_number_str.empty()) {
                    agent_location = stoi(loc_number_str); // locationX = X
                }
            
                feature_potentials[{{var.get_id(), value}}] = var.get_domain_size() - agent_location; // = m - i
            
                for (VariableProxy other_var : variables) {
                    for (int spanner_value = 0; spanner_value < other_var.get_domain_size(); ++spanner_value) {
                        string spanner_fact = other_var.get_fact(spanner_value).get_name();
                        smatch spanner_match;
            
                        if (regex_search(spanner_fact, spanner_match, spanner_at)) {
                            string spanner_loc_name = spanner_match[1].str();
                            string spanner_num_str = spanner_match[2].str();
            
                            int spanner_location = -1;
                            if (spanner_loc_name == "shed") {
                                spanner_location = 0;
                            } else if (spanner_loc_name == "gate") {
                                spanner_location = var.get_domain_size(); // m
                            } else if (spanner_loc_name == "location" && !spanner_num_str.empty()) {
                                spanner_location = stoi(spanner_num_str);
                            }
            
                            if (spanner_location < agent_location) {  // condition: k < i
                                feature_potentials[{{var.get_id(), value}, {other_var.get_id(), spanner_value}}] = numeric_limits<int>::max();
                            }
                        }
                    }
                }
            }

            else if (regex_search(fact_name, match, spanner_at)) { // spanner at location 
                string spanner_loc_name = match[1].str();
                string spanner_num_str = match[2].str();

                if (spanner_loc_name == "shed" || spanner_loc_name == "gate" || spanner_loc_name == "location") {
                    feature_potentials[{{var_id, value}}] = 1;
                }
            }
            else if (regex_search(fact_name, match, nut_loose)) { // loose nut
                int nut_id = stoi(match[1].str());  // extract nut number
                feature_potentials[{{var_id, value}}] = 1;

                for (VariableProxy other_var : variables) {
                    for (int other_value = 0; other_value < other_var.get_domain_size(); ++other_value) {
                        string other_fact_name = other_var.get_fact(other_value).get_name();
                        smatch other_match;
                        if (regex_search(other_fact_name, other_match, spanner_not_usable)) {
                            int spanner_id = stoi(other_match[1].str());  // extract spanner number
                            if (nut_id == spanner_id) {  // ensure numbers match
                                feature_potentials[{{var_id, value}, {other_var.get_id(), other_value}}] = numeric_limits<int>::max();
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