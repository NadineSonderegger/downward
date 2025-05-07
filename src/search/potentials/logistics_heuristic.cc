#include "potential_function_features.h"
#include "potential_heuristic_infinity.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/hash.h"

#include <regex>

using namespace std;

namespace potentials {
static unique_ptr<PotentialFunctionFeatures> create_logistics_potential_function(const TaskProxy &task_proxy) {
    VariablesProxy variables = task_proxy.get_variables();
    utils::HashMap<vector<pair<int, int>>, int> feature_potentials;

    GoalsProxy goal = task_proxy.get_goals();
    unordered_map<int, pair<int, int>> package_goal_location;

    // store goal destination for every package:
    for (FactProxy goal_fact : goal) {
        string goal_name = goal_fact.get_name();
        //cout << "Goal fact: " << goal_name << endl;

        smatch match;
        if (regex_search(goal_name, match, regex(R"(at\(package(\d+), city(\d+)-(\d+)\))"))) {
            int package = stoi(match[1]);
            int city = stoi(match[2]);
            int loc = stoi(match[3]);
            //cout << "package: " << package << " city: " << city << " location: " << loc << endl;
            package_goal_location[package] = {city, loc};
        }   
    }

    // store airport location in every city:
    vector<int> airport_location_by_city(50, -1);

    // store one preferred truck for each city:
    vector<int> designated_truck_by_city(50, -1);


    for (VariableProxy var : task_proxy.get_variables()) {
    
        for (int value = 0; value < var.get_domain_size(); ++value) {
            string fact_name = var.get_fact(value).get_name();
            smatch match;

            if (fact_name.find("package") != string::npos){
                break;
            }
    
            else if (regex_search(fact_name, match, regex(R"(at\(plane\d+, city(\d+)-(\d+)\))"))) {
                int city_index = stoi(match[1].str());
                int location_index = stoi(match[2].str());
                airport_location_by_city[city_index] = location_index;
            }

            else if (regex_search(fact_name, match, regex(R"(at\(truck(\d+), city(\d+)-\d+\))"))) {
                int truck_id = stoi(match[1]);
                int city_id = stoi(match[2]);
                
                // Assign only the first encountered truck per city
                if (designated_truck_by_city[city_id] == -1) {
                    designated_truck_by_city[city_id] = truck_id;
                }
                break;
            }
        }
    }


    for (VariableProxy var : variables) {
        for (VariableProxy other_var : variables) {

            for (int value = 0; value < var.get_domain_size(); ++value) {
                string fact_name = var.get_fact(value).get_name();
                smatch match;

                for (int other_val = 0; other_val < other_var.get_domain_size(); ++other_val) {
                    string other_fact = other_var.get_fact(other_val).get_name();
                    smatch other_match;

                    // Package on the ground
                    if (regex_search(fact_name, match, regex(R"(at\(package(\d+), city(\d+)-(\d+)\))"))) {
                        int package = stoi(match[1]);
                        int package_city = stoi(match[2]);
                        int package_loc = stoi(match[3]);
                        auto goal = package_goal_location.find(package);

                        if (goal != package_goal_location.end()) {
                            int goal_city = goal->second.first;
                            int goal_loc = goal->second.second;

                            if(package_city == goal_city && package_loc == goal_loc){
                                continue;
                            }

                            if(package_city == goal_city){

                                if (regex_search(other_fact, other_match, regex(R"(at\(truck(\d+), city(\d+)-(\d+)\))"))) {
                                    int truck_id = stoi(other_match[1]);
                                    int truck_city = stoi(other_match[2]);
                                    int truck_loc = stoi(other_match[3]);

                                    if (truck_id != designated_truck_by_city[truck_city]) {
                                        continue;
                                    }
                                    
                                    if(truck_city == package_city && truck_loc == package_loc){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 3;
                                    }
                                    else if(truck_city == package_city){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 4;
                                    }
                                }
                            }
                            else if(airport_location_by_city[package_city] == package_loc){

                                if (regex_search(other_fact, other_match, regex(R"(at\(plane(\d+), city(\d+)-(\d+)\))"))) {
                                    int plane_id = stoi(other_match[1]);
                                    int plane_city = stoi(other_match[2]);

                                    if(plane_id != 1){
                                        continue;
                                    }
                                    
                                    if(plane_city == package_city){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 7;
                                    }
                                    else{
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 8;
                                    }
                                }
                            }
                            else{
                                if (regex_search(other_fact, other_match, regex(R"(at\(truck(\d+), city(\d+)-(\d+)\))"))) {
                                    int truck_id = stoi(other_match[1]);
                                    int truck_city = stoi(other_match[2]);
                                    int truck_loc = stoi(other_match[3]);

                                    if (truck_id != designated_truck_by_city[truck_city]) {
                                        continue;
                                    }
                                    
                                    if(truck_city == package_city && truck_loc == package_loc){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 11;
                                    }
                                    else if(truck_city == package_city){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 12;
                                    }
                                }
                            }
                        }
                    }

                    // Package in truck
                    else if (regex_search(fact_name, match, regex(R"(in\(package(\d+), truck(\d+)\))"))) {
                        int package = stoi(match[1]);
                        int truck_id = stoi(match[2]);

                        auto goal = package_goal_location.find(package);

                        if (goal != package_goal_location.end()) {
                            int goal_city = goal->second.first;
                            int goal_loc = goal->second.second;

                            if (regex_search(other_fact, other_match, regex(R"(at\(truck(\d+), city(\d+)-(\d+)\))"))) {
                                int truck = stoi(other_match[1]);
                                int truck_city = stoi(other_match[2]);
                                int truck_loc = stoi(other_match[3]);

                                if(truck_id == truck){

                                    if(truck_city == goal_city && truck_loc == goal_loc){
                                        //cout << "truck has package and is at goal loc" << endl;
                                        //cout << var.get_id() << value << other_var.get_id() << other_val << endl;
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 1;
                                    }
                                    else if(truck_city == goal_city){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 2;
                                    }
                                    else if(airport_location_by_city[truck_city] == truck_loc){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 9;
                                    }
                                    else{
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 10;
                                    }
                                }
                            }
                        }
                    }

                    // Package in plane
                    else if (regex_search(fact_name, match, regex(R"(in\(package(\d+), plane(\d+)\))"))) {
                        int package = stoi(match[1]);
                        int plane_id = stoi(match[2]);

                        auto goal = package_goal_location.find(package);

                        if (goal != package_goal_location.end()) {
                            int goal_city = goal->second.first;

                            if (regex_search(other_fact, other_match, regex(R"(at\(plane(\d+), city(\d+)-\d+\))"))) {
                                int plane = stoi(other_match[1]);
                                int plane_city = stoi(other_match[2]);

                                if(plane_id == plane){

                                    if(plane_city == goal_city){
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 5;
                                    }
                                    else{
                                        feature_potentials[{{var.get_id(), value}, {other_var.get_id(), other_val}}] = 6;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return make_unique<PotentialFunctionFeatures>(move(feature_potentials));
}

class LogisticsHeuristicFeature
    : public plugins::TypedFeature<Evaluator, PotentialHeuristicInfinity> {
public:
    LogisticsHeuristicFeature() : TypedFeature("logistics_heuristic") {
        add_heuristic_options_to_feature(*this, "logistics heuristic");
    }

    virtual shared_ptr<PotentialHeuristicInfinity>
    create_component(const plugins::Options &opts) const override {
        return make_shared<PotentialHeuristicInfinity>(
            create_logistics_potential_function(
                TaskProxy(*opts.get<shared_ptr<AbstractTask>>("transform"))
            ),
            opts.get<shared_ptr<AbstractTask>>("transform"),
            opts.get<bool>("cache_estimates"),
            opts.get<string>("description"),
            opts.get<utils::Verbosity>("verbosity")
        );
    }
};

static plugins::FeaturePlugin<LogisticsHeuristicFeature> _plugin;
}