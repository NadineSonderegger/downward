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

    // store goal location for every package
    unordered_map<int, pair<int, int>> package_goal_location;

    // store airport location in every city:
    unordered_map<int,int> airport_location_by_city;

    // store one preferred truck for each city:
    unordered_map<int,int> designated_truck_by_city;

    // 
    vector<tuple<pair<int,int>,int,pair<int,int>>> trucks;

    // 
    vector<tuple<pair<int,int>,int,int>> airplanes;

    // 
    vector<tuple<pair<int,int>,int,pair<int,int>>> packages_ground;
    vector<tuple<pair<int,int>,int,int>> packages_truck;
    vector<tuple<pair<int,int>,int,int>> packages_plane;
    
    regex package_at_ground(R"(at\(package(\d+), city(\d+)-(\d+)\))");
    regex plane_at(R"(at\(plane(\d+), city(\d+)-(\d+)\))");
    regex truck_at(R"(at\(truck(\d+), city(\d+)-(\d+)\))");
    regex package_in_truck(R"(in\(package(\d+), truck(\d+)\))");
    regex package_in_plane(R"(in\(package(\d+), plane(\d+)\))");

    // store goal destination for every package:
    for (FactProxy goal_fact : goal) {
        string goal_name = goal_fact.get_name();

        smatch match;
        if (regex_search(goal_name, match, package_at_ground)) {
            int package = stoi(match[1]);
            int city = stoi(match[2]);
            int loc = stoi(match[3]);
        
            package_goal_location[package] = {city, loc};
        }  
    }


    for (VariableProxy var : task_proxy.get_variables()) {
    
        for (int value = 0; value < var.get_domain_size(); ++value) {
            string fact_name = var.get_fact(value).get_name();
            smatch match;

            if (regex_search(fact_name, match, package_at_ground)){
                int package = stoi(match[1]);
                int package_city = stoi(match[2]);
                int package_loc = stoi(match[3]);

                packages_ground.push_back({{var.get_id(),value},package,{package_city,package_loc}});
            }

            else if (regex_search(fact_name, match, package_in_truck)){
                int package = stoi(match[1]);
                int truck_id = stoi(match[2]);

                packages_truck.push_back({{var.get_id(),value},package,truck_id});
            }

            else if (regex_search(fact_name, match, package_in_plane)){
                int package = stoi(match[1]);
                int plane_id = stoi(match[2]);

                packages_plane.push_back({{var.get_id(),value},package,plane_id});
            }
    
            else if (regex_search(fact_name, match, plane_at)) {
                int plane = stoi(match[1]);
                int city_index = stoi(match[2]);
                int location_index = stoi(match[3]);
                airport_location_by_city[city_index] = location_index;

                airplanes.push_back({{var.get_id(),value},plane,city_index});
            }

            else if (regex_search(fact_name, match, truck_at)) {
                int truck_id = stoi(match[1]);
                int city_id = stoi(match[2]);
                int location_index = stoi(match[3]);

                trucks.push_back({{var.get_id(),value},truck_id,{city_id,location_index}});
                
                // Assign only the first encountered truck per city
                //if (designated_truck_by_city.find(city_id) == designated_truck_by_city.end()) {
                designated_truck_by_city[city_id] = truck_id;
                //}
            }
        }
    }

    //packages is on the ground:

    for(const auto &package : packages_ground){

        int package_id = get<1>(package);
        int package_city = get<2>(package).first;
        int package_loc = get<2>(package).second;

        auto goal = package_goal_location.find(package_id);

        if (goal != package_goal_location.end()) {
            int goal_city = goal->second.first;
            int goal_loc = goal->second.second;

            if(package_city == goal_city && package_loc == goal_loc){
            }
            else if(package_city == goal_city){
                for(const auto &truck : trucks){

                    int truck_id = get<1>(truck);
                    int truck_city = get<2>(truck).first;
                    int truck_loc = get<2>(truck).second;
            
                    if (truck_id != designated_truck_by_city[truck_city]) {
                    }
                    else if(truck_city == package_city && truck_loc == package_loc){

                        feature_potentials[{get<0>(package), get<0>(truck)}] = 3;
                    }
                    else if(truck_city == package_city){
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 4;
                    }
                }
            }
            else if(airport_location_by_city[package_city] == package_loc){
                for(const auto &airplane : airplanes){
                    int plane_id = get<1>(airplane);
                    int plane_city = get<2>(airplane);

                    if(plane_id == 1){
                        if(plane_city == package_city){
                            feature_potentials[{get<0>(package), get<0>(airplane)}] = 7;
                        }
                        else{
                            feature_potentials[{get<0>(package),get<0>(airplane)}] = 8;
                        }
                    }
                }
            }
            else{
                for(const auto &truck : trucks){

                    int truck_id = get<1>(truck);
                    int truck_city = get<2>(truck).first;
                    int truck_loc = get<2>(truck).second;

                    if (truck_id != designated_truck_by_city[truck_city]) {
                    }
                    else if(truck_city == package_city && truck_loc == package_loc){
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 11;
                    }
                    else if(truck_city == package_city){
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 12;
                    }
                }
            }
        }
    }

    //package is in a truck
    for(const auto &package : packages_truck){
        int package_id = get<1>(package);
        int in_truck = get<2>(package);

        auto goal = package_goal_location.find(package_id);

        if (goal != package_goal_location.end()) {
            int goal_city = goal->second.first;
            int goal_loc = goal->second.second;

            for(const auto &truck : trucks){

                int truck_id = get<1>(truck);
                int truck_city = get<2>(truck).first;
                int truck_loc = get<2>(truck).second;
        
                if(in_truck == truck_id){

                    if(truck_city == goal_city && truck_loc == goal_loc){
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 1;
                    }
                    else if(truck_city == goal_city){
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 2;
                    }
                    else if(airport_location_by_city[truck_city] == truck_loc){
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 9;
                    }
                    else{
                        feature_potentials[{get<0>(package), get<0>(truck)}] = 10;
                    }
                }
            }
        }    
    }

    //package is in plane
    for(const auto &package : packages_plane){
        int package_id = get<1>(package);
        int in_plane = get<2>(package);

        auto goal = package_goal_location.find(package_id);

        if (goal != package_goal_location.end()) {
            int goal_city = goal->second.first;

            for(const auto &airplane: airplanes){
                int plane_id = get<1>(airplane);
                int plane_city = get<2>(airplane);

                if(in_plane == plane_id){

                    if(plane_city == goal_city){
                        feature_potentials[{get<0>(package), get<0>(airplane)}] = 5;
                    }
                    else{
                        feature_potentials[{get<0>(package), get<0>(airplane)}] = 6;
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