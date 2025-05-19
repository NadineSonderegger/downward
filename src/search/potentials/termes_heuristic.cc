#include "potential_function_features.h"
#include "potential_heuristic_infinity.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/hash.h"

#include <regex>

using namespace std;

namespace potentials {
static unique_ptr<PotentialFunctionFeatures> create_termes_potential_function(const TaskProxy &task_proxy) {
    VariablesProxy variables = task_proxy.get_variables();
    utils::HashMap<vector<pair<int, int>>, int> feature_potentials;

    vector<tuple<pair<int,int>,int,int>> fields;
    vector<tuple<pair<int,int>,int>> robot;
    vector<tuple<pair<int,int>,bool>> hand;

    regex height(R"(height\(pos-(\d+), n(\d+)\))");
    regex robot_at(R"(at\(pos-(\d+)\))");

    for (VariableProxy var : task_proxy.get_variables()) {
    
        for (int value = 0; value < var.get_domain_size(); ++value) {
            string fact_name = var.get_fact(value).get_name();
            smatch match;

            if (regex_search(fact_name, match, height)){
                int field_id = stoi(match[1]);
                int is_height = stoi(match[2]);

                fields.push_back({{var.get_id(),value},field_id,is_height});
            }
            else if (regex_search(fact_name, match, robot_at)){
                int field_id = stoi(match[1]);

                robot.push_back({{var.get_id(),value},field_id});
            }
            else{
                int in_hand = fact_name.find("NegatedAtom");

                if (in_hand != string::npos){
                    hand.push_back({{var.get_id(),value},false});
                }
                else{
                    hand.push_back({{var.get_id(),value},true});
                }
            }
        }
    }

    // #fields = #all variables - hand variable - at variable + field_1 variable:
    int field_count = variables.size()-1;

    for(const auto &field : fields){

        int field_id = get<1>(field);
        int field_height = get<2>(field);

        // when building a staircase, place a block is good:
        feature_potentials[{get<0>(field)}] = (field_count-field_height)*(field_count*5);

        // each field has its personal maximum height:
        if (field_height >= field_id){
            feature_potentials[{get<0>(field)}] = numeric_limits<int>::max();
        }

        for(const auto &other_field : fields){
            int other_field_id = get<1>(other_field);
            int other_field_height = get<2>(other_field);

            // fields that are further away cannot be higher than nearby fields:
            if(field_id < other_field_id && other_field_height < field_height ){
                feature_potentials[{get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
            }

            // a neighbor can only be one block higher:
            if(other_field_height > field_height + 1 && field_id + 1 == other_field_id){
                feature_potentials[{get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
            }


            // as long as one floor is not finished yet, another floor cannot be started:
            // (not needed, because currently the robot cannot even walk on the unfinished floor)
            if(field_id < other_field_id && field_height < field_id -1 && other_field_height > field_height + 1){
                feature_potentials[{get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
            }
            

            for(const auto &at : robot){
                int robot_field_id = get<1>(at);

                // as long as one floor is not finished yet, the robot cannot walk on that floor:
                if(field_height < field_id -1 && robot_field_id == other_field_id && field_height < other_field_height){
                    feature_potentials[{get<0>(field), get<0>(other_field),get<0>(at)}] = numeric_limits<int>::max();
                }

                // identifies field_id where we have place next block (cannot handle case where we have to place a block in the last space)
                if(other_field_height < other_field_id -1 && field_height < other_field_height && field_id == other_field_id -1){
                    feature_potentials[{get<0>(field), get<0>(other_field),get<0>(at)}] = 2*abs((other_field_id-2)-robot_field_id);
                }

                // when the last two fields have the same height, then walking forward can be good
                if(other_field_id == field_count && field_height == other_field_height && field_id == other_field_id -1){
                    feature_potentials[{get<0>(field), get<0>(other_field),get<0>(at)}] = -robot_field_id;
                }


                /* for(const auto &in : hand){
                    bool block = get<1>(in);

                    // When the robot is carrying a block, it cannot stand right in front of a step belonging to an unfinished floor:
                    // we want to get rid of this (currently deciding if dimension 4 or not)
                    if(block && robot_field_id == field_id && other_field_height > field_height && field_id + 1 == other_field_id && field_height < field_id -1 ){
                        feature_potentials[{get<0>(field), get<0>(other_field),get<0>(at), get<0>(in)}] = numeric_limits<int>::max();
                    }

                } */
    
            }

        }

    }

    for(const auto &at : robot){
        int field_id = get<1>(at);

        // the robot never has to walk on the last field:
        if(field_id == field_count){
            feature_potentials[{get<0>(at)}] = numeric_limits<int>::max();
        }

        for(const auto &in : hand){
            bool block = get<1>(in);

            if(!block){
                // when not carrying a block, walking towards the depot is good:
                feature_potentials[{get<0>(at), get<0>(in)}] = 4*field_id ;
            }
            else{
                // when carrying a block, walking towards the tower is good:
                // not needed (we identify the best field to walk on and set weight according to this)
                //feature_potentials[{get<0>(at),get<0>(in)}] = -field_id;
            }      
        }
    }
    return make_unique<PotentialFunctionFeatures>(move(feature_potentials));
}

class TermesHeuristicFeature
    : public plugins::TypedFeature<Evaluator, PotentialHeuristicInfinity> {
public:
    TermesHeuristicFeature() : TypedFeature("termes_heuristic") {
        add_heuristic_options_to_feature(*this, "termes heuristic");
    }

    virtual shared_ptr<PotentialHeuristicInfinity>
    create_component(const plugins::Options &opts) const override {
        return make_shared<PotentialHeuristicInfinity>(
            create_termes_potential_function(
                TaskProxy(*opts.get<shared_ptr<AbstractTask>>("transform"))
            ),
            opts.get<shared_ptr<AbstractTask>>("transform"),
            opts.get<bool>("cache_estimates"),
            opts.get<string>("description"),
            opts.get<utils::Verbosity>("verbosity")
        );
    }
};

static plugins::FeaturePlugin<TermesHeuristicFeature> _plugin;
}