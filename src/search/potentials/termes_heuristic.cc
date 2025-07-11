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

    // #fields = #all variables - hand variable - at variable + field_1 variable:
    int field_count = variables.size()-1;
    int max_height = 0;

    vector<vector<int>> weights_building;
    vector<tuple<pair<int,int>,int,int>> fields;
    vector<tuple<pair<int,int>,int,int>> goal_fields;
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
                //cout << "field " << field_id << " with height " << is_height << endl;

                if(is_height > max_height){
                    max_height = is_height;
                }

                if(field_id == field_count){
                    goal_fields.push_back({{var.get_id(),value},field_id,is_height});
                }
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

    // fil up weight matrix
    int diagonal = 1;
    if (field_count > 0) {
        weights_building.resize(field_count, std::vector<int>(max_height+1, 0));

        for(int h = 0; h < max_height + 1 ; h++){
            for(int i = field_count-1; i >-1; i--){

                int sum = 0;

                if(i >= h){

                    if(h == 0 && i != 0){
                        weights_building[i][h] = 0;
                    }
                    else if (h == 0 && i == 0){
                        weights_building[i][h] = 1;
                    }
                    else{
                        for (int ii = 0; ii < field_count ; ii++) {
                            // fields further back on the same height
                            if(ii > i){
                                sum += weights_building[ii][h];
                            }
                            // diagnoals
                            else if(ii < h){
                                sum += weights_building[ii][ii];
                            }
                            // fields in front of and on the height one below
                            else if (ii < i){
                                sum += weights_building[ii][h-1];
                            }
                        }

                        if(i == h){
                            weights_building[i][h] = sum+1;
                            diagonal += weights_building[i][h];
                        }
                        else{
                            weights_building[i][h] = sum+1;  
                        }  
                    }
                }
            }
        }
    }

    // save goal height for each field:
    GoalsProxy goal = task_proxy.get_goals();
    unordered_map<int, int> goal_heights;

    for (FactProxy goal_fact : goal) {
        string goal_name = goal_fact.get_name();

        smatch match;
        if (regex_search(goal_name, match, height)) {
            int field_id = stoi(match[1]);
            int height = stoi(match[2]);
        
            goal_heights[field_id] = height;
        }  
    }

    int walking_back = 1 + diagonal;
    //cout << "diagonal: " << diagonal << endl;
    /*for(int i = 1; i < field_count; i++){
        if(i > goal_heights[field_count]){
            walking_back += weights_building[i-1][goal_heights[field_count]];
        }
    }*/
    int placing_block = walking_back * (field_count);

    // print weight matrix:
    for (const auto &row : weights_building) {
        for (const auto &val : row) {
            cout << val << " ";
        }
        cout << endl;
    }

    // calculate maximum value after the last block of the tower got placed:
    int destructing =  walking_back * (field_count-1);
    for(int i = 1; i < field_count; i++){
        if(i < goal_heights[field_count]){
            destructing += placing_block*(i-1);
            int weight = weights_building[i-1][i]; 
            destructing += weight *abs((i-1)-(field_count-1));
        }
        else{
            destructing += placing_block*(goal_heights[field_count]-1);
            int weight = weights_building[i-1][goal_heights[field_count]-1]; 
            destructing += weight *abs((i-1)-(field_count-1));
        }
    }

    cout << destructing << endl;

    for(const auto &goal_field: goal_fields){
        int goal_field_id = get<1>(goal_field);
        int goal_field_height = get<2>(goal_field);

        for(const auto &field : fields){
            int field_id = get<1>(field);
            int field_height = get<2>(field);

            for(const auto &other_field : fields){
                int other_field_id = get<1>(other_field);
                int other_field_height = get<2>(other_field);

                for(const auto &at : robot){
                    int robot_field_id = get<1>(at);

                    for(const auto &in : hand){
                        bool block = get<1>(in);

                        // each field has its personal maximum height:
                        if (field_height >= field_id){
                            feature_potentials[{get<0>(field)}] = numeric_limits<int>::max();
                        }

                        if((field_id != field_count && field_height > goal_heights[field_count]-1) || (field_id == field_count && field_height > goal_heights[field_count])){
                            feature_potentials[{get<0>(field)}] = numeric_limits<int>::max();
                        }

                        // fields that are further away cannot be lower than nearby fields:
                        if(field_id < other_field_id && other_field_height < field_height ){
                            feature_potentials[{get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
                        }

                        // the robot never has to walk on the last field:
                        if(robot_field_id == field_count){
                            feature_potentials[{get<0>(at)}] = numeric_limits<int>::max();
                        }

                    
                        if(goal_field_height != goal_heights[goal_field_id]){

                            // 
                            feature_potentials[{get<0>(goal_field)}] = destructing;

                            // when building a staircase, place a block is good:
                            feature_potentials[{get<0>(goal_field),get<0>(field)}] = placing_block*((field_id-1)-field_height);

                            // a neighbor can only be one block higher:
                            if(other_field_height > field_height + 1 && field_id + 1 == other_field_id){
                                feature_potentials[{get<0>(goal_field),get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
                            }


                            // as long as one floor is not finished yet, another floor cannot be started:
                            if(field_id < other_field_id && field_height < field_id -1 && other_field_height > field_height + 1){
                                feature_potentials[{get<0>(goal_field),get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
                            }


                            // every field pushes the robot to the field_id-2 field
                            // The higher the field is and the closer it is to the depot, the greater its weight.
                            if(field_height != field_id -1 &&  field_height != 0){
                                int weight = weights_building[field_id-1][field_height];
                                feature_potentials[{get<0>(goal_field),get<0>(field),get<0>(at)}] = weight*abs((field_id-2)-robot_field_id);
                            }

                            // when a floor is finished, walk to the second last field
                            if(field_height == field_id -1){
                                int weight = weights_building[field_id-1][field_height];                            
                              feature_potentials[{get<0>(goal_field),get<0>(field),get<0>(at)}] = weight *abs((field_count-1)-robot_field_id);
                            }

                            // when no block is placed yet, walk to the second last field
                            if(field_id == field_count && field_height == 0){
                                feature_potentials[{get<0>(goal_field),get<0>(field),get<0>(at)}] = abs((field_count-1)-robot_field_id);
                            }

                            if(!block){
                                // when not carrying a block, walking towards the depot is good:
                                feature_potentials[{get<0>(goal_field),get<0>(at), get<0>(in)}] = walking_back*(robot_field_id);
                            }

                        }

                        if(goal_field_height == goal_heights[goal_field_id]){

                            // a neighbor can only be one block higher:
                            if(other_field_height > field_height + 1 && field_id + 1 == other_field_id && other_field_id != field_count){
                                feature_potentials[{get<0>(goal_field),get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
                            }

                            // as long as one floor is not finished yet, another floor cannot be started:
                            if(field_id < other_field_id && field_height < field_id -1 && other_field_height > field_height + 1 && other_field_id != field_count){
                                feature_potentials[{get<0>(goal_field),get<0>(field), get<0>(other_field)}] = numeric_limits<int>::max();
                            }

                            if(block){
                                // when not carrying a block, walking towards the depot is good:
                                feature_potentials[{get<0>(goal_field),get<0>(at), get<0>(in)}] = walking_back*(robot_field_id);
                            }

                            if(field_id != field_count){
                                int weight = weights_building[field_id-1][field_height]; 
                                //each block (except the tower) wants to be carried away:                           
                                feature_potentials[{get<0>(goal_field),get<0>(field),get<0>(at)}] = weight *abs((field_id-1)-robot_field_id);

                                // when destructing a staircase, taking a block is good:
                                feature_potentials[{get<0>(goal_field),get<0>(field)}] = placing_block*(field_height);
                            }

                            

                        }

                    }

                }

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