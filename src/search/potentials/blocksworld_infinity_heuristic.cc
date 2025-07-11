#include "potential_function_features.h"
#include "potential_heuristic_infinity.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/hash.h"

#include <regex>

using namespace std;

namespace potentials{
static unique_ptr<PotentialFunctionFeatures> create_blocksworld_potential_function(const TaskProxy &task_proxy){
    VariablesProxy variables = task_proxy.get_variables();
    utils::HashMap<vector<pair<int, int>>, int> feature_potentials;

    vector<tuple<pair<int,int>,string,bool>> clear;
    vector<tuple<pair<int,int>,string,string>> on;
    unordered_set<string> all_blocks;
    unordered_map<string, int> goal_levels;
    unordered_map<string, string> initial_ons;
    unordered_map<string, int> initial_levels;


    regex clear_regex(R"((NegatedAtom|Atom) clear\((\w+)\))");
    regex on_regex(R"(Atom (on|ontable)\((\w+)(?:, (\w+))?\))");



    for (VariableProxy var : variables) {
        //int var_id = var.get_id();

        for (int value = 0; value < var.get_domain_size(); ++value) {
            string fact_name = var.get_fact(value).get_name();
            smatch match;

            if(regex_search(fact_name,match,clear_regex)){
                string negation = match[1];
                string block = match[2]; 
                bool is_clear = (negation == "Atom"); 

                clear.push_back({{var.get_id(),value},block,is_clear});

                if (all_blocks.find(block) == all_blocks.end()){
                    all_blocks.insert(block);
                }
            }
            else if(regex_search(fact_name,match,on_regex)){
                string predicate = match[1];  // "on" or "ontable"
                string block1 = match[2];     
                string block2 = match[3];     

                if (predicate == "ontable") {
                    on.push_back({{var.get_id(),value},block1,"table"});
                }
                else{
                    on.push_back({{var.get_id(),value},block1,block2});
                }
            }
        }
    }

    GoalsProxy goal = task_proxy.get_goals();
    unordered_map<string, string> goal_ons;

    for (FactProxy goal_fact : goal) {
        string goal_name = goal_fact.get_name();

        smatch match;
        if(regex_search(goal_name,match,on_regex)) {
            string predicate = match[1];  // "on" or "ontable"
            string block1 = match[2];     
            string block2 = match[3]; 

            if (predicate == "ontable") {
                goal_ons[block1] = "table";
            }
            else{
                goal_ons[block1] = block2;
            }
        }  
    }

    for(string block : all_blocks){
        auto it = goal_ons.find(block);
        if (it == goal_ons.end()){
            goal_ons[block] = "table";
        }
    }

    for (const string &block : all_blocks) {
        bool is_top = true;
        for (const auto &goals : goal_ons) {
            if (goals.second == block) {
                is_top = false;
                break;
            }
        }

        if (is_top) {
            string current = block;
            int level = 1;
            while (current != "table") {
                goal_levels[current] = level++;
                auto it = goal_ons.find(current);
                if (it == goal_ons.end()) break;
                current = it->second;
            }
        }
    }

    for(const auto &level : goal_levels){
        cout << level.first << level.second << endl;
    }


    State initial = task_proxy.get_initial_state();
    initial.unpack();
    vector<int> initial_values = initial.get_unpacked_values();

    for (const auto &entry : on) {
        pair<int, int> var_val = get<0>(entry);
        const string &above = get<1>(entry);
        const string &below = get<2>(entry);

        // If the fact holds in the initial state
        if (initial_values[var_val.first] == var_val.second) {
            initial_ons[above] = below;
        }
    }

    for (const string &block : all_blocks) {
        bool is_top = true;
        for (const auto &pair : initial_ons) {
            if (pair.second == block) {
                is_top = false;
                break;
            }
        }

        if (is_top) {
            string current = block;
            int level = 1;
            while (current != "table") {
                initial_levels[current] = level++;
                auto it = initial_ons.find(current);
                if (it == initial_ons.end()) break;
                current = it->second;
            }
        }
    }

    unordered_map<string, vector<string>> controls;
    for (const auto &entry : goal_ons) {
        string upper = entry.first;
        string lower = entry.second;

        while (lower != "table") {
            controls[upper].push_back(lower);
            auto it = goal_ons.find(lower);
            if (it == goal_ons.end())
                break;
            lower = it->second;
        }
    }

    for(const auto &blocks1: on){
        string upper_block1 = get<1>(blocks1);
        string lower_block1 = get<2>(blocks1);
        pair<int,int> values1 = get<0>(blocks1);

        if(lower_block1 == upper_block1){
            feature_potentials[{get<0>(blocks1)}] = numeric_limits<int>::max();
        }

        if(lower_block1 != "table" && upper_block1 != lower_block1 && lower_block1 != goal_ons[upper_block1]){
            feature_potentials[{get<0>(blocks1)}] = 2;
        }

        if(lower_block1 == "table" ){

            if(goal_ons[upper_block1] == "table"){
                feature_potentials[{get<0>(blocks1)}] = 0;
            }
            else{
                feature_potentials[{get<0>(blocks1)}] = 1;
            }
        }

        for(const auto &blocks2: on){
            string upper_block2 = get<1>(blocks2);
            string lower_block2 = get<2>(blocks2);
            pair<int,int> values2 = get<0>(blocks2);

            if(upper_block1 != upper_block2 && lower_block1 != "table" && lower_block2 != "table" &&
                (initial_values[values1.first]!=values1.second || initial_values[values2.first]!=values2.second) &&
                (goal_ons[upper_block1] != lower_block1 || goal_ons[upper_block2] != lower_block2)){

                    feature_potentials[{get<0>(blocks1),get<0>(blocks2)}] = numeric_limits<int>::max();
            }

            if(lower_block1 == goal_ons[upper_block1] && lower_block2 != goal_ons[upper_block2]){
                auto block_controls = controls.find(upper_block1);
                if (block_controls != controls.end()) {
                    bool controlled = find(block_controls->second.begin(), block_controls->second.end(), upper_block2) != block_controls->second.end();
                    if (controlled) {
                        feature_potentials[{get<0>(blocks1), get<0>(blocks2)}] = pow(2,goal_levels[upper_block1]);
                    }
                }
            }

            for(const auto &block_clear: clear){
                string block = get<1>(block_clear);
                bool is_clear = get<2>(block_clear);

                if(!is_clear && lower_block1 != block && lower_block2 != block && 
                    initial_ons[upper_block1]== block &&
                    goal_ons[upper_block2] == block){
                    feature_potentials[{get<0>(blocks1),get<0>(blocks2),get<0>(block_clear)}] = numeric_limits<int>::max();
                }

                if(!is_clear && lower_block1 != block && 
                    initial_ons[upper_block1]== block &&
                    goal_levels[block] == 1){
                    feature_potentials[{get<0>(blocks1),get<0>(block_clear)}] = numeric_limits<int>::max();
                }

                if(!is_clear && lower_block2 != block && 
                    initial_levels[block] == 1 &&
                    goal_ons[upper_block2] == block){
                    feature_potentials[{get<0>(blocks2),get<0>(block_clear)}] = numeric_limits<int>::max();
                }

                if(!is_clear &&  
                    initial_levels[block] == 1 &&
                    goal_levels[block] == 1){
                    feature_potentials[{get<0>(block_clear)}] = numeric_limits<int>::max();
                }
                
            }
        }

        for(const auto &block_clear: clear){
            string block = get<1>(block_clear);
            bool is_clear = get<2>(block_clear);

            if(is_clear && block == lower_block1){
                feature_potentials[{get<0>(blocks1),get<0>(block_clear)}] = numeric_limits<int>::max();
            }
        }
    }

    return make_unique<PotentialFunctionFeatures>(move(feature_potentials));
}
class BlocksworldInfHeuristicFeature
    : public plugins::TypedFeature<Evaluator, PotentialHeuristicInfinity> {
public:
    BlocksworldInfHeuristicFeature() : TypedFeature("blocksworld_infinity_heuristic") {
        add_heuristic_options_to_feature(*this, "blocksworld infinity dda heuristic");
    }

    virtual shared_ptr<PotentialHeuristicInfinity>
    create_component(const plugins::Options &opts) const override {
        return make_shared<PotentialHeuristicInfinity>(
            create_blocksworld_potential_function(
                TaskProxy(*opts.get<shared_ptr<AbstractTask>>("transform"))
            ),
            opts.get<shared_ptr<AbstractTask>>("transform"),
            opts.get<bool>("cache_estimates"),
            opts.get<string>("description"),
            opts.get<utils::Verbosity>("verbosity")
            );
    }
};

static plugins::FeaturePlugin<BlocksworldInfHeuristicFeature> _plugin;
}