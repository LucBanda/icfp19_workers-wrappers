#include "agent.h"
#include "complex"

agent::agent(mine_state *arg_mine, mine_navigator *arg_navigator) {
	mine = arg_mine;
	initial_targets = mine->non_validated_tiles;
	navigator = arg_navigator;
}

agent::~agent() {}

void agent::set_execution_map(string exec) {
	execution = exec;

}

string agent::execution_map_from_node_list(vector<ListDigraph::Node> list_node) {
	string result;
	ListDigraph::Node last_node = navigator->initialNode;
	enum orientation result_orientation;

	for (auto it = list_node.begin(); it != list_node.end(); ++it) {
		ListDigraph::Node endingnode;
		position target_pos = navigator->coord_map[*it];
		if (mine->board_tile_is_painted(target_pos)) {
			continue;
		}
		string new_string =  navigator->goto_node(current_orientation, result_orientation, last_node, *it, &endingnode);
		last_node = endingnode;
		result += new_string;
		mine->apply_command(new_string);
		current_orientation = result_orientation;
	}
	return result;
}

int agent::run() {
	mine->apply_command(execution);
	return 0;
}

double agent::get_cost() {
	double score = mine->time_step;
	//cout << score << endl;
	return score;
}