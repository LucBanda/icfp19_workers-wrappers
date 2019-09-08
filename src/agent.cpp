#include "agent.h"
#include "complex"

agent::agent(mine_state *arg_mine, mine_navigator *arg_navigator, Node arg_start) {
	mine = arg_mine;
	last_node = arg_start;
	initial_targets = mine->non_validated_tiles;
	navigator = arg_navigator;
}

agent::~agent() {}

void agent::set_execution_map(string exec) {
	execution = exec;

}

string agent::execution_map_from_node_list(vector<pair<Node, orientation>> list_node) {
	string result;
	//Node last_node = start;

	//cout << "execution node: " << navigator->graph.id(last_node) << " coord: " << navigator->coord_map[last_node] << " orientation: " << mine->current_orientation << endl;

	for (auto it = list_node.begin(); it != list_node.end(); ++it) {
		position board_tile = navigator->coord_map[it->first];
		if (mine->board_tile_is_painted(board_tile) && !mine->board_tile_has_booster(board_tile)) {
			continue;
		}
		string new_string = navigator->get_orientation(mine->current_orientation, it->second);
		new_string += navigator->goto_node(last_node, it->first);
		last_node = it->first;
		result += new_string;
		mine->apply_command(new_string);
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