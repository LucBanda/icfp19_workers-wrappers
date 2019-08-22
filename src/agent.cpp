#include "agent.h"
#include "complex"

agent::agent(mine_state *arg_mine) {
	mine = arg_mine;
}

agent::~agent() {}

void agent::set_execution_map(string exec) {
	execution = exec;
}

int agent::run() {
	mine->apply_command(execution);
	return 0;
}

int agent::get_cost() {
	int score = mine->non_validated_tiles.size() * 2000 + mine->distance_loss + mine->time_step;
	return score;
}