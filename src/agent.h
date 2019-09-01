#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "mine.h"

class agent {
   public:
	mine_state *mine;
	string execution;

	agent(mine_state *arg_mine, mine_navigator *arg_navigator);
	virtual ~agent();

	double get_cost();
	void step();
	void set_execution_map(string map);
	string execution_map_from_node_list(vector<ListDigraph::Node> node_list);

	int run();
	enum orientation current_orientation = EAST;

   protected:
	double initial_targets;
	string execution_map;
	mine_navigator *navigator;

	//int max_time_step;
};

#endif