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
	orientation orient_start;
	Node last_node;
	//vector<vector<Node>> zones;
	//int zone_id;

	agent(mine_state *arg_mine, mine_navigator *arg_navigator, Node start);
	virtual ~agent();

	double get_cost();
	void step();
	void set_execution_map(string map);
	string execution_map_from_node_list(vector<pair<Node, orientation>> node_list);

	int run();
	//enum orientation current_orientation = EAST;

   protected:
	double initial_targets;
	string execution_map;
	mine_navigator *navigator;

	//int max_time_step;
};

#endif