#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "mine.h"

class agent {
   public:
	Node robot_pos;
	orientation robot_orientation;
	mine_navigator *navigator;
	int time_step;
	Graph::NodeMap<Booster> boosters_map;
	Graph::NodeMap<bool> painted_map;
	int owned_manipulators = 0;
	int owned_fast_wheels = 0;
	int owned_drill = 0;

	agent(mine_navigator *arg_navigator, Node start);
	agent(agent &ag);
	virtual ~agent();

	double get_cost();
	void step();
	void execute_seq(string seq);
	string execution_map_from_node_list(vector<pair<Node, orientation>> node_list);
	void set_current_nb_of_manipulators(int nb);
	int cost_to_next_zone(vector<vector<Node>> &zones, int zone_id);
	vector<Node> manipulators_valid_nodes();

	void paint_valid_nodes();
	vector<vector<position>> relative_manipulators;
//	string take_booster(mine_state *mine, Booster boost);

};

#endif