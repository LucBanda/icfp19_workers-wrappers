#include "agent.h"
#include "complex"
#include "lemon/dijkstra.h"
#include "renderer.h"

static const vector<vector<position>> manipulators_list = {
	{position(0, 1)}, {position(1,1)}, {position(-1, 1)}, {position(0,0)}, //those are the four default manipulators
	{position(0,-1)},
	{position (0,2), position(0, 1)},
	{position (0,-2), position(0, -1)},
	{position (0,3), position(0, 2), position(0,1)},
	{position (0,-3), position(0, -2), position(0, -1)},
	{position (0,4), position(0,3), position(0, 2), position(0,1)},
	{position (0,-4), position(0, -3), position(0,-2), position(0, -1)},
	{position (0, 5), position (0,4), position(0,3), position(0, 2), position(0,1)},
	{position(0, -5), position (0,-4), position(0, -3), position(0,-2), position(0, -1)},
	{position(2,1), position(1,1), position(1,0)},
	{position (-2,1), position(-1,1), position(-1,0)},
	{position (3,1), position(1,0), position(2,1)},
	{position (-3,1), position(-1,0), position(-2,1)},
	{position (4,1), position(1,0), position(2,0), position(2,1), position(3,1)},
	{position (-4,1), position(-1,0), position(-2,0), position(-2,1), position(-3,1)},

};

agent::agent(mine_navigator *arg_navigator, Node arg_start):
	boosters_map(arg_navigator->graph),
	painted_map(arg_navigator->graph, false) {
	navigator = arg_navigator;
	for (Graph::NodeIt it(navigator->graph); it!= INVALID; ++it) {
		boosters_map[it] = arg_navigator->boosters_map[it];
	}
	robot_orientation = EAST;
	robot_pos = arg_start;
	owned_manipulators = 0;
	owned_fast_wheels = 0;
	owned_drill = 0;
	time_step = 0;
	relative_manipulators.push_back(manipulators_list[0]);
	relative_manipulators.push_back(manipulators_list[1]);
	relative_manipulators.push_back(manipulators_list[2]);
	relative_manipulators.push_back(manipulators_list[3]);
	paint_valid_nodes();
}

agent::agent(agent &ag):
	boosters_map(ag.navigator->graph),
	painted_map(ag.navigator->graph, false) {
	navigator = ag.navigator;
	for (Graph::NodeIt it(navigator->graph); it!= INVALID; ++it) {
		boosters_map[it] = ag.boosters_map[it];
		painted_map[it] = ag.painted_map[it];
	}
	robot_orientation = ag.robot_orientation;
	robot_pos = ag.robot_pos;
	owned_manipulators = ag.owned_manipulators;
	owned_fast_wheels = ag.owned_fast_wheels;
	owned_drill = ag.owned_drill;
	time_step = ag.time_step;
	relative_manipulators = ag.relative_manipulators;
}

agent::~agent() {}

static const map<orientation, position> pos_multiplier= {
	{NORTH, position(1,0)},
	{SOUTH, position(-1,0)},
	{WEST, position(0,1)},
	{EAST, position(0,-1)}
};

vector<Node> agent::manipulators_valid_nodes() {
	vector<Node> result;
	position multiplier = pos_multiplier.at(robot_orientation);
	for (auto mask_list:relative_manipulators) {
		bool should_apply = true;
		for (auto mask:mask_list) {
			position mask_pos = mask * multiplier + navigator->coord_map[robot_pos];
			if (!navigator->is_coord_in_map(mask_pos)) {
				should_apply = false;
			}
		}
		if (should_apply) {
			position valid_pos = mask_list[0] * multiplier + navigator->coord_map[robot_pos];
			result.push_back(navigator->node_from_coord(valid_pos));
		}
	}
	return result;
}

void agent::paint_valid_nodes() {
	position multiplier = pos_multiplier.at(robot_orientation);
	for (auto mask_list:relative_manipulators) {
		bool should_apply = true;
		for (auto mask:mask_list) {
			position mask_pos = mask * multiplier + navigator->coord_map[robot_pos];
			if (!navigator->is_coord_in_map(mask_pos)) {
				should_apply = false;
			}
		}
		if (should_apply) {
			position valid_pos = mask_list[0] * multiplier + navigator->coord_map[robot_pos];
			painted_map[navigator->node_from_coord(valid_pos)] = true;
		}
	}
}

void agent::execute_seq(string command) {
	for (unsigned int i = 0; i < command.length(); i++) {
		switch (command[i]) {
			case 'W':
			case 'S':
			case 'A':
			case 'D':
				for(Graph::OutArcIt arc(navigator->graph, robot_pos); arc != INVALID; ++arc) {
					if (navigator->direction_map[arc] == command[i]) {
						auto arc_target = navigator->graph.target(arc);
						robot_pos = arc_target;
					}
				}
				time_step++;
				break;
			case 'E':
				robot_orientation = (enum orientation)((robot_orientation + 1) % 4);
				//result += command[i];
				time_step++;
				break;
			case'B': {
				int pos_x = command.find(',', i);
				int x = stoi(command.substr(i+2, pos_x));
				int pos_y = command.find(')', pos_x);
				string substr = command.substr(pos_x+1, pos_y);
				int y = stoi(substr);
				i = pos_y;
				for (auto boost:manipulators_list) {
					if (boost[0] == position(x,y)) {
						relative_manipulators.push_back(boost);
					}
				}
				//result += "B(" + to_string(x) + ","+ to_string(y) + ")";
				break;
			}

			case 'Q':
				robot_orientation = (enum orientation)(
					robot_orientation == NORTH ? WEST : robot_orientation - 1);
				time_step++;
				//result += command[i];
				break;
		}
		Booster boost = boosters_map[robot_pos];
		if (boost != NONE) {
			boosters_map[robot_pos] = NONE;
			switch (boost) {
			case MANIPULATOR: {
				owned_manipulators ++;
				break;
			}
			case DRILL:
				owned_drill ++;
				break;
			case FASTWHEEL:
				owned_fast_wheels ++;
				break;
			default:
				break;
			}
		}
		paint_valid_nodes();
	}
}


void agent::set_current_nb_of_manipulators(int nb) {
	relative_manipulators.clear();
	relative_manipulators.push_back(manipulators_list[0]);
	relative_manipulators.push_back(manipulators_list[1]);
	relative_manipulators.push_back(manipulators_list[2]);
	relative_manipulators.push_back(manipulators_list[3]);
	for (int i = 4; i < nb; i++) {
		relative_manipulators.push_back(manipulators_list[i]);
		owned_manipulators = nb;
	}
}

string agent::execution_map_from_node_list(vector<pair<Node, orientation>> list_node) {
	string result;
	//for each node in list
	for (auto it = list_node.begin(); it != list_node.end(); ++it) {
		//if tile is not painted and has no manipulator
		if (painted_map[it->first] && boosters_map[it->first] == NONE) {
			continue;
		}

		//orient correctly
		string new_string = navigator->get_orientation(robot_orientation, it->second);
		time_step += new_string.length();
		result += new_string;
		robot_orientation = it->second;

		//goto selected node if it is not the current position
		Arc last_arc;
		if (robot_pos == it->first) {
			continue;
		}

		// apply dijkstra to graph
		Dijkstra<Graph> dijkstra(navigator->graph, navigator->length);
		dijkstra.run(robot_pos, it->first);
		auto path = dijkstra.path(it->first);
		vector<Arc> path_forward;
		path_forward.reserve(path.length());
		for (Dijkstra<Graph>::Path::RevArcIt e(path); e != INVALID; ++e) {
			SmartDigraph::Arc arc = e;
			path_forward.push_back(arc);
		}
		reverse(path_forward.begin(), path_forward.end());
		for (auto e:path_forward) {
			//paint the map, collect the boosters
			result += navigator->direction_map[e];
			robot_pos = navigator->graph.target(e);
			time_step++;

			Booster boost = boosters_map[robot_pos];
			if (boost != NONE) {
				boosters_map[robot_pos] = NONE;
				switch (boost) {
				case MANIPULATOR: {
					owned_manipulators ++;
					vector<position> additionnal_manipulator = manipulators_list[owned_manipulators+3];
					relative_manipulators.push_back(additionnal_manipulator);
					result += "B(" + to_string(additionnal_manipulator[0].real()) + ","+ to_string(additionnal_manipulator[0].imag()) + ")";
					break;
				}
				case DRILL:
					owned_drill ++;
					break;
				case FASTWHEEL:
					owned_fast_wheels ++;
					break;
				default:
					break;
				}
			}
			paint_valid_nodes();
		}
	}
	return result;
}

double agent::get_cost() {
	double score = time_step;
	return score;
}