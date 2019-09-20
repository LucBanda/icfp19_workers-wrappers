#include "agent.h"
#include "complex"
#include "lemon/bfs.h"
#include "renderer.h"

const vector<vector<position>> manipulators_list = {
	{position(0, 1)}, {position(1,1)}, {position(-1, 1)}, {position(0,0)}, //those are the four default manipulators
	{position(0,-1)},
	{position (0,-2), position(0, -1)},
	{position (0,2), position(0, 1)},
	{position (0,-3), position(0, -2), position(0, -1)},
	{position (0,-4), position(0, -3), position(0,-2), position(0, -1)},
	{position (0,3), position(0, 2), position(0,1)},
	{position (0,4), position(0,3), position(0, 2), position(0,1)},
	{position (0, 5), position (0,4), position(0,3), position(0, 2), position(0,1)},
	{position(0, -5), position (0,-4), position(0, -3), position(0,-2), position(0, -1)},
	{position(2,1), position(1,1), position(1,0)},
	{position (-2,1), position(-1,1), position(-1,0)},
	{position (3,1), position(1,0), position(2,1)},
	{position (-3,1), position(-1,0), position(-2,1)},
	{position (4,1), position(1,0), position(2,0), position(2,1), position(3,1)},
	{position (-4,1), position(-1,0), position(-2,0), position(-2,1), position(-3,1)},
	{position (0, 0)},
	{position (0, 0)},
	{position (0, 0)},
	{position (0, 0)},
	{position (0, 0)},
	{position (0, 0)},
	{position (0, 0)},
	{position (0, 0)},
};

agent::agent(navigator_factory &arg_navigators, Node arg_start):
	navigators(arg_navigators),
	nav_select(navigators),
	boosters_map(arg_navigators.full_nav.graph),
	painted_map(arg_navigators.full_nav.graph, false) {

	for (Graph::NodeIt it(nav_select.base_nav->graph); it!= INVALID; ++it) {
		boosters_map[it] = nav_select.base_nav->boosters_map[it];
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
	navigators(ag.navigators),
	nav_select(ag.nav_select),
	boosters_map(nav_select.base_nav->graph),
	painted_map(nav_select.base_nav->graph, false) {
	for (Graph::NodeIt it(nav_select.base_nav->graph); it!= INVALID; ++it) {
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
	for (const auto &mask_list:relative_manipulators) {
		bool should_apply = true;
		for (const auto &mask:mask_list) {
			position mask_pos = mask * multiplier + nav_select.base_nav->coord_map[robot_pos];
			if (!nav_select.navigators.masked_nav.is_coord_in_map(mask_pos)) {
				should_apply = false;
			}
		}
		if (should_apply) {
			position valid_pos = mask_list[0] * multiplier + nav_select.base_nav->coord_map[robot_pos];
			result.push_back(nav_select.base_nav->node_from_coord(valid_pos));
		}
	}
	return result;
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

void agent::paint_valid_nodes() {
	position multiplier = pos_multiplier.at(robot_orientation);
	for (const auto &mask_list:relative_manipulators) {
		bool should_apply = true;
		Node manipulator;
		for (const auto &mask:mask_list) {
			Node node_mask = nav_select.navigators.masked_nav.node_from_coord(mask * multiplier + nav_select.base_nav->coord_map[robot_pos]);
			if (node_mask == INVALID) {
				should_apply = false;
				break;
			} else if (&mask == &mask_list.front()) {
				manipulator = node_mask;
			}
		}
		if (should_apply) {
			//position valid_pos = mask_list[0] * multiplier + navigator->coord_map[robot_pos];
			painted_map[manipulator] = true;
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
				for (int j = 0; j < (1 + (nav_select.current_mode == FAST_MODE || nav_select.current_mode == FAST_DRILL_MODE)); j++) {
					Node moving_node = nav_select.moving_nav->from_full_graph_nodes[robot_pos];
					for(Graph::OutArcIt arc(nav_select.moving_nav->graph, moving_node); arc != INVALID; ++arc) {
						if (nav_select.moving_nav->direction_map[arc] == command[i]) {
							auto arc_target = nav_select.moving_nav->graph.target(arc);
							robot_pos = nav_select.moving_nav->to_full_graph_nodes[arc_target];
						}
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
				time_step++;
				nav_select.step();
				break;
			case 'E':
				robot_orientation = (enum orientation)((robot_orientation + 1) % 4);
				//result += command[i];
				paint_valid_nodes();
				time_step++;
				nav_select.step();
				break;
			case'B': {
				int pos_x = command.find(',', i);
				int x = stoi(command.substr(i+2, pos_x));
				int pos_y = command.find(')', pos_x);
				string substr = command.substr(pos_x+1, pos_y);
				int y = stoi(substr);
				i = pos_y;
				for (const auto &boost:manipulators_list) {
					if (boost[0] == position(x,y)) {
						relative_manipulators.push_back(boost);
					}
				}
				paint_valid_nodes();
				time_step++;
				nav_select.step();
				//result += "B(" + to_string(x) + ","+ to_string(y) + ")";
				break;
			}

			case 'Q':
				robot_orientation = (enum orientation)(
					robot_orientation == NORTH ? WEST : robot_orientation - 1);
				time_step++;
				nav_select.step();
				paint_valid_nodes();
				//result += command[i];
				break;
			case 'F':
				if (owned_fast_wheels > 0) {
					nav_select.activate_boost(FASTWHEEL);
					owned_fast_wheels--;
				}
				break;
			case 'L':
				if (owned_drill > 0) {
					nav_select.activate_boost(DRILL);
					owned_drill--;
				}
				break;
		}
	}
}


int agent::cost_to_next_zone(vector<vector<Node>> &zones, int zone_id) {
	//evalutate cost to next zone
	int cost_to_next_zone = 0;

	if (zone_id < zones.size()) {
		std::vector<Arc> arcpath;
		Bfs<Graph> bfs(nav_select.navigating_nav->graph);

		Bfs<Graph>::DistMap dist(nav_select.navigating_nav->graph);
		Bfs<Graph>::PredMap predmap(nav_select.navigating_nav->graph);
		Bfs<Graph>::ProcessedMap processedmap;
		Bfs<Graph>::ReachedMap reachedmap(nav_select.navigating_nav->graph);

		bfs.distMap(dist);
		bfs.predMap(predmap);
		bfs.processedMap(processedmap);
		bfs.reachedMap(reachedmap);

		bfs.init();
		bfs.addSource(robot_pos);
		Node arrival = INVALID;
		while (!bfs.emptyQueue() && arrival == INVALID) {
			Node n = bfs.processNextNode();
			if (find(zones[zone_id].begin(), zones[zone_id].end(), n) != zones[zone_id].end()) {
					arrival = n;
					break;
			}
		}
		if (arrival != INVALID) {
			cost_to_next_zone = bfs.dist(arrival);
		}
	}

	return cost_to_next_zone;
}

string get_orientation(orientation source, orientation target) {
	string result;
	if (source == (orientation)((target + 1) % 4)) {
		result = "Q";
	} else if ((orientation)((source + 1) % 4) == target) {
		result = "E";
	} else if (source != target)
		result = "QQ";
	return result;
}

string agent::execution_map_from_node_list(vector<pair<Node, orientation>> list_node) {
	string result;
	//for each node in list
	for (const auto &it:list_node) {
		//if tile is not painted and has no manipulator
		if (painted_map[it.first] && boosters_map[it.first] == NONE) {
			continue;
		}

		//orient correctly
		string new_string = get_orientation(robot_orientation, it.second);
		result += new_string;
		execute_seq(new_string);

		//goto selected node if it is not the current position
		if (robot_pos == it.first) {
			continue;
		}

		// apply dijkstra to graph
		Bfs<Graph> bfs(nav_select.navigating_nav->graph);

		Bfs<Graph>::DistMap dist(nav_select.navigating_nav->graph);
		Bfs<Graph>::PredMap predmap(nav_select.navigating_nav->graph);
		Bfs<Graph>::ProcessedMap processedmap;
		Bfs<Graph>::ReachedMap reachedmap(nav_select.navigating_nav->graph);

		bfs.distMap(dist);
		bfs.predMap(predmap);
		bfs.processedMap(processedmap);
		bfs.reachedMap(reachedmap);

		bfs.run(robot_pos, it.first);
		auto path = bfs.path(it.first);
		vector<Arc> path_forward;
		path_forward.reserve(path.length());
		for (Bfs<Graph>::Path::RevArcIt e(path); e != INVALID; ++e) {
			SmartDigraph::Arc arc = e;
			path_forward.push_back(arc);
		}
		reverse(path_forward.begin(), path_forward.end());
		for (const auto &e:path_forward) {
			//paint the map, collect the boosters
			if (painted_map[it.first] && boosters_map[it.first] == NONE) {
				break;
			}
			const Node &result_node = nav_select.navigating_nav->graph.target(e);
			result += nav_select.navigating_nav->direction_map[e];
			robot_pos = result_node;
			time_step++;
			nav_select.step();

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