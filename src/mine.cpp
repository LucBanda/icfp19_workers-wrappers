#include "mine.h"
#include <lemon/bfs.h>
#include <lemon/dfs.h>
#include <lemon/dijkstra.h>
#include <lemon/graph_to_eps.h>
#include <iomanip>
#include "fileparser.h"


mineGraph::mineGraph(int instance):
	coord_map(graph),
	arcDirectionMap(graph),
	direction_map(graph),
	boosters_map(graph) {

}

vector<Booster> mineGraph::boosters_in_node_list(const vector<Node> &zone) {
	vector<position> list_pos;
	vector<Booster> result;
	for (const auto &n : zone) {
		if (boosters_map[n] != NONE) {
			result.push_back(boosters_map[n]);
		}
	}
	return result;
}

vector<vector<Node>> mineGraph::node_from_coords(
	vector<vector<position>> &pos_list) {
	vector<vector<Node>> result;
	for (const auto &zone : pos_list) {
		vector<Node> result_line;
		for (const auto &node : zone) {
			result_line.push_back(
				coord_to_node_map[node.imag() * max_size_x + node.real()]);
		}
		result.push_back(result_line);
	}
	return result;
}

Node mineGraph::node_from_coord(position pos) {
	if (!is_coord_in_map(pos)) return INVALID;
	return coord_to_node_map.at(pos.imag() * max_size_x + pos.real());
}

bool mineGraph::is_coord_in_map(position coord) {
	if (coord.real() < 0 || coord.imag() < 0 || coord.real() >= max_size_x ||
		coord.imag() >= max_size_y)
		return false;
	return coord_to_node_map.find(max_size_x * coord.imag() + coord.real()) !=
		   coord_to_node_map.end();
}

vector<vector<position>> mineGraph::list_of_coords_from_nodes(
	const vector<vector<Node>> &liste) {
	vector<vector<position>> result;
	for (const auto &zone : liste) {
		vector<position> interm_result;
		for (const auto &n : zone) {
			interm_result.push_back(coord_map[n]);
		}
		result.push_back(interm_result);
	}
	return result;
}


static inline bool PointInPolygon(position point, vector<position> polygon) {
	int i, j, nvert = polygon.size();
	bool c = false;

	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((polygon[i].imag() > point.imag()) !=
			 (polygon[j].imag() > point.imag())) &&
			(point.real() < (polygon[j].real() - polygon[i].real()) *
									(point.imag() - polygon[i].imag()) /
									(polygon[j].imag() - polygon[i].imag()) +
								polygon[i].real()))
			c = !c;
	}

	return c;
}

static inline bool is_point_valid(position point, vector<position> &mine_map, vector<vector<position>> &obstacles) {
	bool is_valid = false;
	if (PointInPolygon(point, mine_map)) {
		is_valid = true;
		for (const auto &it:obstacles) {
			if (PointInPolygon(point, it)) {
				is_valid = false;
			}
		}
	}
	return is_valid;
}


void mineGraph::create_masked_nodes(mine_parser &parser) {
	for (int i = 0; i < parser.max_size_x; i++) {
		for (int j = 0; j < parser.max_size_y; j++) {
			if (is_point_valid(position(i, j), parser.mine_map, parser.obstacles)) {
				Node u = graph.addNode();
				position currentpos(i, j);
				coord_map[u] = currentpos;
				if (currentpos == parser.robotPos) {
					initialNode = u;
				}
				coord_to_node_map[max_size_x * j + i] = u;
				//cout << "(" << i << ", " << j << "):" << graph.id(u) << endl;
				boosters_map[u] = NONE;
				for (const auto &boosterIt : parser.boosters) {
					const Booster &boost = boosterIt.first;
					const vector<position> &boostPos = boosterIt.second;
					if (find(boostPos.begin(), boostPos.end(), currentpos) !=
						boostPos.end()) {
						boosters_map[u] = boost;
						boosters_lists[boost].push_back(u);
					}
				}
			}
		}
	}
}

void mineGraph::create_full_nodes(mine_parser &parser) {
	for (int i = 0; i < parser.max_size_x; i++) {
		for (int j = 0; j < parser.max_size_y; j++) {
			Node u = graph.addNode();
			position currentpos(i, j);
			coord_map[u] = currentpos;
			if (currentpos == parser.robotPos) {
				initialNode = u;
			}
			coord_to_node_map[max_size_x * j + i] = u;
			//cout << "(" << i << ", " << j << "):" << graph.id(u) << endl;
			boosters_map[u] = NONE;
			for (const auto &boosterIt : parser.boosters) {
				const Booster &boost = boosterIt.first;
				const vector<position> &boostPos = boosterIt.second;
				if (find(boostPos.begin(), boostPos.end(), currentpos) !=
					boostPos.end()) {
					boosters_map[u] = boost;
					boosters_lists[boost].push_back(u);
				}
			}
		}
	}
}

void mineGraph::create_single_arcs(mine_parser &parser, bool fullnodes) {
	for (NodeIt n(graph); n != INVALID; ++n) {
		for (const auto &directionElem:direction_code) {
			position pos = coord_map[n] + directionElem.second;
			if (is_coord_in_map(pos) || (fullnodes && (parser.max_size_x > pos.real() && pos.real() >= 0 && parser.max_size_y > pos.imag() && pos.imag() >= 0))) {
				Arc e = graph.addArc(n, node_from_coord(pos));
				direction_map[e] = directionElem.first;
				arcDirectionMap[n][direction_map[e]] = e;
				//cout << "arc " << graph.id(e)<< ": " << graph.id(n) << "->" << graph.id(node_from_coord(pos)) << endl;
			}
		}
	}
}

void mineGraph::create_double_arcs(mine_parser &parser, bool fullnodes) {
	for (NodeIt n(graph); n != INVALID; ++n) {
		for (const auto &directionElem:direction_code) {
			position pos2 = coord_map[n] + directionElem.second * 2;
			position pos1 = coord_map[n] + directionElem.second;
			if ((is_coord_in_map(pos2) && is_coord_in_map(pos1))  || (fullnodes && (parser.max_size_x > pos2.real() && pos2.real() >= 0 && parser.max_size_y > pos2.imag() && pos2.imag() >= 0))) {
				Arc e = graph.addArc(n, node_from_coord(pos2));
				direction_map[e] = directionElem.first;
				arcDirectionMap[n][direction_map[e]] = e;
				//cout << "arc(*2) " << graph.id(e)<< ": "<< graph.id(n) << "->" << graph.id(node_from_coord(pos2)) << endl;
			} else if (is_coord_in_map(pos1) || (fullnodes && (parser.max_size_x > pos1.real() && pos1.real() >= 0 && parser.max_size_y > pos1.imag() && pos1.imag() >= 0))) {
				Arc e = graph.addArc(n, node_from_coord(pos1));
				direction_map[e] = directionElem.first;
				arcDirectionMap[n][direction_map[e]] = e;
				//cout << "arc " << graph.id(e)<< ": " << graph.id(n) << "->" << graph.id(node_from_coord(pos1)) << endl;
			}
		}
	}
}

masked_navigator::masked_navigator(mine_parser &parser)
	: mineGraph(parser.instance)
 {
	 name = "masked navigator";
	 max_size_x = parser.max_size_x;
	 max_size_y = parser.max_size_y;
	create_masked_nodes(parser);
	create_single_arcs(parser, false);
}

full_navigator::full_navigator(mine_parser &parser)
	: mineGraph(parser.instance)
 {
	 name = "full navigator";
	 max_size_x = parser.max_size_x;
	 max_size_y = parser.max_size_y;
	create_full_nodes(parser);
	create_single_arcs(parser, true);
}


fast_navigator::fast_navigator(mine_parser &parser):
mineGraph(parser.instance) {
	name = "fast navigator";
	 max_size_x = parser.max_size_x;
	 max_size_y = parser.max_size_y;
	create_masked_nodes(parser);
	create_double_arcs(parser, false);
}

fast_full_navigator::fast_full_navigator(mine_parser &parser):
mineGraph(parser.instance){
	name = "fast full navigator";
	 max_size_x = parser.max_size_x;
	 max_size_y = parser.max_size_y;
	create_full_nodes(parser);
	create_double_arcs(parser, true);
}


navigator_selector::navigator_selector(navigator_factory &arg_navigators):
	navigators(arg_navigators) {
	current_mode = STANDARD_MODE;
	navigating_nav = &navigators.masked_nav;
	moving_nav = &navigators.masked_nav;
	base_nav = &navigators.full_nav;
}

void navigator_selector::activate_boost(enum Booster booster) {
	switch(booster) {
	case FASTWHEEL:
		if (current_mode == DRILL_MODE) {
			current_mode = FAST_DRILL_MODE;
			navigating_nav = &navigators.fast_full_nav;
			moving_nav = &navigators.full_nav;
		} else {
			current_mode = FAST_MODE;
			navigating_nav = &navigators.fast_nav;
			moving_nav = &navigators.masked_nav;
		}
		fast_mode_step_left += 50;
		break;
	case DRILL:
		if (current_mode == FAST_MODE) {
			current_mode = FAST_DRILL_MODE;
			navigating_nav = &navigators.fast_full_nav;
			moving_nav = &navigators.full_nav;
		} else {
			current_mode = DRILL_MODE;
			navigating_nav = &navigators.full_nav;
			moving_nav = &navigators.full_nav;
		}
		drill_mode_step_left += 30;
		break;
	default:
		break;
	}
}

void navigator_selector::step() {
	navigator_changed = false;
	if (fast_mode_step_left > 0) {
		fast_mode_step_left--;
	}
	if (drill_mode_step_left > 0) {
		drill_mode_step_left--;
	}
	if (fast_mode_step_left > 0 && drill_mode_step_left > 0) {
		if (current_mode != FAST_DRILL_MODE) navigator_changed = true;
		current_mode = FAST_DRILL_MODE;
		moving_nav = &navigators.full_nav;
		navigating_nav = &navigators.fast_full_nav;
	} else if (fast_mode_step_left > 0) {
		if (current_mode != FAST_MODE) navigator_changed = true;
		current_mode = FAST_MODE;
		moving_nav = &navigators.masked_nav;
		navigating_nav = &navigators.fast_nav;
	} else if (drill_mode_step_left > 0) {
		if (current_mode != DRILL_MODE) navigator_changed = true;
		current_mode = DRILL_MODE;
		moving_nav = &navigators.full_nav;
		navigating_nav = &navigators.full_nav;
	} else {
		if (current_mode != STANDARD_MODE) navigator_changed = true;
		current_mode = STANDARD_MODE;
		moving_nav = &navigators.masked_nav;
		navigating_nav = &navigators.masked_nav;
	}
}


navigator_factory::navigator_factory(int arg_instance):
	parser(arg_instance),
	masked_nav(parser),
	full_nav(parser),
	fast_nav(parser),
	fast_full_nav(parser) {

	instance = arg_instance;

	for (NodeIt full_node(full_nav.graph); full_node != INVALID; ++full_node) {

		position full_coord = full_nav.coord_map[full_node];
		if (masked_nav.is_coord_in_map(full_coord)) {
			Node main_node = masked_nav.node_from_coord(full_coord);
			Node fast_node = fast_nav.node_from_coord(full_coord);
			masked_nav.to_full_graph_nodes[main_node] = full_node;
			masked_nav.from_full_graph_nodes[full_node] = main_node;
			fast_nav.to_full_graph_nodes[fast_node] = full_node;
			fast_nav.from_full_graph_nodes[full_node] = fast_node;
		} else {
			//masked_nav.to_full_graph_nodes[main_node] = full_node;
			masked_nav.from_full_graph_nodes[full_node] = INVALID;
			fast_nav.from_full_graph_nodes[full_node] = INVALID;
		}
		Node fast_drill_node = fast_full_nav.node_from_coord(full_nav.coord_map[full_node]);

		full_nav.to_full_graph_nodes[full_node] = full_node;
		full_nav.from_full_graph_nodes[full_node] = full_node;

		fast_full_nav.to_full_graph_nodes[fast_drill_node] = full_node;
		fast_full_nav.from_full_graph_nodes[full_node] = fast_drill_node;
	}
}

navigator_factory::~navigator_factory() {
};