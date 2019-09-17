#include "mine.h"
#include <lemon/bfs.h>
#include <lemon/dfs.h>
#include <lemon/dijkstra.h>
#include <lemon/graph_to_eps.h>
#include <iomanip>

static const map<char, Booster> booster_code = {
	{'X', MYSTERE},   {'B', MANIPULATOR}, {'L', DRILL},
	{'F', FASTWHEEL}, {'C', CLONE},		  {'R', TELEPORT}};

static const map<char, position> direction_code = {
	{'W', position(0,1)},
	{'A', position(-1, 0)},
	{'D', position(1, 0)},
	{'S', position(0, -1)}
};

mine_navigator::mine_navigator(int arg_instance)
	: coord_map(graph),
	  direction_map(graph),
	  boosters_map(graph) {
	instance = arg_instance;
	vector<position> mine_map;
	vector<vector<position>> obstacles;
	position robotPos;
	std::map<Booster, vector<position>> boosters;
	ostringstream filename;

	filename << "./part-1-initial/prob-" << setw(3) << setfill('0') << instance
			 << ".desc";

	ifstream file(filename.str());
	if (!file.good()) {
		cout << "cannot find file " << filename.str() << endl;
		return;
	}

	std::string line;
	getline(file, line);

	// remove beginning
	// parse the file
	mine_map = parse_token_list(get_next_tuple_token(&line));
	robotPos = parse_token_list(get_next_tuple_token(&line))[0];
	obstacles = parse_list_of_token_lists(get_next_tuple_token(&line));

	string boosters_str = get_next_tuple_token(&line);
	while (boosters_str.length() > 0) {
		string booster_str = get_next_tuple_polygon(&boosters_str);
		char code = booster_str[0];
		booster_str.erase(0, 1);
		boosters[booster_code.at(code)].push_back(parse_position(booster_str));
	}

	// init variables and graphs
	max_size_x = 0;
	max_size_y = 0;
	for (const auto &it:mine_map) {
		if (max_size_x < it.real()) max_size_x = it.real();
		if (max_size_y < it.imag()) max_size_y = it.imag();
	}

	//create graph nodes
	for (int i = 0; i < max_size_x; i++) {
		for (int j = 0; j < max_size_y; j++) {
			if (is_point_valid(position(i, j), mine_map, obstacles)) {
				Node u = graph.addNode();
				position currentpos(i, j);
				coord_map[u] = currentpos;
				if (currentpos == robotPos) {
					initialNode = u;
				}
				coord_to_node_map[max_size_x * j + i] = u;

				boosters_map[u] = NONE;
				for (const auto &boosterIt : boosters) {
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

	//create graph arcs
	for (NodeIt n(graph); n != INVALID; ++n) {
		for (const auto &directionElem:direction_code) {
			position pos = coord_map[n] + directionElem.second;
			if (is_coord_in_map(pos)) {
				Arc e = graph.addArc(n, node_from_coord(pos));
				direction_map[e] = directionElem.first;
			}
		}
	}
}


vector<Booster> mine_navigator::boosters_in_node_list(const vector<Node> &zone) {
	vector<position> list_pos;
	vector<Booster> result;
	for (const auto &n : zone) {
		if (boosters_map[n] != NONE) {
			result.push_back(boosters_map[n]);
		}
	}
	return result;
}

vector<vector<Node>> mine_navigator::node_from_coords(
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

Node mine_navigator::node_from_coord(position pos) {
	if (!is_coord_in_map(pos)) return INVALID;
	return coord_to_node_map.at(pos.imag() * max_size_x + pos.real());
}

bool mine_navigator::is_coord_in_map(position coord) {
	if (coord.real() < 0 || coord.imag() < 0 || coord.real() >= max_size_x ||
		coord.imag() >= max_size_y)
		return false;
	return coord_to_node_map.find(max_size_x * coord.imag() + coord.real()) !=
		   coord_to_node_map.end();
}

vector<vector<position>> mine_navigator::list_of_coords_from_nodes(
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

string mine_navigator::get_orientation(orientation source, orientation target) {
	string result;
	if (source == (orientation)((target + 1) % 4)) {
		result = "Q";
	} else if ((orientation)((source + 1) % 4) == target) {
		result = "E";
	} else if (source != target)
		result = "QQ";
	return result;
}