#include "mine.h"
#include <lemon/bfs.h>
#include <lemon/dfs.h>
#include <lemon/dijkstra.h>
#include <lemon/graph_to_eps.h>
#include <unordered_map>

#define BOARD_TILE_IS_WALL(x)                                        \
	(((x).real() < 0) || ((x).real() >= max_size_x) || ((x).imag() < 0) || \
	 ((x).imag() >= max_size_y) || board[(x).real()][(x).imag()] == WALL)
#define BOARD_TILE_IS_EMPTY(x)                                        \
	((x).real() >= 0 && (x).real() < max_size_x && (x).imag() >= 0 && \
	 (x).imag() < max_size_y && board[(x).real()][(x).imag()] == EMPTY)
#define BOARD_TILE_IS_PAINTED(x)                                      \
	((x).real() >= 0 && (x).real() < max_size_x && (x).imag() >= 0 && \
	 (x).imag() < max_size_y && board[(x).real()][(x).imag()] == PAINTED)
#define BOARD_TILE(x) board[(x).real()][(x).imag()]

static string get_next_tuple_token(string *line) {
	string token;
	string delimiter = "#";
	size_t pos = line->find(delimiter);
	token = line->substr(0, pos);
	line->erase(0, pos + delimiter.length());
	return token;
}

static string get_next_tuple_polygon(string *line) {
	string token;
	string delimiter = ";";
	size_t pos = line->find(delimiter);
	token = line->substr(0, pos);
	if (pos != line->npos) {
		line->erase(0, pos + delimiter.length());
	} else
		line->erase(0, line->length());
	return token;
}

static position parse_position(string positionstring) {
	string arg1, arg2;
	string delimiter = ",";
	size_t delimpos = positionstring.find(delimiter);
	arg1 = positionstring.substr(1, delimpos - 1);
	arg2 = positionstring.substr(delimpos + delimiter.length(),
								 positionstring.length() - delimpos - 2);
	return position(atoi(arg1.c_str()), atoi(arg2.c_str()));
}

static vector<position> parse_token_list(string line) {
	vector<position> list;
	string t_line = line;
	string token;
	string open_delimiter = "(";
	string token_delimiter = "),(";
	string close_delimiter = ")";

	size_t tokenpos = 0, tokenend;
	tokenpos = t_line.find(open_delimiter);
	tokenend = t_line.find(close_delimiter);
	while (tokenpos != t_line.npos) {
		token = t_line.substr(tokenpos, tokenend + 1);
		list.push_back(parse_position(token));
		t_line.erase(
			0, tokenend + close_delimiter.length() + open_delimiter.length());
		tokenpos = t_line.find(open_delimiter);
		tokenend = t_line.find(close_delimiter);
	}

	return list;
}

static vector<vector<position>> parse_list_of_token_lists(string line) {
	vector<vector<position>> ret;
	string t_line = line;
	while (t_line.length() > 0) {
		ret.push_back(parse_token_list(get_next_tuple_polygon(&t_line)));
	}
	return ret;
}

static bool PointInPolygon(position point, vector<position> polygon) {
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

vector<Booster> mine_navigator::boosters_in_node_list(vector<Node> zone) {
	vector<position> list_pos;
	for (auto n : zone) {
		list_pos.push_back(coord_map[n]);
	}
	return mine->boosters_in_position_list(list_pos);
}

vector<vector<Node>> mine_navigator::node_from_coords(
	vector<vector<position>> pos_list) {
	vector<vector<Node>> result;

	for (auto zone : pos_list) {
		vector<Node> result_line;
		for (auto node : zone) {
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
	if (coord.real() < 0 || coord.imag() < 0 || coord.real() >= mine->max_size_x || coord.imag() >= mine->max_size_y ) return false;
	return coord_to_node_map.find(mine->max_size_x * coord.imag() + coord.real()) != coord_to_node_map.end();
}

mine_navigator::mine_navigator(mine_state *base_mine)
	: coord_map(graph),
	  direction_map(graph),
	  length(graph),
	  boosters_map(graph) {
	int i, j;
	mine = base_mine;

	max_size_x = base_mine->max_size_x;
	for (i = 0; i < base_mine->max_size_x; i++) {
		for (j = 0; j < base_mine->max_size_y; j++) {
			if (!base_mine->board_tile_is_wall(position(i, j))) {
				Node u = graph.addNode();
				position currentpos(i, j);
				coord_map[u] = currentpos;
				if (currentpos == base_mine->robot) {
					initialNode = u;
				}
				coord_to_node_map[base_mine->max_size_x * j + i] = u;
				if (currentpos == base_mine->robot) {
					robot_pos = u;
				}
				if (find(mine->manipulators_boosters.begin(),
						 mine->manipulators_boosters.end(),
						 currentpos) != mine->manipulators_boosters.end()) {
							 boosters_map[u] = MANIPULATOR;
				} else if (find(mine->drill_boosters.begin(),
						 mine->drill_boosters.end(),
						 currentpos) != mine->drill_boosters.end()) {
							boosters_map[u] = DRILL;
				} else if (find(mine->fastwheels_boosters.begin(),
						 mine->fastwheels_boosters.end(),
						 currentpos) != mine->fastwheels_boosters.end()) {
							boosters_map[u] = FASTWHEEL;
				} else if (find(mine->mystere_boosters.begin(),
						 mine->mystere_boosters.end(),
						 currentpos) != mine->mystere_boosters.end()) {
							boosters_map[u] = MYSTERE;
				} else boosters_map[u] = NONE;
			}
		}
	}

	for (NodeIt n(graph); n != INVALID; ++n) {
		for (NodeIt v(graph); v != INVALID; ++v) {
			if (coord_map[n] ==
				position(coord_map[v].real() + 1, coord_map[v].imag())) {
				Arc e = graph.addArc(v, n);
				direction_map[e] = 'D';
				length[e] = 1;
			} else if (coord_map[n] ==
					   position(coord_map[v].real() - 1, coord_map[v].imag())) {
				Arc e = graph.addArc(v, n);
				direction_map[e] = 'A';
				length[e] = 1;
			} else if (coord_map[n] ==
					   position(coord_map[v].real(), coord_map[v].imag() - 1)) {
				Arc e = graph.addArc(v, n);
				direction_map[e] = 'S';
				length[e] = 1;
			} else if (coord_map[n] ==
					   position(coord_map[v].real(), coord_map[v].imag() + 1)) {
				Arc e = graph.addArc(v, n);
				direction_map[e] = 'W';
				length[e] = 1;
			}
		}
	}
}

vector<vector<position>> mine_navigator::list_of_coords_from_nodes(
	const vector<vector<Node>> liste) {
	vector<vector<position>> result;
	for (auto zone : liste) {
		vector<position> interm_result;
		for (auto n : zone) {
			interm_result.push_back(coord_map[n]);
		}
		result.push_back(interm_result);
	}
	return result;
}

vector<Node> mine_navigator::get_node_list() {
	vector<Node> result;
	for (NodeIt n(graph); n != INVALID; ++n) result.push_back(n);
	return result;
}

string mine_navigator::get_orientation(orientation source, orientation target) {
	string result;
	if (source == (orientation)((target + 1) % 4)) {
		result = "Q";
	} else if ((orientation)((source + 1) %4) == target) {
		result = "E";
	} else if (source != target)
		result = "QQ";
	return result;
}

string mine_navigator::goto_node(Node orig, Node target) {
	string result;
	Arc last_arc;

	if (orig == target) {
		return "";
	}

	// apply dijkstra to graph
	std::vector<Arc> arcpath;
	Dijkstra<Graph> dijkstra(graph, length);
	dijkstra.run(target, orig);

	// get back path
	Dijkstra<Graph>::Path path = dijkstra.path(orig);
	for (Dijkstra<Graph>::Path::RevArcIt it(path); it != INVALID;
		 it.operator++()) {
		result += direction_map[it];
	}
	// get all directions of the path in the result
	return result;
}



mine_state::mine_state(string filename) {
	ifstream file(filename);
	vector<position> mine_map;
	if (!file.good()) {
		cout << "cannot find file " << filename << endl;
		return;
	}

	if (file) {
		std::string token;
		std::string line;
		getline(file, line);
		// remove beginning

		mine_map = parse_token_list(get_next_tuple_token(&line));
		robot = parse_token_list(get_next_tuple_token(&line))[0];
		obstacles = parse_list_of_token_lists(get_next_tuple_token(&line));

		string boosters = get_next_tuple_token(&line);
		while (boosters.length() > 0) {
			string booster = get_next_tuple_polygon(&boosters);
			switch (booster[0]) {
				case 'X':
					booster.erase(0, 1);
					mystere_boosters.push_back(parse_position(booster));
					break;
				case 'B':
					booster.erase(0, 1);
					manipulators_boosters.push_back(parse_position(booster));
					break;
				case 'L':
					booster.erase(0, 1);
					drill_boosters.push_back(parse_position(booster));
					break;
				case 'F':
					booster.erase(0, 1);
					fastwheels_boosters.push_back(parse_position(booster));
					break;
				default:
					cout << "unhandled booster: " << booster << endl;
			}
		}
	}
	max_size_x = 0;
	max_size_y = 0;
	non_validated_tiles = 0;
	for (auto it = mine_map.begin(); it != mine_map.end(); ++it) {
		if (max_size_x < it->real()) max_size_x = it->real();
		if (max_size_y < it->imag()) max_size_y = it->imag();
	}
	board = (enum map_tile **)calloc(max_size_x, sizeof(enum map_tile *));
	for (int i = 0; i < max_size_x; i++) {
		board[i] = (enum map_tile *)calloc(max_size_y, sizeof(enum map_tile));
		for (int j = 0; j < max_size_y; j++) {
			position cur(i, j);
			if (is_point_valid(cur, &mine_map)) {
				non_validated_tiles++;
				board[i][j] = EMPTY;
			} else {
				board[i][j] = WALL;
			}
		}
	}
	/*relative_manipulators.push_back(manipulators_list[0]);
	relative_manipulators.push_back(manipulators_list[1]);
	relative_manipulators.push_back(manipulators_list[2]);
	relative_manipulators.push_back(manipulators_list[3]);*/
	// validate points
	//current_orientation = EAST;
	//vector<position> absolute_manip = absolute_valid_manipulators();
	//for (auto it:absolute_manip) {
	//	if (BOARD_TILE_IS_EMPTY(it)) {
	//		BOARD_TILE(it) = PAINTED;
	//		non_validated_tiles--;
	//	}
	//}
	/*owned_fastwheels_boosters = 0;
	owned_drill_boosters = 0;
	owned_manipulators_boosters = 0;
	time_step = 0;
	distance_loss = 0;*/
}


mine_state::mine_state(mine_state *base_mine) {
	robot = base_mine->robot;
	obstacles = base_mine->obstacles;
	mystere_boosters = base_mine->mystere_boosters;
	drill_boosters = base_mine->drill_boosters;
	fastwheels_boosters = base_mine->fastwheels_boosters;
	manipulators_boosters = base_mine->manipulators_boosters;
	max_size_x = base_mine->max_size_x;
	max_size_y = base_mine->max_size_y;
	non_validated_tiles = base_mine->non_validated_tiles;
	relative_manipulators = base_mine->relative_manipulators;
	owned_fastwheels_boosters =  base_mine->owned_fastwheels_boosters;
	owned_drill_boosters =  base_mine->owned_drill_boosters;
	owned_manipulators_boosters = base_mine->owned_manipulators_boosters;
	time_step = base_mine->time_step;
	distance_loss = base_mine->distance_loss;
	current_orientation = base_mine->current_orientation;
	board = (enum map_tile **)calloc(max_size_x, sizeof(enum map_tile *));
	for (int i = 0; i < max_size_x; i++) {
		board[i] = (enum map_tile *)calloc(max_size_y, sizeof(enum map_tile));
		memcpy(board[i], base_mine->board[i],
			   max_size_y * sizeof(enum map_tile));
	}
}

bool mine_state::is_point_valid(position point, vector<position> *mine_map) {
	bool is_valid = false;
	if (PointInPolygon(point, *mine_map)) {
		is_valid = true;
		for (auto it = obstacles.begin(); it != obstacles.end(); ++it) {
			if (PointInPolygon(point, *it)) {
				is_valid = false;
			}
		}
	}
	return is_valid;
}

bool mine_state::board_tile_has_booster(position tile) {
	bool has_manip =
		find(manipulators_boosters.begin(), manipulators_boosters.end(),
			 tile) != manipulators_boosters.end();
	return has_manip;
}

bool mine_state::board_tile_is_painted(position tile) {
	return BOARD_TILE_IS_PAINTED(tile);
}

bool mine_state::board_tile_is_wall(position tile) {
	return BOARD_TILE_IS_WALL(tile);
}

mine_state::~mine_state() {
	for (int i = 0; i < max_size_x; i++) {
		free(board[i]);
	}
	free(board);
}

vector<string> mine_state::get_next_valid_command() {
	vector<string> ret;
	position point_to_test;

	ret.push_back("Q");
	ret.push_back("E");

	if (!BOARD_TILE_IS_WALL(robot + position(1, 0))) ret.push_back("D");
	if (!BOARD_TILE_IS_WALL(robot + position(-1, 0))) ret.push_back("A");
	if (!BOARD_TILE_IS_WALL(robot + position(0, 1))) ret.push_back("W");
	if (!BOARD_TILE_IS_WALL(robot + position(0, -1))) ret.push_back("S");
	return ret;
}

vector<Booster> mine_state::boosters_in_position_list(vector<position> list) {
	vector<Booster> result;
	for (auto pos:list) {
		for (auto booster:manipulators_boosters)
			if (pos == booster)
				result.push_back(MANIPULATOR);

		for (auto booster:fastwheels_boosters)
			if (pos == booster)
				result.push_back(FASTWHEEL);

		for (auto booster:drill_boosters)
			if (pos == booster)
				result.push_back(DRILL);

		for (auto booster:mystere_boosters)
			if (pos == booster)
				result.push_back(MYSTERE);
	}
	return result;
}
