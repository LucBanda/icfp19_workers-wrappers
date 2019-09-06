#include "mine.h"
#include <lemon/dijkstra.h>
#include <lemon/graph_to_eps.h>
#include <lemon/bfs.h>
#include <lemon/dfs.h>

#define BOARD_TILE_IS_WALL(x)		((x).real() < 0 || (x).real() >= max_size_x || (x).imag() < 0 || (x).imag() >= max_size_y || board[(x).real()][(x).imag()] == WALL)
#define BOARD_TILE_IS_EMPTY(x)		((x).real() >= 0 && (x).real() < max_size_x && (x).imag() >= 0 && (x).imag() < max_size_y &&  board[(x).real()][(x).imag()] == EMPTY)
#define BOARD_TILE_IS_PAINTED(x)	((x).real() >= 0 && (x).real() < max_size_x && (x).imag() >= 0 && (x).imag() < max_size_y && board[(x).real()][(x).imag()] == PAINTED)
#define BOARD_TILE(x) 				board[(x).real()][(x).imag()]

static string get_next_tuple_token(string *line) {
	string token;
	string delimiter = "#";
	size_t pos = line->find(delimiter);
	token = line->substr(0, pos);
	line->erase(0, pos + delimiter.length());
	//cout << "get next tuple " << token << ", " << *line << endl;
	return token;
}

static string get_next_tuple_polygon(string *line) {
	string token;
	string delimiter = ";";
	size_t pos = line->find(delimiter);
	token = line->substr(0, pos);
	if (pos != line->npos) {
		line->erase(0, pos + delimiter.length());
	} else line->erase(0, line->length());
	return token;
}

static position parse_position(string positionstring) {
	string arg1, arg2;
	string delimiter = ",";
	size_t delimpos = positionstring.find(delimiter);
	arg1 = positionstring.substr(1, delimpos - 1);
	arg2 = positionstring.substr(delimpos + delimiter.length(), positionstring.length() - delimpos - 2);
	//cout << "args from position " << arg1 << ", " << arg2 << endl;
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
		//cout << token << endl;
		list.push_back(parse_position(token));
		t_line.erase(0, tokenend + close_delimiter.length() + open_delimiter.length());
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

  for(i = 0, j = nvert - 1; i < nvert; j = i++) {
    if( ( (polygon[i].imag() > point.imag() ) != (polygon[j].imag() > point.imag()) ) &&
        (point.real() < (polygon[j].real() - polygon[i].real()) * (point.imag() - polygon[i].imag()) / (polygon[j].imag() - polygon[i].imag()) + polygon[i].real())
      )
      c = !c;
  }

  return c;
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
	owned_fastwheels_boosters = 0;
	owned_drill_boosters = 0;
	owned_manipulators_boosters = 0;
	time_step = 0;
	distance_loss = 0;
	current_orientation = EAST;
	board = (enum map_tile **) calloc(max_size_x, sizeof(enum map_tile *));
	for (int i = 0; i < max_size_x; i++) {
		board[i] = (enum map_tile *) calloc(max_size_y, sizeof(enum map_tile));
		memcpy(board[i], base_mine->board[i], max_size_y * sizeof(enum map_tile));
	}
	/*digraphCopy(base_mine->graph, graph).nodeMap(base_mine->coord_map, coord_map).arcMap(base_mine->length, length).arcMap(base_mine->direction_map, direction_map).arcMap(base_mine->orientation_map, orientation_map).run();
	initialNode = base_mine->initialNode;*/
}

bool mine_state::board_tile_is_painted(position tile) {
	return BOARD_TILE_IS_PAINTED(tile);
}

bool mine_state::board_tile_is_wall(position tile) {
	return BOARD_TILE_IS_WALL(tile);
}

vector<position> mine_state::absolute_manipulators() {
	vector<position> result;

	for (auto &it:relative_manipulators) {
		switch(current_orientation) {
		case NORTH:
			result.push_back(it);
			break;
		case SOUTH:
			result.push_back(-it);
			break;
		case WEST:
			result.emplace_back(-it.imag(), it.real());
			break;
		case EAST:
			result.emplace_back(it.imag(), -it.real());
			break;
		default:
			break;
		}
	}
	return result;
}

typedef dim2::Point<int> Point;

mine_navigator::mine_navigator(mine_state *base_mine): coord_map(graph), direction_map(graph), orientation_map(graph), length(graph), ordered_node_map(graph)  {
	int i,j;
	Graph::NodeMap<Point> coords(graph);
	Graph::NodeMap<double> sizes(graph);
	Graph::ArcMap<int> acolors(graph);
	Palette palette;
 	Palette paletteW(true);
	for (i = 0; i < base_mine->max_size_x; i++) {
		for (j = 0; j < base_mine->max_size_y; j++) {
			if (!base_mine->board_tile_is_wall(position(i,j))) {
				Node u = graph.addNode();
				position currentpos(i,j);
				coord_map[u] = currentpos;
				if (currentpos == base_mine->robot) {
					initialNode = u;
				}
				coords[u] = Point(i,j);
				sizes[u] = .3;
			}
		}
	}

	for (NodeIt n(graph); n != INVALID; ++n) {
		for (NodeIt v(graph); v != INVALID; ++v) {
			if (coord_map[n] == position(coord_map[v].real()  + 1, coord_map[v].imag())) {
				Arc e = graph.addArc(n,v);
				direction_map[e] = 'D';
				orientation_map[e] = EAST;
				length[e] = 1;
				acolors[e] = 0;
			} else if (coord_map[n] == position(coord_map[v].real()  - 1, coord_map[v].imag())) {
				Arc e = graph.addArc(n,v);
				direction_map[e] = 'A';
				orientation_map[e] = WEST;
				length[e] = 1;
				acolors[e] = 1;
			} else if (coord_map[n] == position(coord_map[v].real(), coord_map[v].imag() - 1)) {
				Arc e = graph.addArc(n,v);
				direction_map[e] = 'S';
				orientation_map[e] = SOUTH;
				length[e] = 1;
				acolors[e] = 2;
			} else if (coord_map[n] == position(coord_map[v].real(), coord_map[v].imag() + 1)) {
				Arc e = graph.addArc(n,v);
				direction_map[e] = 'W';
				orientation_map[e] = NORTH;
				length[e] = 1;
				acolors[e] = 3;
			}
		}
	}
}

vector<vector<position>> mine_navigator::list_of_coords_from_nodes(const vector<vector<Node>> liste) {
	vector<vector<position>> result;
	for (auto zone:liste) {
		vector<position> interm_result;
		for (auto n:zone) {
			interm_result.push_back(coord_map[n]);
		}
		result.push_back(interm_result);
	}
	return result;
}
vector<Node> mine_navigator::get_node_list() {
	vector<Node> result;
	for (NodeIt n(graph); n != INVALID; ++n)
		result.push_back(n);
	return result;
}

void mine_navigator::init_ordered_map() {
	vector<Node> list_node = get_node_list();

	for (int it = 0; it < (int)list_node.size(); it++) {
		Bfs<Graph> dfs(graph);
		vector<Node> result;
		dfs.init();
		dfs.addSource(list_node[it]);
		int depth = 3000;
		//bfs.addSource(list_node[it]);
		while (!dfs.emptyQueue() && depth--) {
			Node v = dfs.processNextNode();
			if (find(result.begin(), result.end(), v) == result.end())
				result.push_back(v);
		}
		ordered_node_map[list_node[it]] = result;
	}
}

vector<Node> mine_navigator::get_bfs_from_node(Node start, int depth) {
	vector<Node> result;
	result = ordered_node_map[start];
	if (depth < (int)result.size())
		result.erase(result.begin() + depth, result.end());
	return result;
}

string mine_navigator::goto_node(enum orientation source_orientation,enum orientation &last_orientation, Node orig, Node target, Node *ending_node) {
	string result;
	string last_car = "";
	Arc last_arc;

	if (orig == target){
		for (Graph::InArcIt a(graph, orig); a != INVALID; ++a) {
			char dir = direction_map[a];
			result += dir;
			if (dir == 'A') {
				last_car = "D";
				last_orientation = EAST;
			}
			else if (dir == 'D') {
				last_car = "A";
				last_orientation = WEST;
			}
			else if (dir == 'W') {
				last_car = "S";
				last_orientation = SOUTH;
			}
			else if (dir == 'S') {
				last_car = "W";
				last_orientation = NORTH;
			}
			*ending_node = graph.source(a);
			break;
		}

	} else {
		//apply dijkstra to graph
		std::vector<Arc> arcpath;
		Dijkstra<Graph> dijkstra(graph, length);
		dijkstra.run(target, orig);

		//get back path
		Dijkstra<Graph>::Path path = dijkstra.path(orig);
		for(Dijkstra<Graph>::Path::RevArcIt it(path); it!=INVALID ; it.operator++() ) {
			result += direction_map[it];
			last_orientation = orientation_map[it];
			*ending_node = graph.target(it);
		}

		//get node n-1 of the path
		last_car = "";
		if (result.size() > 0) {
			last_car = result.substr(result.size()-1, result.size());
		}
		result.pop_back();
	}
	//mine->apply_command(result);
	// orient corrctly;
	if ((last_orientation == NORTH && source_orientation == SOUTH)
		|| (last_orientation == SOUTH && source_orientation == NORTH)
		|| (last_orientation == EAST && source_orientation == WEST)
		|| (last_orientation == WEST && source_orientation == EAST)) {
			result += "QQ";
	} else if ((last_orientation == NORTH && source_orientation == EAST)
		|| (last_orientation == EAST && source_orientation == SOUTH)
		|| (last_orientation == SOUTH && source_orientation == WEST)
		|| (last_orientation == WEST && source_orientation == NORTH)) {
			result += "Q";
	} else if ((last_orientation == NORTH && source_orientation == WEST)
		|| (last_orientation == WEST && source_orientation == SOUTH)
		|| (last_orientation == SOUTH && source_orientation == EAST)
		|| (last_orientation == EAST && source_orientation == NORTH)) {
			result += "E";
	}
	//result+=last_car;

	//get all directions of the path in the result
	return result;
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


mine_state::mine_state(string filename) {
	ifstream file(filename);
	vector<position> mine_map;
    if (!file.good()) {
		cout << "cannot find file " << filename <<endl;
		return;
	}

	if (file) {
		std::string token;
		std::string line;
		getline(file, line);
		//cout << line << endl;
		// remove beginning

		mine_map = parse_token_list(get_next_tuple_token(&line));
		//cout << "########################################"<< endl;
		robot = parse_token_list(get_next_tuple_token(&line))[0];
		//cout << "########################################"<< endl;
		obstacles = parse_list_of_token_lists(get_next_tuple_token(&line));

		string boosters = get_next_tuple_token(&line);
		while (boosters.length() > 0){
			string booster = get_next_tuple_polygon(&boosters);
			switch (booster[0]){
			case 'X':
				booster.erase(0,1);
				mystere_boosters.push_back(parse_position(booster));
				break;
			case 'B':
				booster.erase(0,1);
				manipulators_boosters.push_back(parse_position(booster));
				break;
			case 'L':
				booster.erase(0,1);
				drill_boosters.push_back(parse_position(booster));
				break;
			case 'F':
				booster.erase(0,1);
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
	board = (enum map_tile **) calloc(max_size_x, sizeof(enum map_tile *));
	for (int i = 0; i < max_size_x; i++) {
		board[i] = (enum map_tile *) calloc(max_size_y, sizeof(enum map_tile));
		for (int j = 0; j < max_size_y; j++) {
			position cur(i,j);
			if (is_point_valid(cur, &mine_map)) {
				non_validated_tiles++;
				board[i][j] = EMPTY;
			} else {
				board[i][j] = WALL;
			}
		}
	}
	relative_manipulators.push_back(position(0, 1));
	relative_manipulators.push_back(position(1, 1));
	relative_manipulators.push_back(position(-1, 1));
	relative_manipulators.push_back(position(0, 0));
	//validate points
	current_orientation = EAST;
	vector<position> absolute_manip = absolute_manipulators();
	for (auto it = absolute_manip.begin(); it != absolute_manip.end(); ++it) {
		position pos_to_remove = *it + robot;
		if (!board_tile_is_wall(pos_to_remove)) {
			non_validated_tiles--;
			BOARD_TILE(pos_to_remove) = PAINTED;
		}
	}
	owned_fastwheels_boosters = 0;
	owned_drill_boosters = 0;
	owned_manipulators_boosters = 0;
	time_step = 0;
	distance_loss = 0;
	//init_graph();
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

	if (!BOARD_TILE_IS_WALL(robot + position(1,0)))
		ret.push_back("D");
	if (!BOARD_TILE_IS_WALL(robot + position(-1,0)))
		ret.push_back("A");
	if (!BOARD_TILE_IS_WALL(robot + position(0,1)))
		ret.push_back("W");
	if (!BOARD_TILE_IS_WALL(robot + position(0,-1)))
		ret.push_back("S");
	return ret;
}

const vector<position> additionnal_manipulators = {position(0, -1), position(2, 1), position(1,2), position(0, -2)};
string mine_state::strip(string commands) {
	string ret = "";

	for (auto it = commands.begin(); it != commands.end();++it) {
		if (non_validated_tiles == 0) {
			return ret;
		}
		switch (*it) {
			case 'W':
				if (!BOARD_TILE_IS_WALL(robot + position(0,1)))
					ret += "W";
				break;
			case 'D':
				if (!BOARD_TILE_IS_WALL(robot + position(1,0)))
					ret += "D";
				break;
			case 'S':
				if (!BOARD_TILE_IS_WALL(robot + position(0,-1)))
					ret += "S";
				break;
			case 'A':
				if (!BOARD_TILE_IS_WALL(robot + position(-1,0)))
					ret += "A";
				break;
			default:
				ret += *it;
				break;
		}
		apply_command(string(1,*it));
	}
	return ret;
}

void mine_state::apply_command(string command) {
	bool invalid_move = false;

	for (unsigned int i = 0; i < command.length(); i++){
		position new_pos(-1, -1);

		switch (command[i]) {
		case 'W':
			new_pos = robot + position(0,1);
			break;
		case 'S':
			new_pos = robot + position(0,-1);
			break;
		case 'A':
			new_pos = robot + position(-1,0);
			break;
		case 'D':
			new_pos = robot + position(1,0);
			break;
		case 'E':
			current_orientation = (enum orientation)((current_orientation + 1) % 4);
			/*for (unsigned int j = 0; j < relative_manipulators.size(); j++) {
				relative_manipulators[j] = position(relative_manipulators[j].imag(), -relative_manipulators[j].real());
			}*/
			time_step++;
			break;
		case 'Q':
			current_orientation = (enum orientation)(current_orientation == NORTH ? WEST: current_orientation - 1);
			//current_orientation = (enum orientation)((current_orientation - 1));
			/*for (unsigned int j = 0; j < relative_manipulators.size(); j++) {
				relative_manipulators[j] = position(-relative_manipulators[j].imag(), relative_manipulators[j].real());
			}*/
			time_step++;
			break;
		}
		//move
		invalid_move = false;
		if (new_pos != position(-1, -1)) {
			if (!BOARD_TILE_IS_WALL(new_pos)) {
				robot = new_pos;
				time_step++;
			} else {
				invalid_move = true;
			}
		}

		//validate tiles
		bool validated = false;
		auto absolute_manip = absolute_manipulators();
		for (auto it = absolute_manip.begin(); it != absolute_manip.end(); ++it) {
			position pos_to_remove = *it + robot;
			if (BOARD_TILE_IS_EMPTY(pos_to_remove)) {
				non_validated_tiles--;
				if (non_validated_tiles < 0) {
					cout << "issue" << endl;
				}
				BOARD_TILE(pos_to_remove) = PAINTED;
				validated = true;
			}
		}
		if (validated == false && !invalid_move) {
			distance_loss ++;
			//cout << "unusefull" << distance_loss << endl;
		}
		/*cout << "validated = " << validated << endl;
		cout << "distance_loss = " << distance_loss << endl;
		cout << "invalid = " << invalid_move << endl;
		cout << "time_step = " << time_step << endl;
		cout << endl;*/

		//collect manipulator booster
		auto pos_to_remove = find(manipulators_boosters.begin(), manipulators_boosters.end(), robot);
		if (pos_to_remove != manipulators_boosters.end()){
			manipulators_boosters.erase(pos_to_remove);
			owned_manipulators_boosters += 1;
			relative_manipulators.push_back(additionnal_manipulators[owned_manipulators_boosters - 1]);
		}
		//collect fast booster
		pos_to_remove = find(fastwheels_boosters.begin(), fastwheels_boosters.end(), robot);
		if (pos_to_remove != fastwheels_boosters.end()){
			fastwheels_boosters.erase(pos_to_remove);
			owned_fastwheels_boosters += 1;
		}
		//collect manipulator booster
		pos_to_remove = find(drill_boosters.begin(), drill_boosters.end(), robot);
		if (pos_to_remove != drill_boosters.end()){
			drill_boosters.erase(pos_to_remove);
			owned_drill_boosters += 1;
		}
	}
}
