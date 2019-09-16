#ifndef MINE_H
#define MINE_H
#include <lemon/list_graph.h>
#include "common.h"
#include "complex"


enum map_tile {
	EMPTY,
	PAINTED,
	WALL,
};

enum orientation {
	NORTH,
	EAST,
	SOUTH,
	WEST,
};

class mine_state;

enum Booster {
	NONE,
	FASTWHEEL,
	MANIPULATOR,
	DRILL,
	MYSTERE,
	CLONE,
	TELEPORT
};

class mine_navigator {
   public:

	int instance;
	Graph graph;
	Graph::NodeMap<position> coord_map;
	Graph::ArcMap<char> direction_map;
	Graph::NodeMap<Booster> boosters_map;
	std::map<Booster, vector<Node>> boosters_lists;
	Node initialNode;

	std::map<int, Node> coord_to_node_map;
	int max_size_x;
	int max_size_y;

	mine_navigator(int instance);
	~mine_navigator(){};

	vector<vector<position>> list_of_coords_from_nodes(const vector<vector<Node>> &list);
	vector<vector<Node>> node_from_coords(vector<vector<position>> &pos_list);
	Node node_from_coord(position pos);
	bool is_coord_in_map(position coord);

	string get_orientation(orientation source, orientation target);
	vector<Booster> boosters_in_node_list(vector<Node> &zone);
};

static inline string get_next_tuple_token(string *line) {
	string token;
	string delimiter = "#";
	size_t pos = line->find(delimiter);
	token = line->substr(0, pos);
	line->erase(0, pos + delimiter.length());
	return token;
}

static inline string get_next_tuple_polygon(string *line) {
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

static inline position parse_position(string positionstring) {
	string arg1, arg2;
	string delimiter = ",";
	size_t delimpos = positionstring.find(delimiter);
	arg1 = positionstring.substr(1, delimpos - 1);
	arg2 = positionstring.substr(delimpos + delimiter.length(),
								 positionstring.length() - delimpos - 2);
	return position(atoi(arg1.c_str()), atoi(arg2.c_str()));
}

static inline vector<position> parse_token_list(string line) {
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

static inline vector<vector<position>> parse_list_of_token_lists(string line) {
	vector<vector<position>> ret;
	string t_line = line;
	while (t_line.length() > 0) {
		ret.push_back(parse_token_list(get_next_tuple_polygon(&t_line)));
	}
	return ret;
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
		for (auto it = obstacles.begin(); it != obstacles.end(); ++it) {
			if (PointInPolygon(point, *it)) {
				is_valid = false;
			}
		}
	}
	return is_valid;
}

#endif