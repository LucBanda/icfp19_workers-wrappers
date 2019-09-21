#ifndef MINE_H
#define MINE_H
#include <lemon/list_graph.h>
#include "common.h"
#include "complex"
#include "fileparser.h"

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

static const map<char, Booster> booster_code = {
	{'X', MYSTERE},   {'B', MANIPULATOR}, {'L', DRILL},
	{'F', FASTWHEEL}, {'C', CLONE},		  {'R', TELEPORT}};

static const map<char, position> direction_code = {
	{'W', position(0,1)},
	{'A', position(-1, 0)},
	{'D', position(1, 0)},
	{'S', position(0, -1)}
};

class mineGraph {
   public:
    mineGraph(int instance);
	~mineGraph(){};
	string name;
	void create_masked_nodes(mine_parser &parser);
	void create_full_nodes(mine_parser &parser);
	void create_single_arcs(mine_parser &parser, bool fullnodes);
	void create_double_arcs(mine_parser &parser, bool fullnodes);

	int instance;
	int max_size_x;
	int max_size_y;
	Graph graph;
	Graph::NodeMap<position> coord_map;
	Graph::NodeMap<map<char, Arc>> arcDirectionMap;
	Graph::ArcMap<char> direction_map;
	Graph::NodeMap<Booster> boosters_map;
	map<Node, Node> to_full_graph_nodes;
	map<Node, Node> from_full_graph_nodes;

	std::map<Booster, vector<Node>> boosters_lists;
	Node initialNode;
	std::map<int, Node> coord_to_node_map;
	vector<vector<position>> list_of_coords_from_nodes(const vector<vector<Node>> &list);
	vector<vector<Node>> node_from_coords(vector<vector<position>> &pos_list);
	Node node_from_coord(position pos);
	bool is_coord_in_map(position coord);

	vector<Booster> boosters_in_node_list(const vector<Node> &zone);
};

class masked_navigator:public mineGraph {
   public:
	masked_navigator(mine_parser &parser);
	~masked_navigator(){};
};

class full_navigator:public mineGraph {
	public:
	full_navigator(mine_parser &parser);
	~full_navigator() {};
};

class fast_navigator:public mineGraph {
	public:
	fast_navigator(mine_parser &parser);
	~fast_navigator() {};
};

class fast_full_navigator:public mineGraph {
	public:
	fast_full_navigator(mine_parser &parser);
	~fast_full_navigator() {};
};

enum BoostMode {
	STANDARD_MODE,
	DRILL_MODE,
	FAST_MODE,
	FAST_DRILL_MODE
};

class navigator_factory {
	public:
	navigator_factory(int instance);
	~navigator_factory();
	int instance;

	mine_parser parser;
	masked_navigator masked_nav;
	full_navigator full_nav;
	fast_navigator fast_nav;
	fast_full_navigator fast_full_nav;
};

class navigator_selector {
	public:
	navigator_selector(navigator_factory &navigators);
	~navigator_selector() {};

	mineGraph *moving_nav;
	mineGraph *navigating_nav;
	mineGraph *base_nav;

	enum BoostMode current_mode;
	navigator_factory &navigators;

	void activate_boost(enum Booster booster);
	void step();
	mineGraph *graph();

	int fast_mode_step_left = 0;
	int drill_mode_step_left = 0;


};


#endif