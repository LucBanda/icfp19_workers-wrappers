#ifndef MINE_H
#define MINE_H
#include "common.h"
#include "complex"
#include <lemon/list_graph.h>

typedef complex<int> position;

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

class mine_navigator {
   public:
	mine_navigator(mine_state *mine_base);
	~mine_navigator() {};
	void init_graph();
	void init_ordered_map();
	string goto_node(enum orientation source_orientation, enum orientation &last_orientation, ListDigraph::Node orig, ListDigraph::Node target, ListDigraph::Node *ending_node);
	//string orientcorrectly(enum orientation source_orientation, enum orientation target_orientation);
	vector<ListDigraph::Node> get_node_list();
	vector<ListDigraph::Node> get_bfs_from_node(ListDigraph::Node start, int depth);

	ListDigraph graph;
	ListDigraph::NodeMap<position> coord_map;
	ListDigraph::ArcMap<char> direction_map;
	ListDigraph::ArcMap<enum orientation> orientation_map;
	ListDigraph::ArcMap<int> length;
	ListDigraph::Node initialNode;
	ListDigraph::NodeMap<vector<ListDigraph::Node>> ordered_node_map;
};

class mine_state {
   public:
   	mine_state(mine_state *base_mine);
	mine_state(string filename);
	~mine_state();

	double distance_loss;
	int time_step;
	bool is_point_valid(position point, vector<position> *mine_map);
	void apply_command(string command);
	vector<string> get_next_valid_command();
	string strip(string commands);
	bool board_tile_is_painted(position tile);
	bool board_tile_is_wall(position tile);

	int non_validated_tiles;
	enum map_tile **board;

	position robot;
	enum orientation current_orientation;
	vector<position> relative_manipulators;
	vector<position> manipulators_boosters;
	vector<position> fastwheels_boosters;
	vector<position> drill_boosters;
	vector<position> mystere_boosters;
	int owned_manipulators_boosters;
	int owned_fastwheels_boosters;
	int owned_drill_boosters;

	vector<vector<position>> obstacles;
	int max_size_x, max_size_y;

};

#endif