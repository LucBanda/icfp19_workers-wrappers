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
	FASTWHEEL,
	MANIPULATOR,
	DRILL,
	MYSTERE,
};

class mine_navigator {
   public:
	mine_navigator(mine_state *mine_base);
	~mine_navigator(){};
	void init_graph();
	string goto_node(Node orig, Node target);
	mine_state *mine;

	vector<Node> get_node_list();
	vector<vector<position>> list_of_coords_from_nodes(
		const vector<vector<Node>> list);
	vector<vector<Node>> node_from_coords(vector<vector<position>> pos_list);
	Node node_from_coord(position pos);

	string get_orientation(orientation source, orientation target);
	vector<Booster> boosters_in_node_list(vector<Node> zone);

	Graph graph;
	Graph::NodeMap<position> coord_map;
	Graph::ArcMap<char> direction_map;
	Graph::ArcMap<enum orientation> orientation_map;
	Graph::ArcMap<int> length;
	Node initialNode;
	Graph::NodeMap<vector<Node>> ordered_node_map;
	std::map<int, Node> coord_to_node_map;
	int max_size_x;
	Node robot_pos;
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
	bool board_tile_has_booster(position tile);
	vector<position> absolute_valid_manipulators();
	vector<Booster> boosters_in_position_list(vector<position> list);
	void set_current_nb_of_manipulators(int nb);

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