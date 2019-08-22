#ifndef MINE_H
#define MINE_H
#include "common.h"
#include "complex"

typedef complex<int> position;

class mine_state {
   public:
   	mine_state(mine_state *base_mine);
	mine_state(string filename);
	~mine_state();

	int time_step;
	bool is_point_valid(position point);
	void apply_command(string command);
	vector<string> get_next_valid_command();
	string strip(string commands);

	vector<position> non_validated_tiles;

	position robot;
	vector<position> relative_manipulators;
	vector<position> mine_map;
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