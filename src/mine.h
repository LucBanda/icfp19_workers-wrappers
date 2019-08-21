#ifndef MINE_H
#define MINE_H
#include "common.h"
#include "complex"

typedef complex<int> position;

class mine_state {
   public:
	mine_state(string filename);
	~mine_state();

	position robot;
	vector<position> relative_manipulators;
	vector<position> mine_map;
	vector<position> manipulators_boosters;
	vector<position> fastwheels_boosters;
	vector<position> drill_boosters;
	vector<position> mystere_boosters;
	vector<vector<position>> obstacles;

};

#endif