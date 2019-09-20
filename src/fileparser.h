#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include "common.h"
#include <iomanip>

enum Booster {
	NONE,
	FASTWHEEL,
	MANIPULATOR,
	DRILL,
	MYSTERE,
	CLONE,
	TELEPORT
};

using namespace std;

string parse_result(string fileName);
vector<vector<position>> parse_split(string filename);

class mine_parser {
    public:
    mine_parser(int arg_instance);
    ~mine_parser(){};

    int instance;
	vector<position> mine_map;
	vector<vector<position>> obstacles;
	position robotPos;
	std::map<Booster, vector<position>> boosters;
    int max_size_x;
	int max_size_y;
};
#endif