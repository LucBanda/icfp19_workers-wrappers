#include "mine.h"

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
	/*size_t openpos = t_line.find(open_delimiter);
	t_line.erase(0, openpos + open_delimiter.length());
	size_t closepos = t_line.find(close_delimiter);
	t_line.erase(closepos, line.npos);
*/
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
	mine_map = base_mine->mine_map;
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
}

mine_state::mine_state(string filename) {
	ifstream file(filename);
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
	for (auto it = mine_map.begin(); it != mine_map.end(); ++it) {
		if (max_size_x < it->real()) max_size_x = it->real();
		if (max_size_y < it->imag()) max_size_y = it->imag();
	}
	for (int i = 0; i < max_size_x; i++) {
		for (int j = 0; j < max_size_y; j++) {
			position cur(i,j);
			if (is_point_valid(cur)) {
				non_validated_tiles.push_back(cur);
			}
		}
	}
	relative_manipulators.push_back(position(1, 0));
	relative_manipulators.push_back(position(1, 1));
	relative_manipulators.push_back(position(1, -1));
	//validate points
	for (auto it = relative_manipulators.begin(); it != relative_manipulators.end(); ++it) {
		auto pos_to_remove = find(non_validated_tiles.begin(), non_validated_tiles.end(), *it + robot);
		if (pos_to_remove != non_validated_tiles.end()) non_validated_tiles.erase(pos_to_remove);
	}
	owned_fastwheels_boosters = 0;
	owned_drill_boosters = 0;
	owned_manipulators_boosters = 0;
	time_step = 0;
}

mine_state::~mine_state() {}



bool mine_state::is_point_valid(position point) {
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

void mine_state::apply_command(string command) {
	bool invalid_move = false;

	for (unsigned int i = 0; i < command.length(); i++){
		time_step++;
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
			for (unsigned int j = 0; j < relative_manipulators.size(); j++) {
				relative_manipulators[j] = position(relative_manipulators[j].imag(), -relative_manipulators[j].real());
			}
			break;
		case 'Q':
			for (unsigned int j = 0; j < relative_manipulators.size(); j++) {
				relative_manipulators[j] = position(-relative_manipulators[j].imag(), relative_manipulators[j].real());
			}
			break;
		}
		//move
		if (new_pos != position(-1, -1) && is_point_valid(new_pos)) {
			robot = new_pos;
		} else {
			invalid_move = true;
		}

		//validate tiles
		for (auto it = relative_manipulators.begin(); it != relative_manipulators.end(); ++it) {
			auto pos_to_remove = find(non_validated_tiles.begin(), non_validated_tiles.end(), *it + robot);
			if (pos_to_remove != non_validated_tiles.end()) non_validated_tiles.erase(pos_to_remove);
		}
		//collect manipulator booster
		auto pos_to_remove = find(manipulators_boosters.begin(), manipulators_boosters.end(), robot);
		if (pos_to_remove != manipulators_boosters.end()){
			manipulators_boosters.erase(pos_to_remove);
			owned_manipulators_boosters += 1;
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
