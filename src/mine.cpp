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
}

mine_state::~mine_state() {}


bool PointInPolygon(position point, vector<position> polygon) {
  int i, j, nvert = polygon.size();
  bool c = false;

  for(i = 0, j = nvert - 1; i < nvert; j = i++) {
    if( ( (polygon[i].real() >= point.imag() ) != (polygon[j].imag() >= point.imag()) ) &&
        (point.real() <= (polygon[j].real() - polygon[i].real()) * (point.imag() - polygon[i].imag()) / (polygon[j].imag() - polygon[i].imag()) + polygon[i].real())
      )
      c = !c;
  }

  return c;
}