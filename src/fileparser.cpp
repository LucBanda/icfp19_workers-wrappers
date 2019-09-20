#include "fileparser.h"
#include "agent.h"

static std::string getLastLine(std::ifstream& in) {
	std::string line;
	while (in >> std::ws && std::getline(in, line))  // skip empty lines
		;

	return line;
}

string parse_result(string fileName) {
	std::ifstream file(fileName);
	if (!file.good()) return "";
	if (file) {
		std::string token;
		std::string line = getLastLine(file);
		if (line == "") {
			cout << "empty line at end of file" << endl;
			exit(-1);
		}
		cout << line << endl;
		// remove beginning
		std::string delimiter = "*";
		size_t pos = line.find(delimiter);
		return line.substr(pos + delimiter.length(), line.length());

	} else
		std::cout << "Unable to open file.\n";
	return "";
}

vector<vector<position>> parse_split(string filename) {
	vector<vector<position>> result;
	std::ifstream file(filename);
	if (!file.good()) return result;
	if (file) {
		string line;
		while (getline(file, line)) {
			std::istringstream iss(line);
			position pos;
			vector<position> lineresult;
			int x, y;
			char p1, v, p2, slash;
			while (iss >> p1 >> x >> v >> y >> p2 >> slash) {
				pos = position(x, y);
				lineresult.push_back(pos);
			}
			result.push_back(lineresult);
			// process pair (a,b)
		}
	}
	return result;
}



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


mine_parser::mine_parser(int arg_instance) {
	instance = arg_instance;
	ostringstream filename;

	filename << "./part-1-initial/prob-" << setw(3) << setfill('0') << instance
			 << ".desc";

	ifstream file(filename.str());
	if (!file.good()) {
		cout << "cannot find file " << filename.str() << endl;
		return;
	}

	std::string line;
	getline(file, line);

	// remove beginning
	// parse the file
	mine_map = parse_token_list(get_next_tuple_token(&line));
	robotPos = parse_token_list(get_next_tuple_token(&line))[0];
	obstacles = parse_list_of_token_lists(get_next_tuple_token(&line));

	string boosters_str = get_next_tuple_token(&line);
	while (boosters_str.length() > 0) {
		string booster_str = get_next_tuple_polygon(&boosters_str);
		char code = booster_str[0];
		booster_str.erase(0, 1);
		boosters[booster_code.at(code)].push_back(parse_position(booster_str));
	}

	// init variables and graphs
	max_size_x = 0;
	max_size_y = 0;
	for (const auto &it:mine_map) {
		if (max_size_x < it.real()) max_size_x = it.real();
		if (max_size_y < it.imag()) max_size_y = it.imag();
	}

}