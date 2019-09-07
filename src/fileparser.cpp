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
			char p1,v,p2,slash;
			while (iss >> p1 >> x >> v >> y >>p2 >> slash) {
				pos = position(x,y);
				lineresult.push_back(pos);
			}
			result.push_back(lineresult);
			// process pair (a,b)
		}
	}
	return result;
}