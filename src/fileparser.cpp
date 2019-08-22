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