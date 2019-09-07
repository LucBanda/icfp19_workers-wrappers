#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include <iomanip>
#include "agent.h"
#include "mine.h"
#include "common.h"

string parse_result(string fileName);
vector<vector<position>> parse_split(string filename);

#endif