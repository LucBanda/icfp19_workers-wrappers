#ifndef AGENT_H
#define AGENT_H
#include <map>
#include <tuple>
#include "common.h"
#include "mine.h"

class agent {
   public:
	mine_state *mine;
	string execution;

	agent(mine_state *arg_mine);
	virtual ~agent();

	int get_cost();
	void step();
	void set_execution_map(string map);
	int run();

   protected:
	string execution_map;
	int max_time_step;
};

#endif