#include <iomanip>
#include "agent.h"
#include "common.h"
#include "openga.hpp"
#include "renderer.h"
#include "sys/time.h"
#include "fileparser.h"

struct main_status {
	mine_state *mine;
	agent *ag;
};

bool idle(void* user_param) {
	struct main_status* status = (struct main_status*)user_param;
	mine_state *mine = status->mine;
	string command = status->ag->execution.substr(0,1);
	status->ag->execution.erase(0, 1);
	mine->apply_command(command);
	return false;
}

static void print_help() {
	printf(
"options: \n \
	-h : this help \n \
	-i instance: instance of the problem to display \n \
	-l : load the best solution so far for this problem \n \
	-a : do all problem\n \
	-r : resume from time\n \
	-f : load the specified file");
}

int main(int argc, char** argv) {
	bool do_all = false;
	struct main_status status;
	renderer* render;
	int c;
	bool load_result = false;
	int instance = -1;
	string exec;
	string fileName = "";

	while ((c = getopt(argc, argv, "f:r:ahli:")) != -1) switch (c) {
			case 'l':
				load_result = true;
				break;
			case 'i':
				instance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				break;
			case 'f':
				fileName = optarg;
				break;
			case 'h':
			default:
				print_help();
				exit(0);
				break;
		}

	for (int i = 1; i < 151; i++) {
		if (do_all) {
			instance = i;
		}

		//agent* ag;
		render = new renderer();
		ostringstream padded_filename;
		if (instance == -1) padded_filename << "./part-1-examples/example-01.desc";
		 else padded_filename << "./part-1-initial/prob-"<< setw(3) << setfill('0') << instance << ".desc";
		mine_state *mine = new mine_state(padded_filename.str());

		//ag = new agent(instance);

		render->set_mine(mine);
		status.mine = mine;
		status.ag =  new agent(mine);

		if (load_result) {
			if (fileName == "") fileName = "./results/" + to_string(instance) + ".txt";
			string execution = parse_result(fileName);
			status.ag->set_execution_map(execution);
		}

		render->idle = &idle;
		render->idle_param = &status;
		render->mainLoop(NULL);

		delete render;
		delete status.ag;
		delete mine;

		if (!do_all) return 0;
	}
	return 0;
}
