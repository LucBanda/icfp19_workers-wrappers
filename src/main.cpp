#include <iomanip>
#include "agent.h"
#include "common.h"
#include "fileparser.h"
#include "openga.hpp"
#include "renderer.h"
#include "sys/time.h"

struct main_status {
	agent* ag;
	string execution_list;
};

bool idle(void* user_param) {
	struct main_status* status = (struct main_status*)user_param;
	string command = status->execution_list.substr(0, 1);
	if (command == "B") {
		int pos = status->execution_list.find(')');
		command = status->execution_list.substr(0, pos+1);
		status->execution_list.erase(0, pos+1);
	} else {
		status->execution_list.erase(0, 1);
	}
	status->ag->execute_seq(command);
	return false;
}

static void print_help() {
	printf(
		"options: \n \
	-h : this help \n \
	-i instance: instance of the problem to display \n \
	-l : load the best solution so far for this problem \n \
	-a : do all problem\n \
	-r : resume from time\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	struct main_status status;
	renderer* render;
	int c;
	bool load_result = false;
	int instance = 3;
	string exec;
	string fileName = "";
	int start_instance = 0;

	while ((c = getopt(argc, argv, "r:a:hli:p:")) != -1) switch (c) {
			case 'l':
				load_result = true;
				break;
			case 'i':
				instance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				start_instance = atoi(optarg);
				break;
			case 'p':
				break;
			case 'h':
			default:
				print_help();
				break;
		}

	for (int i = start_instance; i < 151; i++) {
		if (do_all) {
			instance = i;
		}

		cout << "********* Instance " << instance << "*************" << endl;
		render = new renderer(instance);
		ostringstream padded_filename;
		if (instance == -1)
			padded_filename << "./part-1-examples/example-01.desc";
		else
			padded_filename << "./part-1-initial/prob-" << setw(3)
							<< setfill('0') << instance << ".desc";
		mine_state* mine = new mine_state(padded_filename.str());
		mine_navigator navigator(mine);
		agent ag(&navigator, navigator.robot_pos);
		status.ag = &ag;
		render->set_agent(&ag);

		if (load_result) {
			string execution = parse_result("./results/" + to_string(instance) + ".txt");
			status.execution_list = execution;
		}
		render->idle = &idle;
		render->idle_param = &status;
		render->mainLoop();

		cout << "time_step " << status.ag->time_step << endl;
		//cout << "distance loss " << status.mine->distance_loss << endl;
		//cout << "useful " << status.mine->time_step - status.mine->distance_loss
		//	 << endl;
		//cout << status.mine->distance_loss / status.mine->time_step * 100.
		//	 << " %" << endl;

		delete render;
		delete mine;

		if (!do_all) return 0;
	}
	return 0;
}
