#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "fileparser.h"
#include "functional"
#include "genetic_optimizer.h"
#include "lemon/dijkstra.h"
#include "openga.hpp"
#include "sys/time.h"

static void print_help() {
	printf(
		"options: \n"
		"	-h : this help \n"
		"	-i instance: instance of the problem to display \n"
		"	-a from: do all problem from \"from\"\n"
		"	-p population: population of each generation (default 2000)\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int start_instance = 0;
	int c;
	int population = 1000;
	int gInstance = 6;

	while ((c = getopt(argc, argv, "p:a:hi:")) != -1) switch (c) {
			case 'i':
				gInstance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				start_instance = atoi(optarg);
				break;
			case 'p':
				population = atoi(optarg);
				break;
			case 'h':
			default:
				print_help();
                break;
		}

	for (int i = start_instance; i < 150; i++) {
		if (do_all) {
			gInstance = i;
		}
		ostringstream padded_filename;
		if (gInstance == -1)
			padded_filename << "./part-1-examples/example-01.desc";
		else
			padded_filename << "./part-1-initial/prob-" << setw(3)
							<< setfill('0') << gInstance << ".desc";
		mine_state mine(padded_filename.str());
		mine_navigator nav(&mine);

		vector<vector<position>> zone_list =
			parse_split("./results/order-" + to_string(gInstance) + ".txt");
		string solution;
		Node start_of_zone = nav.initialNode;
		for (int i = 0; i < zone_list.size(); i++) {
			genetic_optimizer optimizer(gInstance, &nav, nav.mine, zone_list, i,
										start_of_zone, solution);
			pair<string, Node> pair_sol = optimizer.solve(population);
			solution = solution + pair_sol.first;
			start_of_zone = pair_sol.second;
			std::ofstream output_file;
			output_file.open("./results/" + to_string(gInstance) + ".txt",
							 std::ofstream::trunc);
			output_file << solution << endl;
			output_file.close();
		}

		if (!do_all) {
			return 0;
		}
	}
	return 0;
}