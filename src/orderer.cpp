#include "genetic_orderer.h"
#include <lemon/adaptors.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "functional"
#include "lemon/bfs.h"
#include "openga.hpp"
#include "renderer.h"
#include "sys/time.h"
#include "genetic_splitter.h"

static void print_help() {
	printf(
		"options: \n"
		"	-h : this help \n"
		"	-i instance: instance of the problem to display \n"
		"	-l : load the best solution so far for this problem \n"
		"	-a from: do all problem from \"from\"\n"
		"	-n number: number of thrust to optimize (default 1)\n"
		"	-f factor: divider of fuel max to limit thrust range (default is "
		"2)\n"
		"	-p population: population of each generation (default 2000)\n"
		"	-d : enable logging of chromosomes in a file\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int start_instance = 0;
	int c;
	int population = 2000;
	int gInstance = 0;
	bool display = false;

	while ((c = getopt(argc, argv, "dp:a:hi:")) != -1) switch (c) {
			case 'i':
				gInstance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				start_instance = atoi(optarg);
				break;
			case 'd':
				display = true;
				break;
			case 'p':
				population = atoi(optarg);
				break;
			case 'h':
			default:
				print_help();
				exit(0);
		}

	for (int i = start_instance; i < 301; i++) {
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
		cout << endl << "Instance " << gInstance << ", ";
		global_graph_splitter splitter(&nav.graph);
		splitter.target_nb_of_nodes_per_zone = 50;
		vector<vector<Node>> final_sol = splitter.solve(population);

        //genetic_orderer(&nav.graph, );
		if (display) {
			vector<vector<position>> solution =
				nav.list_of_coords_from_nodes(final_sol);
			renderer subrender;
			subrender.set_mine(&mine);
			subrender.set_zones(&solution);
			subrender.mainLoop();
		}

		if (!do_all) {
			return 0;
		}
	}

	return 0;
}
