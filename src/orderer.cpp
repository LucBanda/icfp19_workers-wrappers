#include <lemon/adaptors.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "fileparser.h"
#include "functional"
#include "genetic_orderer.h"
#include "genetic_splitter.h"
#include "lemon/bfs.h"
#include "openga.hpp"
#include "renderer.h"
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
	int population = 2000;
	int gInstance = 4;

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
				// exit(0);
		}
	EA::Chronometer timer;
	timer.tic();
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
		vector<vector<position>> solution_pos =
			parse_split("./results/split-" + to_string(gInstance) + ".txt");
		vector<vector<Node>> previous_solution =
			nav.node_from_coords(solution_pos);

		timer.tic();
		vector<vector<position>> solution;
		genetic_orderer orderer(nav, previous_solution);
		cout << "instance " << gInstance
			 << ": orderer generation = " << timer.toc() << " s" << endl;
		timer.tic();
		vector<vector<Node>> ordered_solution = orderer.solve(population);
		solution = nav.list_of_coords_from_nodes(ordered_solution);

		std::ofstream output_file;
		output_file.open("./results/order-" + to_string(gInstance) + ".txt",
						 std::ofstream::trunc);

		for (auto zone : solution) {
			for (auto point : zone)
				output_file << "(" << point.real() << "," << point.imag()
							<< ")/";
			output_file << endl;
		}

		output_file.flush();
		output_file.close();

		if (!do_all) {
			return 0;
		}
	}

	return 0;
}
