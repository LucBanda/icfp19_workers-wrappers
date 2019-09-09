#include <lemon/adaptors.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "fileparser.h"
#include "functional"
#include "genetic_optimizer.h"
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
		"	-p population: population of each generation (default 2000)\n"
		"	-s size: targeted size of each area\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int start_instance = 0;
	int c;
	int population =2000;
	int gInstance = 4;
	int region_size = 50;
	bool perform_partitionning = false;
	bool perform_ordering = false;
	bool perform_optimization = false;

	while ((c = getopt(argc, argv, "123s:p:a:hi:")) != -1) switch (c) {
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
			case 's':
				region_size = atoi(optarg);
				break;
			case '1':
				perform_partitionning = true;
				break;
			case '2':
				perform_partitionning = true;
				break;
			case '3':
				perform_optimization = true;
				break;
			case 'h':
			default:
				print_help();
				exit(0);
		}

	EA::Chronometer timer;

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
		cout << "___________ INSTANCE " << gInstance << " ______________" << endl;

		if (perform_partitionning){
			cout << "*********** SPLITTING ****************" << endl;

			// partition graph
			global_graph_splitter splitter(&nav.graph);
			splitter.target_nb_of_nodes_per_zone = region_size;
			vector<vector<Node>> final_sol = splitter.solve(population);
			ofstream output_file;
			output_file.open("./results/split-" + to_string(gInstance) + ".txt",
							std::ofstream::trunc);

			vector<vector<position>> solution =
				nav.list_of_coords_from_nodes(final_sol);

			for (auto zone : solution) {
				for (auto point : zone)
					output_file << "(" << point.real() << "," << point.imag()
								<< ")/";
				output_file << endl;
			}

			output_file.close();
		}
		if (perform_ordering) {
			vector<vector<position>> solution_pos =
				parse_split("./results/split-" + to_string(gInstance) + ".txt");
			vector<vector<Node>> previous_solution =
				nav.node_from_coords(solution_pos);

			vector<Node> centered_nodes;
			for (auto zone : previous_solution) {
				centered_nodes.push_back(zone[0]);
			}
			timer.tic();

			cout << "************ ORDERING ****************" << endl;
			genetic_orderer orderer(nav, centered_nodes);
			cout << "instance " << gInstance
				<< ": orderer generation = " << timer.toc() << " s" << endl;
			timer.tic();
			vector<Node> ordered_zones = orderer.solve(population);
			vector<vector<Node>> ordered_solution;
			cout << "instance " << gInstance << ": found solution = " << timer.toc()
				<< " s" << endl;
			for (auto zoneordre : ordered_zones) {
				for (auto zone : previous_solution) {
					if (zone[0] == zoneordre) ordered_solution.push_back(zone);
				}
			}
			vector<vector<position>> solution = nav.list_of_coords_from_nodes(ordered_solution);
			ofstream output_file;
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
		}

		if (perform_optimization) {
			cout << "************ SOLVING ****************" << endl;
			vector<vector<position>> zone_list =
				parse_split("./results/order-" + to_string(gInstance) + ".txt");
			string solution_str = "";
			Node start_of_zone = nav.initialNode;
			for (int i = 0; i < zone_list.size(); i++) {
				genetic_optimizer optimizer(gInstance, &nav, zone_list, i,
											start_of_zone, solution_str);
				pair<string, Node> pair_sol = optimizer.solve(population);
				solution_str = solution_str + pair_sol.first;
				start_of_zone = pair_sol.second;
				std::ofstream output_file;
				output_file.open("./results/" + to_string(gInstance) + ".txt",
								std::ofstream::trunc);
				output_file << solution_str << endl;
				output_file.close();
				cout << "instance " << gInstance << ", " << i+1 << "/" << zone_list.size() << ", total size = " << solution_str.length() << endl;
			}
		}

		if (!do_all) {
			return 0;
		}
	}

	return 0;
}