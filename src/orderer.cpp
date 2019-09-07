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
#include "fileparser.h"

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
	int gInstance = 6;
	bool display = false;
    bool load_file = false;

	while ((c = getopt(argc, argv, "dp:a:hi:l")) != -1) switch (c) {
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
            case 'l':
                load_file = true;
                break;
			case 'h':
			default:
				print_help();
				//exit(0);
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
        vector<vector<position>> solution_pos = parse_split("./results/split-" + to_string(gInstance) + ".txt");
        vector<vector<Node>> previous_solution = nav.node_from_coords(solution_pos);

        vector<Node> centered_nodes;
        for (auto zone:previous_solution) {
            centered_nodes.push_back(zone[0]);
        }
        timer.tic();
        vector<vector<position>> solution;
        if (!load_file) {
            genetic_orderer orderer(nav, centered_nodes);
            cout << "instance "<< gInstance << ": orderer generation = " << timer.toc() << " s" << endl;
            timer.tic();
            vector<Node> ordered_zones = orderer.solve(population);
            vector<vector<Node>> ordered_solution;
            cout << "instance "<< gInstance << ": found solution = " << timer.toc() << " s" << endl;
            for (auto zoneordre:ordered_zones) {
                for (auto zone:previous_solution) {
                    if (zone[0] == zoneordre)
                        ordered_solution.push_back(zone);
                }
            }
            solution =
				nav.list_of_coords_from_nodes(ordered_solution);
        } else {
            solution = parse_split("./results/split-" + to_string(gInstance) + ".txt");
        }

        std::ofstream output_file;
        output_file.open("./results/order-"+to_string(gInstance) + ".txt",
                        std::ofstream::trunc);

        for (auto zone:solution) {
            for (auto point:zone)
                output_file << "("<< point.real() << "," << point.imag() << ")/";
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
