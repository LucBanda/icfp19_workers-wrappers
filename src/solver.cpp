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

static const vector<tuple<int, int, int, int, int>> testbench_table = {
	{2, 50, 3000, 5000, 3000},
	{3, 30, 3000, 5000, 3000},
	{4, 30, 3000, 5000, 3000},
	{10, 30, 3000, 5000, 3000},
	{35, 30, 3000, 5000, 3000},
	{57, 100, 3000, 5000, 3000},
	{150, 150, 3000, 5000, 3000},
	{201, 150, 3000, 5000, 3000},
};

static void print_help() {
	printf(
		"options: \n"
		"	-h : this help \n"
		"	-i instance: instance of the problem to display \n"
		"	-a from: do all problem from \"from\"\n"
		"	-p population: population of each generation (default 2000)\n"
		"	-s size: targeted size of each area\n"
		"	-v : enable verbose mode");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int start_instance = 0;
	int c;
	int population1 = 3000;
	int population2 = 5000;
	int population3 = 3000;
	int gInstance = 3;
	int region_size = 50;
	bool perform_partitionning = false;
	bool perform_ordering = false;
	bool perform_optimization = true;
	bool verbose = true;
	int popu;
	bool testbench = false;
	string testbench_identifier = "";

	while ((c = getopt(argc, argv, "t:123s:p:a:hi:v")) != -1) switch (c) {
		case 't':
			testbench = true;
			testbench_identifier = optarg;
			break;
		case 'i':
			gInstance = atoi(optarg);
			break;
		case 'a':
			do_all = true;
			start_instance = atoi(optarg);
			break;
		case 'p':
			popu = atoi(optarg);
			population1 = popu;
			population2 = popu;
			population3 = popu;
			break;
		case 's':
			region_size = atoi(optarg);
			break;
		case '1':
			perform_partitionning = true;
			break;
		case '2':
			perform_ordering = true;
			break;
		case '3':
			perform_optimization = true;
			break;
		case 'v':
			verbose = true;
			break;
		case 'h':
		default:
			print_help();
			exit(0);
	}
	//parameters are : <0> = instance; <1> size_of_region; <2> population1; <3> population2; <4> population3
	vector<tuple<int, int, int, int, int>> problems;
	if (do_all) {
		for (int i = start_instance; i < 301; i++) {
			population1 = 3000;
			population2 = 5000;
			population3 = 3000;
			problems.push_back(make_tuple(i, region_size, population1, population2, population3));
		}
	} else if (testbench) {
		problems.clear();
		perform_partitionning = true;
		perform_ordering = true;
		perform_optimization = true;
		for (auto it:testbench_table) {
			problems.push_back(it);
		}
		std::ofstream output_file;
		output_file.open("./testbench/" + testbench_identifier + ".txt",
						std::ofstream::app);
		output_file << "----------------------------------------------------------" << endl;
	} else {
		problems.emplace_back(gInstance, region_size, population1, population2, population3);
	}
	EA::Chronometer timer;

	for (auto problem:problems) {
		tie(gInstance, region_size, population1, population2, population3) = problem;
		cout << "instance = " << gInstance << ", region_size = " << region_size << ", populations = " << population1 << " " << population2 << " " << population3 << endl;
		EA::Chronometer main_timer;
		main_timer.tic();
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
			vector<vector<Node>> final_sol = splitter.solve(population1);
			ofstream output_file;
			output_file.open("./results/split-" + to_string(gInstance) + ".txt",
							std::ofstream::trunc);

			vector<vector<position>> solution =
				nav.list_of_coords_from_nodes(final_sol);

			cout << "sizes = ";
			for (auto zone : final_sol) cout << zone.size() << " ";
			cout << endl;

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

			cout << "************ ORDERING ****************" << endl;
			timer.tic();
			genetic_orderer orderer(nav, previous_solution);
			orderer.verbose = verbose;
			vector<vector<Node>> ordered_solution = orderer.solve(population2);

			cout << "instance " << gInstance
				<< ": order found = " << timer.toc() << " s" << endl;
			vector<vector<position>> solution = nav.list_of_coords_from_nodes(ordered_solution);
			ofstream output_file;
			output_file.open("./results/order-" + to_string(gInstance) + ".txt",
							std::ofstream::trunc);

			cout << "sizes = ";
			for (auto zone : solution) cout << zone.size() << " ";
			cout << endl;
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
				genetic_optimizer optimizer(gInstance, &nav, nav.mine, zone_list, i,
											start_of_zone, solution_str);
				pair<string, Node> pair_sol = optimizer.solve(population3);
				solution_str = solution_str + pair_sol.first;
				start_of_zone = pair_sol.second;
				std::ofstream output_file;
				output_file.open("./results/" + to_string(gInstance) + ".txt",
								std::ofstream::trunc);
				output_file << solution_str << endl;
				output_file.flush();
				output_file.close();
				cout << "instance " << gInstance << ", " << i+1 << "/" << zone_list.size() << ", total score = " << optimizer.score << endl;
			}
			std::ofstream output_file;
			output_file.open("./testbench/" + testbench_identifier + ".txt",
							std::ofstream::app);
			output_file << "instance " << gInstance << ", time = " << main_timer.toc() << ", total size = " << solution_str.length() << "  " << solution_str << endl;
			//output_file << solution_str << endl;
			output_file.close();
		}
		cout << endl;
	}

	return 0;
}