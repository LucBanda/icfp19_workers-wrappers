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
#include "filesystem"

static const vector<tuple<int, int, int, int, int>> testbench_table = {
	{2, 50, 3000, 3000, 3000},
	{3, 30, 3000, 3000, 3000},
	{4, 30, 3000, 3000, 3000},
	{10, 30, 3000, 3000, 3000},
	{35, 30, 3000, 3000, 3000},
	{57, 100, 3000, 3000, 3000},
	{150, 150, 3000, 3000, 3000},
	{201, 150, 3000, 3000, 3000},
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
	int population1 = 300;
	int population2 = 3000;
	int population3 = 500;
	int gInstance = 46;
	int region_size = 50;
	bool perform_partitionning = false;
	bool perform_ordering = false;
	bool perform_optimization = true;
	bool verbose = false;
	int popu;
	bool testbench = false;
	bool auto_region_size = true;
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
			auto_region_size = false;
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
		auto_region_size = true;
		for (int i = start_instance; i < 301; i++) {
			population1 = 300;
			population2 = 3000;
			population3 = 1000;
			problems.push_back(make_tuple(i, region_size, population1, population2, population3));
		}
	} else if (testbench) {
		problems.clear();
		perform_partitionning = true;
		perform_ordering = true;
		perform_optimization = true;
		auto_region_size = true;
		for (const auto &it:testbench_table) {
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

	for (const auto &problem:problems) {
		tie(gInstance, region_size, population1, population2, population3) = problem;
		if (auto_region_size) {
			if (gInstance == 2) region_size = 50;
			else if (gInstance < 21) region_size = 30;
			else if (gInstance < 51) region_size = 50;
			else if (gInstance < 101) region_size = 70;
			else if (gInstance < 151) region_size = 100;
			else if (gInstance < 181) region_size = 70;
			else if (gInstance < 201) region_size = 150;
			else region_size = 300;
		}
		cout << "instance = " << gInstance << ", region_size = " << region_size << ", populations = " << population1 << " " << population2 << " " << population3 << endl;
		EA::Chronometer main_timer;
		main_timer.tic();

		navigator_factory navigators(gInstance);
		masked_navigator &nav = navigators.masked_nav;
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
			for (const auto &zone : final_sol) cout << zone.size() << " ";
			cout << endl;

			for (const auto &zone : solution) {
				for (const auto &point : zone)
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
			genetic_orderer orderer(navigators, previous_solution);
			orderer.verbose = verbose;
			vector<vector<Node>> ordered_solution = orderer.solve(population2);

			cout << "instance " << gInstance
				<< ": order found = " << timer.toc() << " s" << endl;
			vector<vector<position>> solution = nav.list_of_coords_from_nodes(ordered_solution);
			ofstream output_file;
			output_file.open("./results/order-" + to_string(gInstance) + ".txt",
							std::ofstream::trunc);

			cout << "sizes = ";
			for (const auto &zone : solution) cout << zone.size() << " ";
			cout << endl;
			for (const auto &zone : solution) {
				for (const auto &point : zone)
					output_file << "(" << point.real() << "," << point.imag()
								<< ")/";
				output_file << endl;
			}

			output_file.flush();
			output_file.close();
		}
		int score;
		if (perform_optimization) {
			cout << "************ SOLVING ****************" << endl;
			vector<vector<position>> zone_list =
				parse_split("./results/order-" + to_string(gInstance) + ".txt");
			string solution_str = "";
			agent ag(navigators, navigators.full_nav.initialNode);
			for (int i = 0; i < zone_list.size(); i++) {
				genetic_optimizer optimizer(gInstance, ag, zone_list, i,
											solution_str);
				optimizer.verbose = verbose;
				string sol = optimizer.solve(population3);
				solution_str +=  sol;
				ag.execute_seq(sol);
				std::ofstream output_file;
				output_file.open("./results/" + to_string(gInstance) + ".txt",
								std::ofstream::trunc);
				output_file << solution_str << endl;
				output_file.flush();
				output_file.close();
				cout << "instance " << gInstance << ", " << i+1 << "/" << zone_list.size() << ", total score = " << optimizer.score << endl;
				score = optimizer.score;
			}
			std::ofstream output_file;
			ostringstream score_filename;
			score_filename << "./scores/" << gInstance << "_s" << region_size <<
							  "_p" << population1 << "_p" << population2 << "_p" << population3 << "_"<< score;
			output_file.open(score_filename.str(),
							std::ofstream::trunc);
			output_file << solution_str << endl;
			output_file.flush();
			output_file.close();
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