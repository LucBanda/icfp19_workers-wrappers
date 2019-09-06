#include "solver.h"
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

using std::cout;
using std::endl;
using std::string;

void genetic_graph_splitter::init_genes(
	MySolution& p, const std::function<double(void)>& rnd01) {
	int local_nb_of_zones = nb_of_zones;
	nb_of_node_per_zone = nb_of_nodes / local_nb_of_zones;
	while (local_nb_of_zones--) {
		p.split.emplace_back(
			node_list[rnd01() * node_list.size()],
			nb_of_node_per_zone + nb_of_node_per_zone / 4. * rnd01());
	}
}

vector<vector<Node>> genetic_graph_splitter::partition_graph_with_split(
	const vector<pair<Node, int>>& loc_split, double* score) {
	ListDigraph::NodeMap<bool> filter(*graph, true);
	vector<vector<Node>> result_total;

	for (auto patch : loc_split) {
		auto depth = patch.second;
		auto node = patch.first;
		if (filter[node] == true) {
			vector<Node> result;
			result.clear();
			FilterNodes<ListDigraph> subgraph(*graph, filter);
			Bfs<FilterNodes<ListDigraph>> bfs(subgraph);
			bfs.init();
			bfs.addSource(node);
			int cnt = 0;
			// bfs.addSource(list_node[it]);
			while (!bfs.emptyQueue() && depth > 0) {
				FilterNodes<ListDigraph>::Node n = bfs.processNextNode();
				if (filter[n] == true) {
					filter[n] = false;
					result.push_back(n);
					depth--;
					cnt++;
				}
			}
			if (cnt > 0) {
				vector<Node> result2;
				for (auto res : result) {
					result2.push_back(res);
				}
				result_total.push_back(result2);
			}

			if (score) *score += abs(nb_of_node_per_zone - cnt);
		}
	}
	if (score) {
		for (ListDigraph::NodeIt n(*graph); n != INVALID; ++n) {
			if (filter[n]) *score += nb_of_nodes;
		}
	}
	return result_total;
}

bool genetic_graph_splitter::eval_solution(const MySolution& p,
										   MyMiddleCost& c) {
	c.objective1 = 0;
	partition_graph_with_split(p.split, &c.objective1);
	return true;
}

MySolution genetic_graph_splitter::mutate(
	const MySolution& X_base, const std::function<double(void)>& rnd01,
	double shrink_scale) {
	MySolution X_new = X_base;
	int index = rnd01() * X_base.split.size();
	pair<Node, int> patch = X_base.split[index];
	Node node = patch.first;
	int nb = patch.second;
	int index_of_node = rnd01() * nb_of_nodes;
	node = graph->nodeFromId(index_of_node);
	nb += (-10. + 20. * rnd01());
	X_new.split[index] = make_pair(node, nb);

	int indexswap1 = X_base.split.size() * rnd01();
	int indexswap2 = X_base.split.size() * rnd01();
	iter_swap(X_new.split.begin() + indexswap1,
			  X_new.split.begin() + indexswap2);

	return X_new;
}

MySolution genetic_graph_splitter::crossover(
	const MySolution& X1, const MySolution& X2,
	const std::function<double(void)>& rnd01) {
	MySolution X_new;
	int index_to_split = X1.split.size() * rnd01();
	for (int i = 0; i < X1.split.size(); i++) {
		if (i < index_to_split)
			X_new.split.push_back(X1.split[i]);
		else
			X_new.split.push_back(X2.split[i]);
	}

	return X_new;
}

double genetic_graph_splitter::calculate_SO_total_fitness(
	const GA_Type::thisChromosomeType& X) {
	// finalize the cost
	double final_cost = 0.0;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

std::ofstream output_file;

void genetic_graph_splitter::SO_report_generation(
	int generation_number,
	const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
	const MySolution& best_genes) {
	cout << "instance = " << instance << ", "
		 << "Generation [" << generation_number << "], " << setprecision(10)
		 << "Best=" << -last_generation.best_total_cost << ", "
		 << "Average=" << -last_generation.average_cost << ", "
		 << "genes size=" << best_genes.to_string(this) << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;
}

genetic_graph_splitter::genetic_graph_splitter(ListDigraph* arg_graph) {
	graph = arg_graph;
	nb_of_nodes = 0;
	for (ListDigraph::NodeIt n(*graph); n != INVALID; ++n) {
		nb_of_nodes++;
		node_list.push_back(n);
	}
}
genetic_graph_splitter::~genetic_graph_splitter() {}

vector<vector<Node>>& genetic_graph_splitter::fix_solution(
	vector<vector<Node>>& solution) {
	ListDigraph::NodeMap<bool> filter(*graph, true);
	ListDigraph::NodeMap<int> zoneMap(*graph);

	for (int i = 0; i < solution.size(); i++) {
		for (auto n : solution[i]) {
			filter[n] = false;
			zoneMap[n] = i;
		}
	}
	FilterNodes<ListDigraph> subgraph(*graph, filter);
	for (FilterNodes<ListDigraph>::NodeIt n(subgraph); n != INVALID; ++n) {
		Bfs<ListDigraph> bfs(*graph);

		bfs.init();
		bfs.addSource(n);
		while (!bfs.emptyQueue()) {
			Node t = bfs.processNextNode();
			if (filter[t] == false) {
				solution[zoneMap[t]].push_back(n);
				break;
			}
		}
	}
	return solution;
}

vector<vector<Node>> genetic_graph_splitter::solve(int population_size) {
	using namespace std::placeholders;

	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = population_size;
	ga_obj.generation_max = 3000;
	ga_obj.calculate_SO_total_fitness = std::bind(
		&genetic_graph_splitter::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes =
		std::bind(&genetic_graph_splitter::init_genes, this, _1, _2);
	ga_obj.eval_solution =
		std::bind(&genetic_graph_splitter::eval_solution, this, _1, _2);
	ga_obj.mutate =
		std::bind(&genetic_graph_splitter::mutate, this, _1, _2, _3);
	ga_obj.crossover =
		std::bind(&genetic_graph_splitter::crossover, this, _1, _2, _3);
	ga_obj.SO_report_generation = std::bind(
		&genetic_graph_splitter::SO_report_generation_empty, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.6;
	ga_obj.best_stall_max = 30;
	ga_obj.average_stall_max = 10;
	ga_obj.elite_count = 10;

	ga_obj.solve();

	vector<pair<Node, int>> solution =
		ga_obj.last_generation
			.chromosomes[ga_obj.last_generation.best_chromosome_index]
			.genes.split;
	vector<vector<Node>> soluce = partition_graph_with_split(solution, NULL);
	return fix_solution(soluce);
}

global_graph_splitter::global_graph_splitter(ListDigraph* src_graph) {
	graph = src_graph;
}

vector<vector<Node>> global_graph_splitter::solve(int population) {
	EA::Chronometer timer;
	vector<vector<ListDigraph::Node>> sol_nodes;
	vector<Node> temp_res;
	for (ListDigraph::NodeIt it(*graph); it != INVALID; ++it) {
		temp_res.push_back(it);
	}
	sol_nodes.push_back(temp_res);
	timer.tic();
	vector<vector<ListDigraph::Node>> final_sol = sol_nodes;
	bool cont = true;
	int divisor = 5;
	while (cont) {
		sol_nodes = final_sol;
		final_sol.clear();
		bool something_done = false;
		for (auto patch : sol_nodes) {
			if (patch.size() < target_nb_of_nodes_per_zone) {
				final_sol.push_back(patch);
				continue;
			} else if (patch.size() < 2 * target_nb_of_nodes_per_zone) {
				divisor = 2;
			} else if (patch.size() < 3 * target_nb_of_nodes_per_zone) {
				divisor = 3;
			} else if (patch.size() < 4 * target_nb_of_nodes_per_zone) {
				divisor = 4;
			} else {
				divisor = 5;
			}
			Graph::NodeMap<bool> filtered(*graph, false);
			for (auto it : patch) {
				filtered[it] = true;
			}
			something_done = true;
			FilterNodes<Graph> subgraph(*graph, filtered);
			// copy subgraph
			ListDigraph temp_graph;
			DigraphCopy<FilterNodes<ListDigraph>, ListDigraph> cp(subgraph,
																  temp_graph);
			ListDigraph::NodeMap<FilterNodes<ListDigraph>::Node> nr(temp_graph);
			cp.nodeCrossRef(nr);
			cp.run();
			genetic_graph_splitter suboptimizer(&temp_graph);
			suboptimizer.nb_of_zones = divisor;
			vector<vector<ListDigraph::Node>> subsol_nodes =
				suboptimizer.solve(population);
			vector<vector<ListDigraph::Node>> mapped_subsol_nodes;
			for (auto zone : subsol_nodes) {
				vector<Node> temp_res;
				for (auto node : zone) {
					temp_res.push_back(nr[node]);
				}
				mapped_subsol_nodes.push_back(temp_res);
			}
			for (auto new_patch : mapped_subsol_nodes)
				final_sol.push_back(new_patch);
		}
		if (!something_done) {
			cont = false;
		} else
			cout << "iterative pass in " << timer.toc() << endl;
	}
	cout << "sizes = ";
	for (auto zone : final_sol) cout << zone.size() << " ";
	cout << endl;

	return final_sol;
}

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

/*static vector<vector<Node>> bipartite_until(const ListDigraph &graph, int
until_size) { vector<vector<Node>> result; return result;
}*/

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
		splitter.target_nb_of_nodes_per_zone = 30;
		vector<vector<Node>> final_sol = splitter.solve(population);
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