#include "genetic_optimizer.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "fileparser.h"
#include "functional"
#include "lemon/dijkstra.h"
#include "openga.hpp"
#include "sys/time.h"

using std::cout;
using std::endl;
using std::string;

static int randomfunc(int j) { return rand() % j; }

void genetic_optimizer::init_genes(MySolution& p,
								   const std::function<double(void)>& rnd01) {
	struct timeval time;
	gettimeofday(&time, NULL);
	srand((time.tv_sec) + (time.tv_usec));
	//p.node_list.clear();
	vector<Node> list_node = zones[zone_id];
	p.node_list.reserve(list_node.size());
	/*for (const auto it: list_node) {
		if (navigator->boosters_map[it] == MANIPULATOR) {
			orientation orient = (orientation)(rnd01() * 4);
			p.node_list.emplace_back(it, orient);
		}
	}*/
	for (int i = 1; i <= 4; i++) {
		if (nodes_per_degree.find(i) != nodes_per_degree.end()) {
			vector<Node> list_per_degree = nodes_per_degree[i];
			random_shuffle(list_per_degree.begin(), list_per_degree.end(), randomfunc);
			for (const auto& node:list_per_degree) {
				orientation orient;
				if (i == 1) {
					for (Graph::InArcIt it(navigator->graph, node); it!=INVALID; ++it) {
						switch (navigator->direction_map[it]) {
							case 'D': orient = EAST; break;
							case 'W': orient = NORTH; break;
							case 'S': orient = SOUTH; break;
							case 'A': orient = WEST; break;
						}
					}
				} else {
					orient = (orientation)(rnd01() * 4);
				}
				p.node_list.emplace_back(node, orient);
			}
		}
	}
}

bool genetic_optimizer::eval_solution(const MySolution& p, MyMiddleCost& c) {
	agent executeur(base_agent);
	//executeur.execute_seq(start_string);
	executeur.execution_map_from_node_list(p.node_list);
	int cost_to_next_zone = executeur.cost_to_next_zone(zones, zone_id + 1);

	c.objective1 = executeur.get_cost() + cost_to_next_zone;

	return true;  // solution is accepted
}

MySolution genetic_optimizer::mutate(const MySolution& X_base,
									 const std::function<double(void)>& rnd01,
									 double shrink_scale) {
	MySolution X_new = X_base;
	int nb_of_mutations = max(1, (int)(10. * rnd01() * shrink_scale));

	int swap1 = 0;
	int swap2 = 0;
	int minswap, maxswap;
	double action = rnd01();

	while (swap1 == swap2) {
		swap1 = rnd01() * X_base.node_list.size();
		swap2 = rnd01() * X_base.node_list.size();
	}

	minswap = min(swap1, swap2);
	maxswap = max(swap1, swap2);

	if (action < .5)
		reverse(X_new.node_list.begin() + minswap, X_new.node_list.begin() + maxswap);
	else
		for (int i = 0; i < nb_of_mutations; i++) {
			swap1 = rnd01() * X_base.node_list.size();
			pair<Node, orientation> node = X_new.node_list[swap1];
			orientation new_orient = (orientation)(
				(node.second + (int)(4. * rnd01())) % 4);
			X_new.node_list.erase(X_new.node_list.begin() + swap1);
			X_new.node_list.emplace(X_new.node_list.begin() + swap1, node.first,
									new_orient);
		}
	return X_new;
}

MySolution genetic_optimizer::crossover(
	const MySolution& X1, const MySolution& X2,
	const std::function<double(void)>& rnd01) {
	MySolution X_new;

	X_new.node_list.reserve(X1.node_list.size());
	int position1 = rnd01() * X1.node_list.size();
	int position2 = rnd01() * X1.node_list.size();
	while (position1 == position2) {
		position1 = rnd01() * X1.node_list.size();
		position2 = rnd01() * X1.node_list.size();
	}
	int positionmin = min(position1, position2);
	int positionmax = max(position1, position2);
	vector<Node> nodes_to_check;
	nodes_to_check.reserve(X1.node_list.size());

	for (int i = positionmin; i < positionmax; i++) {
		X_new.node_list.push_back(X1.node_list[i]);
		nodes_to_check.push_back(X1.node_list[i].first);
	}
	int j = positionmax;
	while (1) {
		pair<Node, orientation> node = X2.node_list[j++];
		if (j == (int)X1.node_list.size()) j = 0;
		int i = X2.node_list.size();
		while (find(nodes_to_check.begin(), nodes_to_check.end(), node.first) !=
				   nodes_to_check.end() &&
			   i) {
			i--;
			node = X2.node_list[j++];
			if (j == (int)X1.node_list.size()) j = 0;
		}
		if (i == 0) break;
		X_new.node_list.push_back(node);
		nodes_to_check.push_back(node.first);
	}
	return X_new;
}

double genetic_optimizer::calculate_SO_total_fitness(
	const GA_Type::thisChromosomeType& X) {
	// finalize the cost
	double final_cost = 0.0;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

void genetic_optimizer::SO_report_generation(
	int generation_number,
	const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
	const MySolution& best_genes) {
	cout << "Problem Id:" << instance << ", "
		 << "Generation [" << generation_number << "], " << setprecision(10)
		 << "Best=" << -last_generation.best_total_cost << ", "
		 << "Average=" << -last_generation.average_cost << ", "
		 << "genes size=" << best_genes.to_string(this).length() << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;
}

genetic_optimizer::genetic_optimizer(int arg_instance, agent &arg_base_agent,
									 vector<vector<position>>& arg_zones,
									 int arg_zone_id,
									 string arg_start_string):base_agent(arg_base_agent) {
	instance = arg_instance;
	zone_id = arg_zone_id;
	start_string = arg_start_string;
	navigator = base_agent.nav_select.navigating_nav;
	zones = navigator->node_from_coords(arg_zones);
	for (const auto &node: zones[zone_id]) {
		int degree = countOutArcs(navigator->graph, node);
		nodes_per_degree[degree].push_back(node);
	}
}
genetic_optimizer::~genetic_optimizer() {}

string genetic_optimizer::solve(int population_size, int generation_max) {
	using namespace std::placeholders;

	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	//ga_obj.idle_delay_us = 1;  // switch between threads quickly
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = population_size;
	ga_obj.generation_max = generation_max;
	ga_obj.calculate_SO_total_fitness =
		std::bind(&genetic_optimizer::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes = std::bind(&genetic_optimizer::init_genes, this, _1, _2);
	ga_obj.eval_solution =
		std::bind(&genetic_optimizer::eval_solution, this, _1, _2);
	ga_obj.mutate = std::bind(&genetic_optimizer::mutate, this, _1, _2, _3);
	ga_obj.crossover =
		std::bind(&genetic_optimizer::crossover, this, _1, _2, _3);
	if (verbose)
		ga_obj.SO_report_generation =
			std::bind(&genetic_optimizer::SO_report_generation, this, _1, _2, _3);
	else
		ga_obj.SO_report_generation =
			std::bind(&genetic_optimizer::SO_report_generation_empty, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 20;
	ga_obj.average_stall_max = 5;
	ga_obj.elite_count = min(10, population_size);
	ga_obj.use_quick_search = population_size < 5000;

	ga_obj.solve();
	score = ga_obj.last_generation.best_total_cost;
	return ga_obj.last_generation
		.chromosomes[ga_obj.last_generation.best_chromosome_index]
		.genes.to_string(this);
}