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
	p.node_list.clear();
	vector<Node> list_node = zones[zone_id];
	random_shuffle(std::begin(list_node), std::end(list_node), randomfunc);
	for (int i = 0; i < list_node.size(); i++) {
		p.node_list.emplace_back(list_node[i], (orientation)(rnd01() * 4));
	}
}

bool genetic_optimizer::eval_solution(const MySolution& p, MyMiddleCost& c) {
	mine_state mine(&base_mine);
	agent executeur(&mine, navigator, start);
	executeur.execution_map_from_node_list(p.node_list);
	Node exit_node = executeur.last_node;

	int cost_to_next_zone = 0;
	if (zone_id < zones.size() - 1) {
		std::vector<Arc> arcpath;
		Dijkstra<Graph> dijkstra(navigator->graph, navigator->length);
		dijkstra.run(zones[zone_id + 1][0], exit_node);

		// get back path
		Dijkstra<Graph>::Path path = dijkstra.path(exit_node);
		for (Dijkstra<Graph>::Path::RevArcIt it(path); it != INVALID;
			 ++it) {
			if (find(zones[zone_id + 1].begin(), zones[zone_id + 1].end(),
					 navigator->graph.source(it)) != zones[zone_id + 1].end())
				break;
			cost_to_next_zone++;
		}
	}
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
	//double action = rnd01();
	while (swap1 == swap2) {
		swap1 = rnd01() * X_base.node_list.size();
		swap2 = rnd01() * X_base.node_list.size();
	}

	minswap = min(swap1, swap2);
	maxswap = max(swap1, swap2);

	MySolution X_input = X_new;
	X_new.node_list.clear();
	int i;
	for (i = 0; i < minswap; i++) {
		X_new.node_list.push_back(X_input.node_list[i]);
	}
	for (i = maxswap - 1; i >= minswap; i--) {
		X_new.node_list.push_back(X_input.node_list[i]);
	}
	for (i = maxswap; i < X_input.node_list.size(); i++) {
		X_new.node_list.push_back(X_input.node_list[i]);
	}

	for (int i = 0; i < nb_of_mutations; i++) {
		/*int swap1 = 0;
		int swap2 = 0;
		int minswap, maxswap;
		double action = rnd01();
		while (swap1 == swap2) {
			swap1 = rnd01() * X_base.node_list.size();
			swap2 = rnd01() * X_base.node_list.size();
		}

		minswap = min(swap1, swap2);
		maxswap = max(swap1, swap2);

		if (action < .5) {

		} else {*/
		swap1 = rnd01() * X_base.node_list.size();
			pair<Node, orientation> node;
			node = X_new.node_list[swap1];
			orientation new_orient = (orientation)(
				(X_new.node_list[swap1].second + (int)(4. * rnd01())) % 4);
			X_new.node_list.erase(X_new.node_list.begin() + swap1);
			X_new.node_list.emplace(X_new.node_list.begin() + swap1, node.first,
									new_orient);

			node = X_new.node_list[swap2];
			new_orient = (orientation)(
				(X_new.node_list[swap2].second + (int)(4. * rnd01())) % 4);
			X_new.node_list.erase(X_new.node_list.begin() + swap2);
			X_new.node_list.emplace(X_new.node_list.begin() + swap2, node.first,
									new_orient);
		//}
	}
	return X_new;
}

MySolution genetic_optimizer::crossover(
	const MySolution& X1, const MySolution& X2,
	const std::function<double(void)>& rnd01) {
	MySolution X_new;


	int position1 = rnd01() * X1.node_list.size();
	int position2 = rnd01() * X1.node_list.size();
	while (position1 == position2) {
		position1 = rnd01() * X1.node_list.size();
		position2 = rnd01() * X1.node_list.size();
	}
	int positionmin = min(position1, position2);
	int positionmax = max(position1, position2);
	vector<pair<Node, orientation>> new_nodes;
	vector<Node> nodes_to_check;

	for (int i = positionmin; i < positionmax; i++) {
		new_nodes.push_back(X1.node_list[i]);
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
		new_nodes.push_back(node);
		nodes_to_check.push_back(node.first);
	}
	X_new.node_list = new_nodes;
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
		 << "genes size=" << best_genes.to_string(this).first.length() << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;
}

genetic_optimizer::genetic_optimizer(int arg_instance, mine_navigator* arg_nav, mine_state *mine,
									 vector<vector<position>>& arg_zones,
									 int arg_zone_id, Node start_node,
									 string start_string)
	: base_mine(mine) {
	instance = arg_instance;
	start = start_node;
	navigator = arg_nav;
	zone_id = arg_zone_id;
	base_mine.apply_command(start_string);
	zones = navigator->node_from_coords(arg_zones);
}
genetic_optimizer::~genetic_optimizer() {}

pair<string, Node> genetic_optimizer::solve(int population_size, int generation_max) {
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
	ga_obj.SO_report_generation =
		std::bind(&genetic_optimizer::SO_report_generation_empty, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 20;
	ga_obj.average_stall_max = 20;
	ga_obj.elite_count = 50;
	ga_obj.use_quick_search = population_size < 6000;

	ga_obj.solve();
	score = ga_obj.last_generation.best_total_cost;
	return ga_obj.last_generation
		.chromosomes[ga_obj.last_generation.best_chromosome_index]
		.genes.to_string(this);
}