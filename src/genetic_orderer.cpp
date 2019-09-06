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

using std::cout;
using std::endl;
using std::string;

void genetic_orderer::init_genes(
	MySolution& p, const std::function<double(void)>& rnd01) {
}

bool genetic_orderer::eval_solution(const genetic_orderer::MySolution& p,
										   MyMiddleCost& c) {
	c.objective1 = 0;
	return true;
}

genetic_orderer::MySolution genetic_orderer::mutate(
	const genetic_orderer::MySolution& X_base, const std::function<double(void)>& rnd01,
	double shrink_scale) {
	genetic_orderer::MySolution X_new = X_base;

	return X_new;
}

genetic_orderer::MySolution genetic_orderer::crossover(
	const genetic_orderer::MySolution& X1, const genetic_orderer::MySolution& X2,
	const std::function<double(void)>& rnd01) {
    genetic_orderer::MySolution X_new;

	return X_new;
}

double genetic_orderer::calculate_SO_total_fitness(
	const GA_Type::thisChromosomeType& X) {
	// finalize the cost
	double final_cost = 0.0;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

void genetic_orderer::SO_report_generation(
	int generation_number,
	const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
	const MySolution& best_genes) {
	cout << "Generation [" << generation_number << "], " << setprecision(10)
		 << "Best=" << -last_generation.best_total_cost << ", "
		 << "Average=" << -last_generation.average_cost << ", "
		 << "genes size=" << best_genes.to_string(this) << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;
}

genetic_orderer::genetic_orderer(Graph *arg_graph, vector<Node> &arg_zone_centers) {
	//graph = arg_graph;
}
genetic_orderer::~genetic_orderer() {}

vector<Node> genetic_orderer::solve(int population_size) {
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
		&genetic_orderer::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes =
		std::bind(&genetic_orderer::init_genes, this, _1, _2);
	ga_obj.eval_solution =
		std::bind(&genetic_orderer::eval_solution, this, _1, _2);
	ga_obj.mutate =
		std::bind(&genetic_orderer::mutate, this, _1, _2, _3);
	ga_obj.crossover =
		std::bind(&genetic_orderer::crossover, this, _1, _2, _3);
	ga_obj.SO_report_generation = std::bind(
		&genetic_orderer::SO_report_generation_empty, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 30;
	ga_obj.average_stall_max = 10;
	ga_obj.elite_count = 10;

	ga_obj.solve();

	vector<Node> solution =
		ga_obj.last_generation
			.chromosomes[ga_obj.last_generation.best_chromosome_index]
			.genes.split;
	return solution;
}