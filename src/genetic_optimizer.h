#ifndef GENETIC_OPTIMIZER_H
#define GENETIC_OPTIMIZER_H
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "openga.hpp"

#define MAX_NUMBER_OF_THRUSTS 5

class genetic_optimizer;
struct MySolution;
struct MyMiddleCost;

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

class genetic_optimizer {
   public:
	int instance;
	bool verbose = false;
	agent &base_agent;

	string filename;
	mineGraph *navigator;
	vector<vector<Node>> zones;
	map<int, vector<Node>> nodes_per_degree;

	string start_string;

	int zone_id;
	Node start;
	int score;

	genetic_optimizer(int arg_instance, agent &arg_base_agent,
					  vector<vector<position>>& arg_zones, int arg_zone_id,
					  string arg_start_string);
	~genetic_optimizer();
	string solve(int population_size, int generation_max = 5000);

   private:
	bool eval_solution(const MySolution& p, MyMiddleCost& c);
	void init_genes(MySolution& p, const std::function<double(void)>& rnd01);
	MySolution mutate(const MySolution& X_base,
					  const std::function<double(void)>& rnd01,
					  double shrink_scale);
	MySolution crossover(const MySolution& X1, const MySolution& X2,
						 const std::function<double(void)>& rnd01);
	double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X);
	void SO_report_generation(
		int generation_number,
		const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
		const MySolution& best_genes);
    void SO_report_generation_empty(
		int generation_number,
		const EA::GenerationType<MySolution, MyMiddleCost>&last_generation,
        const MySolution& best_genes){};
};

struct MySolution {
	vector<pair<Node, orientation>> node_list;

	string to_string(genetic_optimizer* optim) const {
		agent ag(optim->base_agent);
		string res_string = ag.execution_map_from_node_list(node_list);
		return res_string;
	}
};

struct MyMiddleCost {
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};
#endif