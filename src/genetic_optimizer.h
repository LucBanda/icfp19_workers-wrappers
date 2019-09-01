#ifndef GENETIC_OPTIMIZER_H
#define GENETIC_OPTIMIZER_H
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "agent.h"
#include "openga.hpp"
#
#define MAX_NUMBER_OF_THRUSTS 5

class genetic_optimizer;
struct MySolution;
struct MyMiddleCost;

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

class genetic_optimizer {
    public:
        int instance;
        string filename;
		mine_state *base_mine;
        mine_navigator *navigator;

        genetic_optimizer(int instance);
        ~genetic_optimizer();
        bool solve(int population_size);

    private:
        bool eval_solution(const MySolution& p, MyMiddleCost& c);
        void init_genes(MySolution& p, const std::function<double(void)>& rnd01);
        MySolution mutate(const MySolution& X_base,
				  const std::function<double(void)>& rnd01,
				  double shrink_scale);
        MySolution crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01);
        double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X);
        void SO_report_generation(int generation_number,
                    const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
                    const MySolution& best_genes);
};

struct MySolution {
	vector<ListDigraph::Node> node_list;

	string to_string(genetic_optimizer * optim) const {
        mine_state mine(optim->base_mine);
        agent ag(&mine, optim->navigator);
        return ag.execution_map_from_node_list(node_list);
	}
};

struct MyMiddleCost {
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};
#endif