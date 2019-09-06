#ifndef genetic_graph_splitter_H
#define genetic_graph_splitter_H
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "agent.h"
#include "openga.hpp"
#
#define MAX_NUMBER_OF_THRUSTS 5

class genetic_graph_splitter;
struct MySolution;
struct MyMiddleCost;

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

class genetic_graph_splitter {
    public:
        Graph *graph;
        int nb_of_nodes;
        int nb_of_node_per_zone;
        int nb_of_zones;
        int instance;
        vector<Node> node_list;

        genetic_graph_splitter(Graph *arg_graph);
        ~genetic_graph_splitter();
        vector<vector<Node>> solve(int population_size);
        vector<vector<Node>> partition_graph_with_split(const vector<pair<Node, int>> &loc_split, double *score);
        vector<vector<Node>> &fix_solution(vector<vector<Node>> &solution);

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
        void SO_report_generation_empty(int generation_number,
                    const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
                    const MySolution& best_genes) {};
};

struct MySolution {
	vector<pair<Node, int>> split;

	string to_string(genetic_graph_splitter * optim) const {
        ostringstream res;
        for (auto it:split) {
            res << "(" << optim->graph->id(it.first) << " , " << it.second << ") / ";
        }
        return res.str();
	}
};

struct MyMiddleCost {
	// This is where the results of simulation
	// is stored but not yet finalized.
	double objective1;
};

class global_graph_splitter {
    public:
    Graph *graph;
    int target_nb_of_nodes_per_zone;

    global_graph_splitter(Graph *arg_graph);
    ~global_graph_splitter() {};
    vector<vector<Node>> solve(int population_size);
};
#endif