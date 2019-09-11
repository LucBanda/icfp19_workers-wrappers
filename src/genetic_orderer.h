#ifndef genetic_orderer_H
#define genetic_orderer_H
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "openga.hpp"

#define MAX_NUMBER_OF_THRUSTS 5

class genetic_orderer {
	struct MySolution {
		vector<SmartGraph::Node> split;

		string to_string(genetic_orderer* optim) const {
			ostringstream res;
			for (auto it : split) {
				res << optim->graph.id(it) << " / ";
			}
			return res.str();
		}
	};

	struct MyMiddleCost {
		// This is where the results of simulation
		// is stored but not yet finalized.
		double objective1;
	};

	typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
	typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

   public:
	SmartGraph graph;
	mine_navigator& nav;
	vector<vector<Node>> zones;

	SmartGraph::NodeMap<vector<Node>> submine_to_mine_nodes;
	SmartGraph::NodeMap<vector<Booster>> boosters_map;
	SmartGraph::EdgeMap<int> cost;
	vector<SmartGraph::Node> node_list;
	vector<Node> start_zone;
	SmartGraph::Node starting_node;
	bool verbose;

	genetic_orderer(mine_navigator& arg_nav, vector<vector<Node>> &arg_zones);
	~genetic_orderer();
	vector<vector<Node>> solve(int population_size);

   private:
	pair<vector<SmartGraph::Node>, int> execute_sequence(const genetic_orderer::MySolution &p);
	bool eval_solution(const genetic_orderer::MySolution& p,
					   genetic_orderer::MyMiddleCost& c);
	void init_genes(genetic_orderer::MySolution& p,
					const std::function<double(void)>& rnd01);
	genetic_orderer::MySolution mutate(
		const genetic_orderer::MySolution& X_base,
		const std::function<double(void)>& rnd01, double shrink_scale);
	genetic_orderer::MySolution crossover(
		const genetic_orderer::MySolution& X1,
		const genetic_orderer::MySolution& X2,
		const std::function<double(void)>& rnd01);
	double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X);
	void SO_report_generation(
		int generation_number,
		const EA::GenerationType<genetic_orderer::MySolution,
								 genetic_orderer::MyMiddleCost>&
									last_generation,
		const MySolution& best_genes);
	void SO_report_generation_empty(
		int generation_number,
		const EA::GenerationType<genetic_orderer::MySolution,
								 genetic_orderer::MyMiddleCost>&
			last_generation,
		const genetic_orderer::MySolution& best_genes){};
};

#endif