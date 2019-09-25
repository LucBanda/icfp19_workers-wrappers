#ifndef genetic_orderer_H
#define genetic_orderer_H
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "openga.hpp"

class genetic_orderer {
	struct MySolution {
		vector<int> zone_choice;
		vector<Node> booster_order;
		map<SmartGraph::Node, vector<pair<Node, orientation>>> node_order;

		string to_string(genetic_orderer* optim) const {
			auto result = optim->execute_sequence(*this);
			return result.second;
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
	agent &base_agent;

	SmartGraph::NodeMap<vector<Node>> zone_to_masked_nodes;
	Graph::NodeMap<SmartGraph::Node> masked_node_to_zone;
	map<SmartGraph::Node, map<int, vector<Node>>> nodes_per_degree;
	bool verbose;

	genetic_orderer(agent &arg_base_agent, vector<vector<Node>> &arg_zones);
	~genetic_orderer();
	string solve(int population_size);
	int score;

   private:
	pair<int, string> execute_sequence(const genetic_orderer::MySolution &p);
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