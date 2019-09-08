#include "genetic_orderer.h"
#include <lemon/adaptors.h>
#include <lemon/dijkstra.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "functional"
#include "genetic_splitter.h"
#include "lemon/bfs.h"
#include "openga.hpp"
#include "renderer.h"
#include "sys/time.h"

using std::cout;
using std::endl;
using std::string;

static int randomfunc(int j) { return rand() % j; }

// this initializes the genes, generating a random shuffle of node orders
void genetic_orderer::init_genes(MySolution& p,
								 const std::function<double(void)>& rnd01) {
	struct timeval time;
	gettimeofday(&time, NULL);
	srand((time.tv_sec) + (time.tv_usec));
	vector<SmartGraph::Node> nodes = node_list;
	random_shuffle(nodes.begin(), nodes.end(), randomfunc);
	p.split.insert(p.split.end(), nodes.begin(), nodes.end());
}

// evaluate the solution by walking through edges and adding distances.
// a factor should be applied to following sequences if a booster is in a zone
// all following distances are multiplied by 0.9 for a manipulator
// reduction of 30 steps for each fastwheel
// other calculation should be done for drill if supported
bool genetic_orderer::eval_solution(const genetic_orderer::MySolution& p,
									MyMiddleCost& c) {
	c.objective1 = 0;
	SmartGraph::Node orig = starting_node;
	SmartGraph::Node dest;
	c.objective1 = 0;
	for (auto it = p.split.begin(); it != p.split.end(); ++it) {
		dest = *it;
		for (SmartGraph::IncEdgeIt e(graph, orig); e != INVALID; ++e) {
			if (graph.v(e) == dest) {
				orig = dest;
				c.objective1 += cost[e];
				break;
			}
		}
	}
	return true;
}

// this mutation applies a swap or reverses a sequence 50% of the times
genetic_orderer::MySolution genetic_orderer::mutate(
	const genetic_orderer::MySolution& X_base,
	const std::function<double(void)>& rnd01, double shrink_scale) {
	MySolution X_new;
	int swap1 = 0;
	int swap2 = 0;
	int minswap, maxswap;
	double action = rnd01();
	while (swap1 == swap2) {
		swap1 = rnd01() * X_base.split.size();
		swap2 = rnd01() * X_base.split.size();
	}

	minswap = min(swap1, swap2);
	maxswap = max(swap1, swap2);

	if (action < .5) {
		// this reverse the sequence between minswap and maxswap
		int i;
		for (i = 0; i < minswap; i++) {
			X_new.split.push_back(X_base.split[i]);
		}
		for (i = maxswap - 1; i >= minswap; i--) {
			X_new.split.push_back(X_base.split[i]);
		}
		for (i = maxswap; i < X_base.split.size(); i++) {
			X_new.split.push_back(X_base.split[i]);
		}
	} else {
		// this swaps swap1 and swap2
		X_new.split = X_base.split;
		iter_swap(X_new.split.begin() + swap1, X_new.split.begin() + swap2);
	}

	return X_new;
}

// this crossover keeps the relative orders of nodes of X1 and X2 and combine
// them
genetic_orderer::MySolution genetic_orderer::crossover(
	const genetic_orderer::MySolution& X1,
	const genetic_orderer::MySolution& X2,
	const std::function<double(void)>& rnd01) {
	genetic_orderer::MySolution X_new;

	int position1 = rnd01() * X1.split.size();
	int position2 = rnd01() * X1.split.size();
	while (position1 == position2) {
		position1 = rnd01() * X1.split.size();
		position2 = rnd01() * X1.split.size();
	}
	int positionmin = min(position1, position2);
	int positionmax = max(position1, position2);
	vector<SmartGraph::Node> new_nodes;

	// start new nodes by pushing the middle of X1
	for (int i = positionmin; i < positionmax; i++) {
		new_nodes.push_back(X1.split[i]);
	}

	// then complete the sequence by getting X2 nodes if they are not already in
	// the list
	int j = positionmax;
	while (1) {
		SmartGraph::Node node = X2.split[j++];
		if (j == (int)X1.split.size()) j = 0;
		int i = X2.split.size();
		while (find(new_nodes.begin(), new_nodes.end(), node) !=
				   new_nodes.end() &&
			   i) {
			i--;
			node = X2.split[j++];
			if (j == (int)X1.split.size()) j = 0;
		}
		if (i == 0) break;
		new_nodes.push_back(node);
	}

	X_new.split = new_nodes;

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
		 << "Average=" << -last_generation.average_cost
		 << ", "
		 //<< "genes size=" << best_genes.to_string(this) << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;
}

genetic_orderer::genetic_orderer(mine_navigator& arg_nav,
								 vector<Node>& arg_zone_centers)
	: nav(arg_nav), submine_to_mine_nodes(graph), cost(graph) {
	Dijkstra<Graph> dijkstra(nav.graph, nav.length);
	dijkstra.run(nav.robot_pos);
	int smallest_len = countNodes(nav.graph);

	// calculate the starting zone by applying dijstra between center nodes and
	// starting point
	for (auto zone : arg_zone_centers) {
		if (dijkstra.distMap()[zone] < smallest_len) {
			start_zone = zone;
			smallest_len = dijkstra.distMap()[zone];
		}
	}

	// create graph of centers
	for (auto zone : arg_zone_centers) {
		SmartGraph::Node u = graph.addNode();
		submine_to_mine_nodes[u] = zone;
		if (zone != start_zone)
			node_list.push_back(u);
		else
			starting_node = u;
	}

	// calculate distances of centers and add that as edges of the graph
	for (SmartGraph::NodeIt orig(graph); orig != INVALID; ++orig) {
		for (SmartGraph::NodeIt dest(graph); dest != INVALID; ++dest) {
			if (orig != dest) {
				Dijkstra<Graph> dijkstra(nav.graph, nav.length);
				dijkstra.run(submine_to_mine_nodes[orig],
							 submine_to_mine_nodes[dest]);
				SmartGraph::Edge e = graph.addEdge(orig, dest);
				int cost_score = dijkstra.dist(submine_to_mine_nodes[dest]);
				cost[e] = cost_score;
			}
		}
	}
}

genetic_orderer::~genetic_orderer() {}

// here we apply genetic algorithm to the population of pathes between all
// centers the goal is to determine the shortest path going through all zones
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
	ga_obj.calculate_SO_total_fitness =
		std::bind(&genetic_orderer::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes = std::bind(&genetic_orderer::init_genes, this, _1, _2);
	ga_obj.eval_solution =
		std::bind(&genetic_orderer::eval_solution, this, _1, _2);
	ga_obj.mutate = std::bind(&genetic_orderer::mutate, this, _1, _2, _3);
	ga_obj.crossover = std::bind(&genetic_orderer::crossover, this, _1, _2, _3);
	ga_obj.SO_report_generation = std::bind(
		&genetic_orderer::SO_report_generation_empty, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 50;
	ga_obj.average_stall_max = 30;
	ga_obj.elite_count = 100;

	ga_obj.solve();

	// put result in form, get back original centers from center graph
	vector<SmartGraph::Node> solution =
		ga_obj.last_generation
			.chromosomes[ga_obj.last_generation.best_chromosome_index]
			.genes.split;
	solution.insert(solution.begin(), starting_node);
	vector<Node> result;
	for (auto n : solution) {
		result.push_back(submine_to_mine_nodes[n]);
	}
	return result;
}