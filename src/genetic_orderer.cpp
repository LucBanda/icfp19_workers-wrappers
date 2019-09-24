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
#include <set>
#include "agent.h"
#include "genetic_optimizer.h"

using std::cout;
using std::endl;
using std::string;

//static int randomfunc(int j) { return rand() % j; }

// this initializes the genes, generating a random shuffle of node orders
void genetic_orderer::init_genes(MySolution& p,
								 const std::function<double(void)>& rnd01) {

	for (int i = 0; i < node_list.size(); i++) {
		p.params.push_back((int)(rnd01() * 10.));
	}

}

int genetic_orderer::execute_sequence(const genetic_orderer::MySolution &p, vector<SmartGraph::Node> *result) {

	int result_score = 0;

	SmartGraph::NodeMap<bool> filled(graph, false);
	SmartGraph::EdgeMap<int>  length(graph, 1);

	//start with start node
	result_score = 0;
	result->push_back(starting_node);
	filled[starting_node] = true;
	SmartGraph::Node current_node = starting_node;
	int step = 0;
	int zones_to_fill = node_list.size();

	while (zones_to_fill) {
		//find bfs next nodes which are not filled
		std::vector<Arc> arcpath;
		Bfs<SmartGraph> bfs(graph);

		//provide maps to algorithm just to gain some time on allocating them in a thread
		Bfs<SmartGraph>::DistMap dist(graph);
		Bfs<SmartGraph>::PredMap predmap(graph);
		Bfs<SmartGraph>::ProcessedMap processedmap;
		Bfs<SmartGraph>::ReachedMap reachedmap(graph);
		vector<SmartGraph::Node> closests;
		bfs.distMap(dist);
		bfs.predMap(predmap);
		bfs.processedMap(processedmap);
		bfs.reachedMap(reachedmap);

		bfs.init();
		bfs.addSource(current_node);
		int mindist = INT_MAX;
		while (!bfs.emptyQueue()) {
			SmartGraph::Node n = bfs.processNextNode();
			if (!filled[n]) {
				if (dist[n] <= mindist) {
					mindist = dist[n];
					closests.push_back(n);
				} else {
					break;
				}
			}
		}
		int min_degree = INT_MAX;
		vector<SmartGraph::Node> final_candidates;
		for (const auto &closest:closests) {
			int degree = 0;
			for (SmartGraph::OutArcIt e(graph, closest); e != INVALID; ++e) {
				if (!filled[graph.target(e)]) {
					degree ++;
				}
			}
			if (degree <= min_degree) {
				min_degree = degree;
				final_candidates.push_back(closest);
			}
		}
		SmartGraph::Node nextZone;
		int size_candidates = final_candidates.size();
		if (size_candidates > 1) {
			nextZone = final_candidates[p.params[step++]%size_candidates];
		} else {
			nextZone = final_candidates[0];
		}
		result_score += dist[nextZone];
		filled[nextZone] = true;
		result->push_back(nextZone);
		zones_to_fill--;
		current_node = nextZone;

	}
	return result_score;

}

// evaluate the solution by walking through edges and adding distances.
// a factor should be applied to following sequences if a booster is in a zone
// all following distances are multiplied by 0.9 for a manipulator
// reduction of 30 steps for each fastwheel
// other calculation should be done for drill if supported
bool genetic_orderer::eval_solution(const genetic_orderer::MySolution& p,
									MyMiddleCost& c) {
	vector<SmartGraph::Node> result;
	c.objective1 = execute_sequence(p, &result);
	return true;
}

// this mutation applies a swap or reverses a sequence 50% of the times
genetic_orderer::MySolution genetic_orderer::mutate(
	const genetic_orderer::MySolution& X_base,
	const std::function<double(void)>& rnd01, double shrink_scale) {

	MySolution X_new = X_base;

	//int swap1 = rnd01() * X_new.split.size();
	int mutate_gene = rnd01() * X_new.params.size();
	X_new.params[mutate_gene] = max(0, (int)(-2 + 2 * rnd01()));
	return X_new;
}

// this crossover keeps the relative orders of nodes of X1 and X2 and combine
// them
genetic_orderer::MySolution genetic_orderer::crossover(
	const genetic_orderer::MySolution& X1,
	const genetic_orderer::MySolution& X2,
	const std::function<double(void)>& rnd01) {

	int split = rnd01() * X1.params.size();
	genetic_orderer::MySolution X_new;

	X_new.params.insert(X_new.params.begin(), X1.params.begin(), X1.params.begin() + split);
	X_new.params.insert(X_new.params.begin() + split, X2.params.begin() + split, X2.params.end());
	//X_new.split.reserve(X1.split.size());


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

genetic_orderer::genetic_orderer(navigator_factory& arg_navigators,
								 vector<vector<Node>>& zones)
	: navigators(arg_navigators),
	  submine_to_mine_nodes(graph),
	  boosters_map(graph),
	  cost(graph)
	 {

	agent ag(arg_navigators,navigators.full_nav.initialNode);
	vector<Node> manipulator_boosters;
	for (NodeIt node(ag.nav_select.navigating_nav->graph); node!= INVALID; ++node) {
		if (ag.boosters_map[ag.nav_select.navigating_nav->to_full_graph_nodes[node]] == MANIPULATOR)
			manipulator_boosters.push_back(node);
	}
	ag.collect_boosters(manipulator_boosters);
	Node start_after_boosters = navigators.masked_nav.from_full_graph_nodes[ag.robot_pos];

	masked_navigator &nav = navigators.masked_nav;
	struct timeval time;
	gettimeofday(&time, NULL);
	srand((time.tv_sec) + (time.tv_usec));
	//make the map from node of original graph to zone
	Graph::NodeMap<vector<Node>> node_to_zone_map(nav.graph);
	for (const auto zone:zones) {
		for (const auto &node:zone) {
			node_to_zone_map[node] = zone;
		}
	}
	// calculate the starting zone by applying dijstra between center nodes and
	// starting point
	vector<Node> start_zone;
	for (const auto &zone : zones) {
		if (find(zone.begin(), zone.end(), start_after_boosters) != zone.end()) {
			start_zone = zone;
		}
	}

	// create graph of zones
	for (const auto &zone : zones) {
		SmartGraph::Node u = graph.addNode();
		submine_to_mine_nodes[u] = zone;
		if (zone != start_zone) {
			node_list.push_back(u);
		}
		else
			starting_node = u;
		boosters_map[u] = nav.boosters_in_node_list(zone);
	}

	// calculate adjacent zones with bfs from the center
	for (SmartGraph::NodeIt orig(graph); orig != INVALID; ++orig) {
		set<vector<Node>> neighbors;
		for (const auto &node:submine_to_mine_nodes[orig]) {
			//iterate over all cell of zone, if a neighbor of the cell is not in the current zone, find the zone of the neigbor
			for(Graph::OutArcIt arc(nav.graph, node); arc != INVALID; ++arc) {
				auto arc_target = nav.graph.target(arc);
				if (find(submine_to_mine_nodes[orig].begin(), submine_to_mine_nodes[orig].end(), arc_target) == submine_to_mine_nodes[orig].end()) {
					//neighbor found
					auto res = node_to_zone_map[arc_target];
					neighbors.insert(res);
				}
			}
		}
		//populate edges from neighbors
		for (const auto &nb:neighbors) {
			for (SmartGraph::NodeIt dest(graph); dest!= INVALID; ++dest) {
				if (submine_to_mine_nodes[dest] == nb) {
					SmartGraph::Edge e = graph.addEdge(orig, dest);
					Bfs<Graph> bfs(nav.graph);
					bfs.run(submine_to_mine_nodes[orig][0],
								submine_to_mine_nodes[dest][0]);
					int cost_score = bfs.dist(submine_to_mine_nodes[dest][0]);
					cost[e] = cost_score;
				}
			}
		}
	}
}


genetic_orderer::~genetic_orderer() {}

// here we apply genetic algorithm to the population of pathes between all
// centers the goal is to determine the shortest path going through all zones
vector<vector<Node>> genetic_orderer::solve(int population_size) {
	using namespace std::placeholders;

	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = population_size;
	ga_obj.generation_max = 6000;
	ga_obj.calculate_SO_total_fitness =
		std::bind(&genetic_orderer::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes = std::bind(&genetic_orderer::init_genes, this, _1, _2);
	ga_obj.eval_solution =
		std::bind(&genetic_orderer::eval_solution, this, _1, _2);
	ga_obj.mutate = std::bind(&genetic_orderer::mutate, this, _1, _2, _3);
	ga_obj.crossover = std::bind(&genetic_orderer::crossover, this, _1, _2, _3);
	if (verbose) {
		ga_obj.SO_report_generation = std::bind(
			&genetic_orderer::SO_report_generation, this, _1, _2, _3);
	} else {
		ga_obj.SO_report_generation = std::bind(
			&genetic_orderer::SO_report_generation_empty, this, _1, _2, _3);
	}
	ga_obj.crossover_fraction = 0.8;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 200;
	ga_obj.average_stall_max = 5;
	ga_obj.elite_count = 20;
	ga_obj.use_quick_search = population_size < 6000;

	ga_obj.solve();

	// put result in form, get back original centers from center graph
	auto seq = ga_obj.last_generation
			.chromosomes[ga_obj.last_generation.best_chromosome_index]
			.genes;
	vector<SmartGraph::Node> seq_result;
	execute_sequence(seq, &seq_result);

	seq_result.insert(seq_result.begin(), starting_node);

	vector<vector<Node>> result;
	for (const auto &n : seq_result) {
		result.push_back(submine_to_mine_nodes[n]);
	}
	return result;
}