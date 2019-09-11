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

pair<vector<SmartGraph::Node>, int> genetic_orderer::execute_sequence(const genetic_orderer::MySolution &p) {
	vector<SmartGraph::Node> result_order;
	int result_score = 0;

	SmartGraph::Node orig = starting_node;
	SmartGraph::NodeMap<bool> filled(graph, false);
	SmartGraph::EdgeMap<int> length(graph, 1);

	SmartGraph::Node dest;
	int manip_boosters_catches = 0;
	int fastwheel_boosters_credit = 0;

	//start with start node
	result_score = start_zone.size(); //get its cost
	filled[starting_node] = true;
	for (SmartGraph::IncEdgeIt e(graph, starting_node); e != INVALID; ++e) {
		length[e] = cost[e]; // new length if going through a node is approximated to the distance of the centers
	}
	//result_order.push_back(starting_node);

	for (auto it = p.split.begin(); it != p.split.end(); ++it) {
		dest = *it;
		if (filled[dest]) {
			//if next node to visit is filled, do not take it into account
			continue;
		}
		Dijkstra<SmartGraph, SmartGraph::EdgeMap<int>> dijkstra(graph, length);

		dijkstra.run(orig, dest);
		//auto path = dijkstra.path(dest);
		vector<SmartGraph::Edge> pathEdges;
		vector<SmartGraph::Node> pathNodes;
		SmartGraph::Edge predEdge = dijkstra.predArc(dest);
		SmartGraph::Node predNode = dijkstra.predNode(dest);
		while (predEdge != INVALID) {
			pathEdges.push_back(predEdge);
			pathNodes.push_back(predNode);
			predEdge = dijkstra.predArc(predNode);
			predNode = dijkstra.predNode(predNode);
		}
		reverse(pathEdges.begin(), pathEdges.end());
		reverse(pathNodes.begin(), pathNodes.end());
		pathNodes.push_back(dest);
		int path_cost = 0;
		int zone_cost = 0;
		for (int i = 0; i < pathNodes.size(); i++) {
			auto zone = pathNodes[i];
			if (zone != dest) {
				auto edge = pathEdges[i];
				path_cost += length[edge];
			}
			if (!filled[zone]) {
				zone_cost += submine_to_mine_nodes[zone].size() / (1. + manip_boosters_catches / 2.);
				result_order.push_back(zone);
				filled[zone] = true;
				// from now on, dest is considered validated as well as non validated nodes on the path
				// update length map for next dijkstras
				for (SmartGraph::IncEdgeIt e(graph, zone); e != INVALID; ++e) {
					length[e] = cost[e]; // new length if going through a node is approximated to the distance of the centers
				}
							//collect boosters
				for (auto boost:boosters_map[zone])
					if (boost == MANIPULATOR) {
						manip_boosters_catches++;
					} else if (boost == FASTWHEEL) {
						fastwheel_boosters_credit += 50;
					}
			}
		}
		orig = dest;
		//here we should consider discount of fastwheel
		result_score += zone_cost + path_cost;
	}
	return make_pair(result_order, result_score);

}
// evaluate the solution by walking through edges and adding distances.
// a factor should be applied to following sequences if a booster is in a zone
// all following distances are multiplied by 0.9 for a manipulator
// reduction of 30 steps for each fastwheel
// other calculation should be done for drill if supported
bool genetic_orderer::eval_solution(const genetic_orderer::MySolution& p,
									MyMiddleCost& c) {

	auto result = execute_sequence(p);
	c.objective1 = result.second;
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
								 vector<vector<Node>>& zones)
	: nav(arg_nav), submine_to_mine_nodes(graph), boosters_map(graph), cost(graph) {


	//make the map from node of original graph to zone
	Graph::NodeMap<vector<Node>> node_to_zone_map(nav.graph);
	for (auto zone:zones) {
		for (auto node:zone) {
			node_to_zone_map[node] = zone;
		}
	}

	// calculate the starting zone by applying dijstra between center nodes and
	// starting point
	for (auto zone : zones) {
		if (find(zone.begin(), zone.end(), nav.robot_pos) != zone.end()) {
			start_zone = zone;
		}
	}


	// create graph of zones
	for (auto zone : zones) {
		SmartGraph::Node u = graph.addNode();
		submine_to_mine_nodes[u] = zone;
		if (zone != start_zone) {
			node_list.push_back(u);
		}
		else
			starting_node = u;
		boosters_map[u] = arg_nav.boosters_in_node_list(zone);
	}

	// calculate adjacent zones with bfs from the center
	for (SmartGraph::NodeIt orig(graph); orig != INVALID; ++orig) {
		set<vector<Node>> neighbors;
		for (auto node:submine_to_mine_nodes[orig]) {
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
		for (auto nb:neighbors) {
			for (SmartGraph::NodeIt dest(graph); dest!= INVALID; ++dest) {
				if (submine_to_mine_nodes[dest] == nb) {
					SmartGraph::Edge e = graph.addEdge(orig, dest);
					Dijkstra<Graph> dijkstra(nav.graph, nav.length);
					dijkstra.run(submine_to_mine_nodes[orig][0],
								submine_to_mine_nodes[dest][0]);
					int cost_score = dijkstra.dist(submine_to_mine_nodes[dest][0]);
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
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 30;
	ga_obj.average_stall_max = 10;
	ga_obj.elite_count = 10;

	ga_obj.solve();

	// put result in form, get back original centers from center graph
	auto seq = ga_obj.last_generation
			.chromosomes[ga_obj.last_generation.best_chromosome_index]
			.genes;
	auto seq_result = execute_sequence(seq);

	seq_result.first.insert(seq_result.first.begin(), starting_node);

	vector<vector<Node>> result;
	for (auto n : seq_result.first) {
		result.push_back(submine_to_mine_nodes[n]);
	}
	return result;
}