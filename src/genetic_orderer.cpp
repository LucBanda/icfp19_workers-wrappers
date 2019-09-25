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

static int randomfunc(int j) { return rand() % j; }

// this initializes the genes, generating a random shuffle of node orders
void genetic_orderer::init_genes(MySolution& p,
								 const std::function<double(void)>& rnd01) {

	//init booster order
	for (NodeIt node(base_agent.nav_select.navigating_nav->graph); node!= INVALID; ++node) {
		if (base_agent.nav_select.navigating_nav->boosters_map[node] == MANIPULATOR)
			p.booster_order.push_back(node);
	}
	random_shuffle(p.booster_order.begin(), p.booster_order.end(), randomfunc);

	//init choices for zone walk
	vector<int> choices;
	for (int i = 0; i < countNodes(graph); i++)
		p.zone_choice.push_back((int)(rnd01() * 10.));

	//init zones
	for (SmartGraph::NodeIt zone(graph); zone != INVALID; ++zone) {
		vector<Node> list_node = zone_to_masked_nodes[zone];

		p.node_order[zone].reserve(list_node.size());
		for (int i = 1; i <= 4; i++) {
			if (nodes_per_degree[zone].find(i) != nodes_per_degree[zone].end()) {
				vector<Node> list_per_degree = nodes_per_degree[zone][i];
				random_shuffle(list_per_degree.begin(), list_per_degree.end(), randomfunc);
				for (const auto& node:list_per_degree) {
					orientation orient;
					if (i == 1) {
						for (Graph::InArcIt it(base_agent.nav_select.navigating_nav->graph, node); it!=INVALID; ++it) {
							switch (base_agent.nav_select.navigating_nav->direction_map[it]) {
								case 'D': orient = EAST; break;
								case 'W': orient = NORTH; break;
								case 'S': orient = SOUTH; break;
								case 'A': orient = WEST; break;
							}
						}
					} else {
						orient = NORTH/*(orientation)(rnd01() * 4)*/;
					}
					p.node_order[zone].emplace_back(node, orient);
				}
			}
		}
	}
	//}
}

pair<int, string> genetic_orderer::execute_sequence(const genetic_orderer::MySolution &p) {
	string result_str;
	agent ag(base_agent);

	SmartGraph::NodeMap<bool> filled(graph, false);
	SmartGraph::EdgeMap<int>  length(graph, 1);

	result_str = ag.collect_boosters(p.booster_order);

	//start with start node
	SmartGraph::Node current_node = masked_node_to_zone[ag.navigators.masked_nav.from_full_graph_nodes[ag.robot_pos]] ;

	int step = 0;
	int zones_to_fill = countNodes(graph);
	//filled[current_node] = true;
	result_str += ag.execution_map_from_node_list(p.node_order.at(current_node));
	filled[current_node] = true;
	zones_to_fill--;

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
			nextZone = final_candidates[p.zone_choice[step++]%size_candidates];
		} else {
			nextZone = final_candidates[0];
		}
		filled[nextZone] = true;
		zones_to_fill--;
		current_node = nextZone;
		result_str += ag.execution_map_from_node_list(p.node_order.at(current_node));
	}
	return make_pair(ag.time_step, result_str);
}

// evaluate the solution by walking through edges and adding distances.
// a factor should be applied to following sequences if a booster is in a zone
// all following distances are multiplied by 0.9 for a manipulator
// reduction of 30 steps for each fastwheel
// other calculation should be done for drill if supported
bool genetic_orderer::eval_solution(const genetic_orderer::MySolution& p,
									MyMiddleCost& c) {
	string str;
	c.objective1 = execute_sequence(p).first;
	return true;
}

// this mutation applies a swap or reverses a sequence 50% of the times
genetic_orderer::MySolution genetic_orderer::mutate(
	const genetic_orderer::MySolution& X_base,
	const std::function<double(void)>& rnd01, double shrink_scale) {

	MySolution X_new = X_base;
	int nb_of_mutations = max(1, (int)(100. * rnd01() * shrink_scale));

	for (int i = 0; i < nb_of_mutations; i++) {
		double action = rnd01();
		if (action < 0.05 && X_new.booster_order.size() > 1) { // 5% of chance to mutate boost order
			int swap1 = rnd01() * X_new.booster_order.size();
			int swap2 = rnd01() * X_new.booster_order.size();
			while (swap1 == swap2) {
				swap1 = rnd01() * X_new.booster_order.size();
				swap2 = rnd01() * X_new.booster_order.size();
			}
			iter_swap(X_new.booster_order.begin() + swap1, X_new.booster_order.begin() + swap2);
		} else if (action < 0.1) { // 5% chances to mutate choices of zones
			int mutate_gene = rnd01() * X_new.zone_choice.size();
			X_new.zone_choice[mutate_gene] = max(0, (int)(-2 + 2 * rnd01()));
		} else {// 90% of chances to mutate the zones
			int node_id_to_mutate = rnd01() * countNodes(graph);
			SmartGraph::Node zone_to_mutate;
			for (SmartGraph::NodeIt it(graph); it != INVALID; ++it) {
				node_id_to_mutate--;
				if (node_id_to_mutate < 0) {
					zone_to_mutate = it;
					break;
				}
			}
			if (action < 0.55) {
				int swap1 = 0;
				int swap2 = 0;
				while (swap1 == swap2) {
					swap1 = rnd01() * X_new.node_order[zone_to_mutate].size();
					swap2 = rnd01() * X_new.node_order[zone_to_mutate].size();
				}

				int minswap = min(swap1, swap2);
				int maxswap = max(swap1, swap2);
				reverse(X_new.node_order[zone_to_mutate].begin() + minswap, X_new.node_order[zone_to_mutate].begin() + maxswap);
			} else {
				int nb_of_direction_changes = max(1, (int)(10. * rnd01() * shrink_scale));
				for (int i = 0; i < nb_of_direction_changes; i++) {
					int swap1 = rnd01() * X_new.node_order[zone_to_mutate].size();
					pair<Node, orientation> node = X_new.node_order[zone_to_mutate][swap1];
					orientation new_orient = (orientation)(
						(node.second + (int)(4. * rnd01())) % 4);
					X_new.node_order[zone_to_mutate].erase(X_new.node_order[zone_to_mutate].begin() + swap1);
					X_new.node_order[zone_to_mutate].emplace(X_new.node_order[zone_to_mutate].begin() + swap1, node.first,
											new_orient);
				}
			}
		}
	}
	return X_new;
}

// this crossover keeps the relative orders of nodes of X1 and X2 and combine
// them
genetic_orderer::MySolution genetic_orderer::crossover(
	const genetic_orderer::MySolution& X1,
	const genetic_orderer::MySolution& X2,
	const std::function<double(void)>& rnd01) {

	//cross over zone choice easily
	int split = rnd01() * X1.zone_choice.size();
	genetic_orderer::MySolution X_new;

	X_new.zone_choice.insert(X_new.zone_choice.begin(), X1.zone_choice.begin(), X1.zone_choice.begin() + split);
	X_new.zone_choice.insert(X_new.zone_choice.begin() + split, X2.zone_choice.begin() + split, X2.zone_choice.end());

	//XCO for booster list
	if (X_new.booster_order.size() > 1) {
		X_new.booster_order.reserve(X1.booster_order.size());
		int position1 = rnd01() * X1.booster_order.size();
		int position2 = rnd01() * X1.booster_order.size();
		while (position1 == position2) {
			position1 = rnd01() * X1.booster_order.size();
			position2 = rnd01() * X1.booster_order.size();
		}
		int positionmin = min(position1, position2);
		int positionmax = max(position1, position2);
		vector<Node> nodes_to_check;
		nodes_to_check.reserve(X1.booster_order.size());

		for (int i = positionmin; i < positionmax; i++) {
			X_new.booster_order.push_back(X1.booster_order[i]);
			nodes_to_check.push_back(X1.booster_order[i]);
		}
		int j = positionmax;
		while (1) {
			Node node = X2.booster_order[j++];
			if (j == (int)X1.booster_order.size()) j = 0;
			int i = X2.booster_order.size();
			while (find(nodes_to_check.begin(), nodes_to_check.end(), node) !=
					nodes_to_check.end() &&
				i) {
				i--;
				node = X2.booster_order[j++];
				if (j == (int)X1.booster_order.size()) j = 0;
			}
			if (i == 0) break;
			X_new.booster_order.push_back(node);
			nodes_to_check.push_back(node);
		}
	} else X_new.booster_order = X1.booster_order;

	//XCO each zones
	for (SmartGraph::NodeIt zone(graph); zone != INVALID; ++zone) {
	/*	X_new.node_order[zone].reserve(X1.node_order.at(zone).size());
		int position1 = rnd01() * X1.node_order.at(zone).size();
		int position2 = rnd01() * X1.node_order.at(zone).size();
		while (position1 == position2) {
			position1 = rnd01() * X1.node_order.at(zone).size();
			position2 = rnd01() * X1.node_order.at(zone).size();
		}
		int positionmin = min(position1, position2);
		int positionmax = max(position1, position2);
		vector<Node> nodes_to_check;
		nodes_to_check.reserve(X1.node_order.at(zone).size());

		for (int i = positionmin; i < positionmax; i++) {
			X_new.node_order[zone].push_back(X1.node_order.at(zone)[i]);
			nodes_to_check.push_back(X1.node_order.at(zone)[i].first);
		}
		int j = positionmax;
		while (1) {
			pair<Node, orientation> node = X2.node_order.at(zone)[j++];
			if (j == (int)X1.node_order.at(zone).size()) j = 0;
			int i = X2.node_order.at(zone).size();
			while (find(nodes_to_check.begin(), nodes_to_check.end(), node.first) !=
					nodes_to_check.end() &&
				i) {
				i--;
				node = X2.node_order.at(zone)[j++];
				if (j == (int)X1.node_order.at(zone).size()) j = 0;
			}
			if (i == 0) break;
			X_new.node_order[zone].push_back(node);
			nodes_to_check.push_back(node.first);
		}*/
		double which_one = rnd01();
		if (which_one < 0.5) {
			X_new.node_order[zone] = X1.node_order.at(zone);
		} else {
			X_new.node_order[zone] = X2.node_order.at(zone);
		}
	}

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

	ofstream output_file;
	output_file.open("./results/" + to_string(base_agent.navigators.instance) + ".txt",
					std::ofstream::trunc);
	output_file << best_genes.to_string(this) << endl;
	output_file.flush();
	output_file.close();

}

genetic_orderer::genetic_orderer(agent &arg_base_agent,
								 vector<vector<Node>>& zones)
	: base_agent(arg_base_agent),
	  zone_to_masked_nodes(graph),
	  masked_node_to_zone(base_agent.navigators.masked_nav.graph)
	 {

	struct timeval time;
	gettimeofday(&time, NULL);
	srand((time.tv_sec) + (time.tv_usec));

// create graph of zones
	for (const auto &zone : zones) {
		SmartGraph::Node u = graph.addNode();
		zone_to_masked_nodes[u] = zone;
		for (const auto node:zone) {
			masked_node_to_zone[node] = u;
		}
	}
	//masked_navigator &nav = base_agent.navigators.masked_nav;

	// calculate adjacent zones with bfs from the center
	for (SmartGraph::NodeIt orig(graph); orig != INVALID; ++orig) {
		set<SmartGraph::Node> neighbors;
		for (const auto &node:zone_to_masked_nodes[orig]) {
			//iterate over all cell of zone, if a neighbor of the cell is not in the current zone, find the zone of the neigbor
			for(Graph::OutArcIt arc(base_agent.navigators.masked_nav.graph, node); arc != INVALID; ++arc) {
				auto arc_target = base_agent.navigators.masked_nav.graph.target(arc);
				if (find(zone_to_masked_nodes[orig].begin(), zone_to_masked_nodes[orig].end(), arc_target) == zone_to_masked_nodes[orig].end()) {
					//neighbor found
					auto res = masked_node_to_zone[arc_target];
					neighbors.insert(res);
				}
			}
		}
		//populate edges from neighbors
		for (const auto &nb:neighbors) {
			for (SmartGraph::NodeIt dest(graph); dest!= INVALID; ++dest) {
				if (dest == nb) {
					graph.addEdge(orig, dest);
				}
			}
		}
	}
	for (NodeIt node(base_agent.navigators.masked_nav.graph); node != INVALID; ++node) {
		int degree = countOutArcs(base_agent.navigators.masked_nav.graph, node);
		nodes_per_degree[masked_node_to_zone[node]][degree].push_back(node);
	}
}


genetic_orderer::~genetic_orderer() {}

// here we apply genetic algorithm to the population of pathes between all
// centers the goal is to determine the shortest path going through all zones
string genetic_orderer::solve(int population_size) {
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
	ga_obj.elite_count = 30;
	ga_obj.use_quick_search = population_size < 6000;

	ga_obj.solve();

	// put result in form, get back original centers from center graph
	auto seq = ga_obj.last_generation
			.chromosomes[ga_obj.last_generation.best_chromosome_index]
			.genes;
	return seq.to_string(this);
}