#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "openga.hpp"
#include "genetic_optimizer.h"
#include "functional"
#include "sys/time.h"

using std::cout;
using std::endl;
using std::string;

int randomfunc(int j)
{
    return rand() % j;
}

void genetic_optimizer::init_genes(MySolution& p, const std::function<double(void)>& rnd01) {
	struct timeval time;
     gettimeofday(&time,NULL);

     // microsecond has 1 000 000
     // Assuming you did not need quite that accuracy
     // Also do not assume the system clock has that accuracy.
    srand((time.tv_sec) + (time.tv_usec));
	//srand(unsigned(time(NULL)));
	vector<ListDigraph::Node> list_node = navigator->get_node_list();
	p.node_list.clear();
	//std::reverse(list_node.begin(), list_node.end());
	random_shuffle(std::begin(list_node), std::end(list_node), randomfunc);
	for (auto it = list_node.begin(); it != list_node.end(); ++it) {
		int depth = max(10., list_node.size() * rnd01());
		if (find(p.node_list.begin(), p.node_list.end(), *it) == p.node_list.end()) {
			vector<ListDigraph::Node> bfsresult = navigator->get_bfs_from_node(*it, depth);
			for(auto res = bfsresult.begin(); res != bfsresult.end(); ++res)
				if (find(p.node_list.begin(), p.node_list.end(), *res) == p.node_list.end())
					p.node_list.push_back(*res);
		}
	}
	/*for (int i = 0; i < p.node_list.size(); i++)
		cout << navigator->graph.id(p.node_list[i]) << " ";
	cout << endl;*/
	//p.node_list = list_node;
}

bool genetic_optimizer::eval_solution(const MySolution& p, MyMiddleCost& c) {
	mine_state mine(base_mine);

	agent executeur(&mine, navigator);

	//executeur.set_execution_map(p.execution);
	executeur.execution_map_from_node_list(p.node_list);
	c.objective1 = executeur.get_cost();
	if (c.objective1 == 0) {
		cout << "issue" << endl;
	}

	return true;  // solution is accepted
}

MySolution genetic_optimizer::mutate(const MySolution& X_base,
				  const std::function<double(void)>& rnd01,
				  double shrink_scale) {
	MySolution X_new;
	int swap1 = 0;
	int swap2 = 0;
	int minswap, maxswap;
	double action = rnd01();
	while (swap1 == swap2) {
		swap1 = rnd01() * X_base.node_list.size();
		swap2 = rnd01() * X_base.node_list.size();
	}

	minswap = min(swap1, swap2);
	maxswap = max(swap1, swap2);

	if (action < .5) {
		int i;
		for (i = 0; i < minswap; i++) {
			X_new.node_list.push_back(X_base.node_list[i]);
		}
		for (i = maxswap-1; i >= minswap; i--) {
			X_new.node_list.push_back(X_base.node_list[i]);
		}
		for (i = maxswap; i < X_base.node_list.size(); i++) {
			X_new.node_list.push_back(X_base.node_list[i]);
		}
	} else {
		X_new.node_list = X_base.node_list;
		iter_swap(X_new.node_list.begin() + swap1, X_new.node_list.begin() + swap2);
	}
	/*cout << "base_node=";
	for (int i = 0; i < X_base.node_list.size(); i++)
		cout << navigator->graph.id(X_base.node_list[i]) << " ";
	cout << endl;
	cout << "new_node=";
	for (int i = 0; i < X_new.node_list.size(); i++)
		cout << navigator->graph.id(X_new.node_list[i]) << " ";
	cout << endl;*/
	return X_new;
}

MySolution genetic_optimizer::crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01) {
	MySolution X_new;
	int position1 = rnd01() * X1.node_list.size();
	int position2 = rnd01() * X1.node_list.size();
	int positionmin = min(position1, position2);
	int positionmax = max(position1, position2);
	vector<ListDigraph::Node> new_nodes;
	int i;
	//int action = 10 * rnd01();

	/*if (action > 9) {
		X_new.node_list = X1.node_list;
		std::shuffle(std::begin(X_new.node_list), std::end(X_new.node_list), std::default_random_engine());
		return X_new;
	}*/

	for (i = positionmin; i < positionmax; i++) {
		new_nodes.push_back(X1.node_list[i]);
	}

	int j = 0;
	for (i = 0; i < positionmin; i++) {
		ListDigraph::Node node = X2.node_list[j++];
		while (find(new_nodes.begin(), new_nodes.end(), node) != new_nodes.end()) {
			node = X2.node_list[j++];
		}
		new_nodes.insert(new_nodes.begin() + i, node);
	}

	for (i = positionmax; i < (int)X1.node_list.size(); i++) {
		ListDigraph::Node node = X2.node_list[j++];
		while (find(new_nodes.begin(), new_nodes.end(), node) != new_nodes.end()) {
			node = X2.node_list[j++];
		}
		new_nodes.insert(new_nodes.begin() + i, node);
	}
	X_new.node_list = new_nodes;

	/*cout << "X1=";
	for (int i = 0; i < X1.node_list.size(); i++)
		cout << navigator->graph.id(X1.node_list[i]) << " ";
	cout << endl;
	cout << "X2=";
	for (int i = 0; i < X2.node_list.size(); i++)
		cout << navigator->graph.id(X2.node_list[i]) << " ";
	cout << endl;
		cout << "Xnew=";
	for (int i = 0; i < X_new.node_list.size(); i++)
		cout << navigator->graph.id(X_new.node_list[i]) << " ";
	cout << endl;

	cout << "positionmin = " << positionmin << "positionmax = " << positionmax << endl;*/
	return X_new;
}

double genetic_optimizer::calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X) {
	// finalize the cost
	double final_cost = 0.0;
	final_cost += X.middle_costs.objective1;
	return final_cost;
}

std::ofstream output_file;

void genetic_optimizer::SO_report_generation(
	int generation_number,
	const EA::GenerationType<MySolution, MyMiddleCost>& last_generation,
	const MySolution& best_genes) {
	cout << "Problem Id:" << instance << ", "
		 << "Generation [" << generation_number << "], " << setprecision(10)
		 << "Best=" << -last_generation.best_total_cost << ", "
		 << "Average=" << -last_generation.average_cost << ", "
		 << "genes size=" << best_genes.to_string(this).length() << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;


	output_file.open(filename,
					 std::ofstream::app);
	output_file << generation_number << "   \t" << std::setw(11)
				<< -last_generation.average_cost << "   \t" << std::setw(11)
				<< -last_generation.best_total_cost << "\t" << std::setw(20)
				<< best_genes.to_string(this) << "\n";
	output_file.flush();
	output_file.close();
}

genetic_optimizer::genetic_optimizer(int arg_instance) {
	instance = arg_instance;
	filename = "./results/" + to_string(instance) + ".txt";
	ostringstream instance_file;
	if (instance == -1) instance_file << "./part-1-examples/example-01.desc";
	else instance_file << "./part-1-initial/prob-"<< setw(3) << setfill('0') << instance << ".desc";

	EA::Chronometer timer;
	timer.tic();
	base_mine = new mine_state(instance_file.str());
	cout << "time to load mine " << timer.toc() << endl;
	timer.tic();
	navigator = new mine_navigator(base_mine);
	cout << "time to init graph " << timer.toc() << endl;
	timer.tic();
	navigator->init_ordered_map();
	cout << "time to populate map " << timer.toc() << endl;

	ifstream file_to_test(filename);
	if (!file_to_test.good()) {
		cout << "file do not exist" << endl;
		cout << "start from scratch" << endl;
		output_file.open("./results/" + to_string(instance) + ".txt");
		output_file << "step"
					<< "\t"
					<< "cost_avg"
					<< "\t"
					<< "cost_best"
					<< "\t"
					<< "solution_best"
					<< "\n";
		output_file.close();
	}

}
genetic_optimizer::~genetic_optimizer() {
	delete base_mine;
	delete navigator;
}

bool genetic_optimizer::solve(int population_size) {
	using namespace std::placeholders;

	EA::Chronometer timer;
	timer.tic();

	GA_Type ga_obj;
	ga_obj.problem_mode = EA::GA_MODE::SOGA;
	ga_obj.multi_threading = true;
	ga_obj.idle_delay_us = 1;  // switch between threads quickly
	ga_obj.dynamic_threading = false;
	ga_obj.verbose = false;
	ga_obj.population = population_size;
	ga_obj.generation_max = 5000;
	ga_obj.calculate_SO_total_fitness = std::bind(&genetic_optimizer::calculate_SO_total_fitness, this, _1);
	ga_obj.init_genes = std::bind(&genetic_optimizer::init_genes, this, _1, _2);
	ga_obj.eval_solution = std::bind(&genetic_optimizer::eval_solution, this, _1, _2);
	ga_obj.mutate = std::bind(&genetic_optimizer::mutate, this, _1, _2, _3);
	ga_obj.crossover = std::bind(&genetic_optimizer::crossover, this, _1, _2, _3);
	ga_obj.SO_report_generation = std::bind(&genetic_optimizer::SO_report_generation, this, _1, _2, _3);
	ga_obj.crossover_fraction = 0.7;
	ga_obj.mutation_rate = 0.3;
	ga_obj.best_stall_max = 200;
	ga_obj.average_stall_max = 50;
	ga_obj.elite_count = 300;
	EA::StopReason reason = ga_obj.solve();
	cout << "The problem is optimized in " << timer.toc()
			<< " seconds." << endl;
	cout << "cause: " << ga_obj.stop_reason_to_string(reason)
			<< endl;
	cout << "m= " << ga_obj.mutation_rate << ", c= " << ga_obj.crossover_fraction << ", e= " << ga_obj.elite_count << endl;

	return true;
}


static void print_help() {
	printf(
		"options: \n"
		"	-h : this help \n"
		"	-i instance: instance of the problem to display \n"
		"	-l : load the best solution so far for this problem \n"
		"	-a from: do all problem from \"from\"\n"
		"	-n number: number of thrust to optimize (default 1)\n"
		"	-f factor: divider of fuel max to limit thrust range (default is "
		"2)\n"
		"	-p population: population of each generation (default 2000)\n"
		"	-d : enable logging of chromosomes in a file\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int start_instance = 0;
	int c;
	int population = 2000;
	int gInstance = 0;

	while ((c = getopt(argc, argv, "p:a:hi:")) != -1) switch (c) {
			case 'i':
				gInstance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				start_instance = atoi(optarg);
				break;
			case 'p':
				population = atoi(optarg);
				break;
			case 'h':
			default:
				print_help();
				exit(0);
		}

	for (int i = start_instance; i < 150; i++) {
		if (do_all) {
			gInstance = i;
		}

		genetic_optimizer optimizer(gInstance);

		bool solved = optimizer.solve(population);
		if (solved) break;

		if (!do_all) {
			return 0;
		}
	}
	return 0;
}