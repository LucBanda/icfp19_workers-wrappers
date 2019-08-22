#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "openga.hpp"
#include "genetic_optimizer.h"
#include "functional"

using std::cout;
using std::endl;
using std::string;

void genetic_optimizer::init_genes(MySolution& p, const std::function<double(void)>& rnd01) {
	int size = base_mine->non_validated_tiles.size() * 5;
	mine_state test_mine(base_mine);

	string gene = "";
	for (int i = 0; i < size; i++) {
		vector<string> valid_possibilities = test_mine.get_next_valid_command();
		int poss = (valid_possibilities.size()) * rnd01();
		gene += valid_possibilities[poss];
		test_mine.apply_command(string(1, gene.back()));
	}
	p.execution = gene;
}

bool genetic_optimizer::eval_solution(const MySolution& p, MyMiddleCost& c) {
	mine_state mine(base_mine);
	agent executeur(&mine);

	executeur.set_execution_map(p.execution);
	executeur.run();
	c.objective1 = executeur.get_cost();

	return true;  // solution is accepted
}

MySolution genetic_optimizer::mutate(const MySolution& X_base,
				  const std::function<double(void)>& rnd01,
				  double shrink_scale) {
	MySolution X_new;
	X_new.execution = X_base.execution;
	for (int j = 0; j < base_mine->non_validated_tiles.size() / 100 * shrink_scale; j++) {
		int action = floor(3. * rnd01());
		if (action == 0) {
			//replace
			int pos = floor((X_base.execution.length()) * rnd01());
			int poss = floor((sizeof(possibilities)) * rnd01());
			X_new.execution[pos] = possibilities[poss];
		} else if (action == 1) {
			//insert
			int pos = floor((X_base.execution.length()) * rnd01());
			int poss = floor((sizeof(possibilities)) * rnd01());
			X_new.execution = X_base.execution;
			X_new.execution.insert(pos, to_string(possibilities[poss]));
		} else  {
			//remove
			int pos = floor((X_base.execution.length()) * rnd01());
			X_new.execution = X_base.execution;
			X_new.execution.erase(pos);
		}
	}
	/*mine_state tester(base_mine);
	X_new.execution = tester.strip(X_new.execution);
	*/return X_new;
}

MySolution genetic_optimizer::crossover(const MySolution& X1, const MySolution& X2,
					 const std::function<double(void)>& rnd01) {
	MySolution X_new;
	double proportion1 = rnd01() / 3;
	double proportion2 = proportion1 + rnd01() / 3;
	double proportion3 = proportion2 + rnd01() / 3;
	//int action = floor(2. * rnd01());

	if (1) {
		int keep1 = rnd01() * 2;
		int keep2 = rnd01() * 2;
		if (keep1)
			X_new.execution += X1.execution.substr(0, X1.execution.length() * proportion1)
							+ X2.execution.substr(X2.execution.length() * proportion1, X2.execution.length() * proportion2);
		if (keep2)
			X_new.execution += X1.execution.substr(X1.execution.length() * proportion2, X1.execution.length() * proportion3)
							+ X2.execution.substr(X2.execution.length() * proportion3, X2.execution.length());
	} else {
		for (auto it1 = X1.execution.begin(), it2 = X2.execution.begin(); it1 != X1.execution.end() && it2!= X2.execution.end(); ++it1, ++it2) {
			if ((*it1 == 'W') && (*it2 == 'S'))
				continue;
			if ((*it1 == 'S') && (*it2 == 'W'))
				continue;
			if ((*it1 == 'D') && (*it2 == 'A'))
				continue;
			if ((*it1 == 'A') && (*it2 == 'D'))
				continue;
			int whichone = floor(2 * rnd01());
			if (whichone)
				X_new.execution += *it1;
			else
				X_new.execution += *it2;
		}

	}

	/*mine_state tester(base_mine);
	X_new.execution = tester.strip(X_new.execution);*/
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
		 << "Best genes size=" << best_genes.execution.size()
		 << ", "
		 << "Exe_time=" << last_generation.exe_time << endl;


	output_file.open(filename,
					 std::ofstream::app);
	output_file << generation_number << "   \t" << std::setw(11)
				<< -last_generation.average_cost << "   \t" << std::setw(11)
				<< -last_generation.best_total_cost << "\t" << std::setw(20)
				<< best_genes.to_string() << "\n";
	output_file.flush();
	output_file.close();
}

genetic_optimizer::genetic_optimizer(int arg_instance) {
	instance = arg_instance;
	filename = "./results/" + to_string(instance) + ".txt";
	ostringstream instance_file;
	if (instance == -1) instance_file << "./part-1-examples/example-01.desc";
	else instance_file << "./part-1-initial/prob-"<< setw(3) << setfill('0') << instance << ".desc";

	base_mine = new mine_state(instance_file.str());

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
	ga_obj.best_stall_max = 50;
	ga_obj.average_stall_max = 50;
	ga_obj.elite_count = 10;
	EA::StopReason reason = ga_obj.solve();
	cout << "The problem is optimized in " << timer.toc()
			<< " seconds." << endl;
	cout << "cause: " << ga_obj.stop_reason_to_string(reason)
			<< endl;

	return true;
}


static void print_help() {
	printf(
		"options: \n"
		"	-h : this help \n"
		"	-i instance: instance of the problem to display \n"
		"	-l : load the best solution so far for this problem \n"
		"	-a : do all problem\n"
		"	-n number: number of thrust to optimize (default 1)\n"
		"	-f factor: divider of fuel max to limit thrust range (default is "
		"2)\n"
		"	-p population: population of each generation (default 2000)\n"
		"	-d : enable logging of chromosomes in a file\n");
}

int main(int argc, char** argv) {
	bool do_all = false;
	int c;
	bool continue_after_stall = false;
	int population = 2000;
	int fuel_factor = 2.;
	int nb_of_thrusts = 1;
	int gInstance = 0;
	bool verbose_chromosomes = false;

	while ((c = getopt(argc, argv, "vdf:p:n:ahci:")) != -1) switch (c) {
			case 'i':
				gInstance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				break;
			case 'c':
				continue_after_stall = true;
				break;
			case 'n':
				nb_of_thrusts = atoi(optarg);
				break;
			case 'f':
				fuel_factor = atoi(optarg);
				break;
			case 'p':
				population = atoi(optarg);
				break;
			case 'v':
				verbose_chromosomes = true;
				break;
			case 'h':
			default:
				print_help();
				exit(0);
		}

	for (int i = 1; i < 5; i++) {
		for (int j = 1; j < 5; j++) {
			if (do_all) {
				gInstance = i * 1000 + j;
			}

			do {
				genetic_optimizer optimizer(gInstance);

				bool solved = optimizer.solve(population);
				if (solved) break;

			} while (continue_after_stall);

			if (!do_all) {
				return 0;
			}
		}
	}
	return 0;
}