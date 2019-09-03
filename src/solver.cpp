#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include "agent.h"
#include "functional"
#include "sys/time.h"

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

	int instance = 0;

	while ((c = getopt(argc, argv, "vdf:p:n:a:hci:")) != -1) switch (c) {
			case 'i':
				instance = atoi(optarg);
				break;
			case 'a':
				do_all = true;
				start_instance = atoi(optarg);
				break;
			case 'h':
			default:
				print_help();
				exit(0);
		}


	return 0;
}