#ifndef COMMON_H
#define COMMON_H

#include <functional>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <string>
#include <sys/stat.h>
#include <math.h>
#include <sstream>
#include <vector>
#include <complex>
#include <lemon/list_graph.h>
#include <unistd.h>
#include <lemon/adaptors.h>
#include <lemon/maps.h>

using namespace std;
using namespace lemon;

typedef std::complex<double> Complex;
typedef ListDigraph::Node Node;
typedef ListDigraph::Arc Arc;
typedef ListDigraph::NodeIt NodeIt;
typedef ListDigraph::ArcIt ArcIt;
typedef ListDigraph Graph;
typedef FilterNodes<ListDigraph> lSubGraph;


#endif
