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
#include <lemon/smart_graph.h>
#include <unistd.h>
#include <lemon/adaptors.h>
#include <lemon/maps.h>

using namespace std;
using namespace lemon;

//typedef std::complex<double> Complex;
typedef SmartDigraph::Node Node;
typedef SmartDigraph::Arc Arc;
typedef SmartDigraph::NodeIt NodeIt;
typedef SmartDigraph::ArcIt ArcIt;
typedef SmartDigraph Graph;
typedef FilterNodes<SmartDigraph> lSubGraph;
typedef complex<int> position;

#endif
