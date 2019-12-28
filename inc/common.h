#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::get;
using std::tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;

//#include <boost/graph/adjacency_list.hpp>
//typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
//typedef confilctGraph_t::vertex_descriptor vertex_t;
//typedef confilctGraph_t::edge_descriptor edge_t;

enum split_strategy { NON_DISJOINT, RANDOM, SINGLETONS, WIDTH, DISJOINT3, SPLIT_COUNT };
enum heuristics_type { NONE, CG, DG, WDG, STRATEGY_COUNT };

typedef vector< unordered_set<int64_t> > CAT; // conflict avoidance table

struct PathEntry
{
	int location;
	bool single;
	PathEntry(int loc = -1) { location = loc; single = false; }
};

typedef std::vector<PathEntry> Path;

bool validMove(int curr, int next, int num_col, int map_size);

bool isSamePath(const Path& p1, const Path& p2);

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
struct three_tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
        auto h1 = std::hash<T1>{}(get<0>(p));
        auto h2 = std::hash<T2>{}(get<1>(p));
        auto h3 = std::hash<T3>{}(get<2>(p));
        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2 ^ h3;
    }
};
