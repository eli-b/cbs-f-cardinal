#pragma once

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/pool/pool.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
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
using std::string;

// #define NDEBUG 

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
	int location = -1;
	// bool single = false;
	int mdd_width;  // TODO:: Myabe this can be deleted as we always build/look for MDDs when we classify conflicts

	bool is_single() const
	{
		return mdd_width == 1;
	}
	// PathEntry(int loc = -1) { location = loc; single = false; }
};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
/*struct three_tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
        auto h1 = std::hash<T1>{}(get<0>(p));
        auto h2 = std::hash<T2>{}(get<1>(p));
        auto h3 = std::hash<T3>{}(get<2>(p));
        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2 ^ h3;
    }
};*/

struct DeleterFromPool
{
	explicit DeleterFromPool(boost::pool<>& pool) : pool(&pool) {}
	DeleterFromPool(const DeleterFromPool& other) : pool(other.pool) {}
	DeleterFromPool(DeleterFromPool&& other)  noexcept : pool(other.pool) {}

	void operator()(void *p)
	{
		pool->free(p);
	}
	boost::pool<> *pool;
};

struct ConstraintState
{
	bool vertex = false;
	bool edge[5] = { false, false, false, false, false };

	ConstraintState() = default;
	ConstraintState(const ConstraintState& other) : vertex(other.vertex) {
		edge[0] = other.edge[0];
		edge[1] = other.edge[1];
		edge[2] = other.edge[2];
		edge[3] = other.edge[3];
		edge[4] = other.edge[4];
	}
};

extern boost::pool<> constraint_state_pool;
extern DeleterFromPool constraintStatePoolDeleter;
