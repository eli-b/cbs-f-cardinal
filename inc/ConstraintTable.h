#pragma once

#include "common.h"
#include "CBSNode.h"
#include "XytHolder.h"


class ConstraintTable
{
public:
	int length_min = 0;  // Path must be at least this long
	int length_max = MAX_TIMESTEP;  // Path must be at most this long
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.
	size_t num_col;
	size_t map_size;
	int cat_size = 0;

	//enum moves { NORTH, EAST, SOUTH, WEST, WAIT, MOVES_COUNT };

	int getHoldingTime(); // the earliest timestep that the agent can hold its goal location

	// void clear(){ct.clear(); cat_small.clear(); cat_large.clear(); landmarks.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}

	bool constrained(size_t loc, int t) const;
	bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;
	//int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;
	ConstraintTable() = delete;
	ConstraintTable(size_t num_col, size_t map_size, int goal_location = -1)
	 : goal_location(goal_location), num_col(num_col), map_size(map_size),
	   ct(map_size, constraintStatePoolDeleter) {}
	ConstraintTable(const ConstraintTable& other) : ct(other.map_size, constraintStatePoolDeleter) { copy(other); }

	void copy(const ConstraintTable& other);
	void build(const CBSNode& node, int agent); // build the constraint table for the given agent at the given node
	//void buildCAT(int agent, const vector<Path*>& paths, size_t cat_size); // build the conflict avoidance table

	void insert(size_t loc, int t_min, int t_max); // insert a vertex constraint to the constraint table
	void insert(size_t from, size_t to, int t_min, int t_max); // insert an edge constraint to the constraint table

	size_t getNumOfLandmarks() const { return landmarks.size(); }
	unordered_map<size_t, size_t> getLandmarks() const { return landmarks; }
	list<pair<int, int>> decodeBarrier(int B1, int B2, int t);
protected:
	// Constraint Table (CT)
	// The cost of this approach is giving up locality - nearby locations would have different hashes and edges and their
	// ends won't be close together. The benefit is space savings - most ConstraintState and AvoidanceState are sparse
	// in true values.
	// Mapping x,y coordinates to location IDs could have the same issue, if there were maps that were wider than a page.
	// For now, having width*height vectors is fine (~100,000*8 bytes is OK), but squaring that for edges is not.
	unordered_map<size_t, list<pair<int, int>>> ct_ranges; // location -> time range, or edge -> time range
	XytHolder<ConstraintState, DeleterFromPool> ct;

	unordered_map<size_t, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep

	void insertLandmark(size_t loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep

	// Guaranteed not to clash with any location or any other edge
	inline size_t getEdgeIndex(size_t from, size_t to) const { return (1 + from) * map_size + to; }
	inline std::pair<size_t, size_t> getEdgeLocations(size_t edge_index) const {
		return make_pair(edge_index / map_size - 1, edge_index % map_size);
	}

private:
	//size_t map_size_threshold = 10000;
	//vector<list<size_t>> cat_large; // conflict avoidance table for large maps
	//vector<vector<bool>> cat_small; // conflict avoidance table for small maps

	size_t move_offsets[4];
};

