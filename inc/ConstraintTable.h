#pragma once
#include "common.h"
#include "ICBSNode.h"

class ConstraintTable
{
public:
	int length_min = 0;
	int length_max = INT_MAX;
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.
	int num_col;
	int map_size;

	int getHoldingTime(); // the earliest timestep that the agent can hold its goal location

	void clear(){ct.clear(); cat.clear(); landmarks.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}

	bool constrained(int loc, int t) const;
    bool constrained(int curr_loc, int next_loc, int next_t) const;
	int getNumOfConflictsForStep(int curr_id, int next_id, int next_timestep) const;


	ConstraintTable() = default;
	ConstraintTable(int num_col, int map_size, int goal_location = -1) : goal_location(goal_location), num_col(num_col), map_size(map_size) {}
    // ConstraintTable(int goal_location): goal_location(goal_location) {}
	ConstraintTable(const ConstraintTable& other) {copy(other); }

	
	void build(const ICBSNode& node, int agent); // build the constraint table for the given agent at the give node 
	void buildCAT(int agent, const vector<Path*>& paths); // build the conflict avoidance table

protected:
	unordered_map<size_t, list<pair<int, int> > > ct; // csontraint table: location -> time range, or edge -> time range
	unordered_map< size_t, list<pair<int, int> > > cat; // conflict avoidance table cat:  location -> time range, or edge -> time range
	unordered_map<size_t, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep

	void insert2CT(int loc, int t_min, int t_max); // insert a vertex constraint to the constraint table
	void insert2CT(int from, int to, int t_min, int t_max); // insert an edge constraint to the constraint table
	void insertLandmark(int loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep
	void copy(const ConstraintTable& other);

	inline int getEdgeIndex(int from,int to) const {return (1 + from) * map_size + to;  }
};

