#pragma once
#include "common.h"

class ConstraintTable
{
public:
	int length_min = 0;
	int length_max = INT_MAX;
	int goal_location{};
	int latest_timestep = 0; // No negative constraints after this timestep.

	void clear(){CT.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}
	void insert(int loc, int t_min, int t_max);
    void insert(int from, int to, int t_min, int t_max, int map_size);
	bool is_constrained(int loc, int t) const;
    bool is_constrained(int curr_loc, int next_loc, int next_t,  int map_size) const;

    void copy(const ConstraintTable& other);

	unordered_map<size_t, list<pair<int, int> > > CT;

	ConstraintTable()= default;
    ConstraintTable(int goal_location): goal_location(goal_location) {}
	ConstraintTable(const ConstraintTable& other) {copy(other); }
	
};

