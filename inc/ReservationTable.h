// This is used by SIPP
#pragma once
#include "ConstraintTable.h"

typedef tuple<size_t, size_t, size_t> Interval; // [t_min, t_max), num_of_collisions


class ReservationTable: public ConstraintTable
{
public:
    double runtime;

	ReservationTable() = default;
	ReservationTable(int num_col, int map_size, int goal_location = -1): ConstraintTable(num_col, map_size, goal_location) {}
	ReservationTable(const ConstraintTable& other) {copy(other); }


    list<Interval> get_safe_intervals(int location, size_t lower_bound, size_t upper_bound);
	list<Interval> get_safe_intervals(int from, int to, size_t lower_bound, size_t upper_bound);

	// int get_holding_time(int location);
   Interval get_first_safe_interval(int location);
    bool find_safe_interval(Interval& interval, int location, int t_min);

	void buildCAT(int agent, const vector<Path*>& paths); // build the conflict avoidance table

    void print() const;

private:
	unordered_map<size_t, list<Interval > > rt; // location -> [t_min, t_max), num_of_collisions

	unordered_map< size_t, list<pair<int, int> > > cat; // conflict avoidance table cat:  location -> time range, or edge -> time range

    void insert2RT(int location, size_t t_min, size_t t_max);
    void insertSoftConstraint2RT(int location, size_t t_min, size_t t_max);
	void mergeIntervals(list<Interval >& intervals);

	void updateRT(int location); // update RT at the gvien location

	int getNumOfConflictsForStep(int curr_id, int next_id, int next_timestep) const;
};