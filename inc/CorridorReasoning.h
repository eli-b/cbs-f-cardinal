#pragma once
#include "ReservationTable.h"
#include "Instance.h"

enum corridor_strategy { NC, C, DISJOINTC };

class CorridorReasoning
{
public:
	corridor_strategy strategy;
	CorridorReasoning(const Instance& instance, const vector<ConstraintTable>& initial_constraints, bool usingSIPP):
		instance(instance), initial_constraints(initial_constraints), usingSIPP(usingSIPP) {}
	
	shared_ptr<Conflict> findCorridorConflict(const shared_ptr<Conflict>& conflict,
		const vector<Path*>& paths,
		bool cardinal, const CBSNode& node);

private:
	const Instance& instance;
	const vector<ConstraintTable>& initial_constraints;
	bool usingSIPP;

	int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t);
	int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);


	// int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size);
	int getBypassLengthByAStar(int start, int end, std::pair<int, int> blocked,
		const ConstraintTable& constraint_table, int upper_bound);
	int getBypassLengthBySIPP(int start, int end, std::pair<int, int> blocked,
		ReservationTable& reservation_table, int upper_bound);

	bool blocked(const Path& path, const Constraint& constraint);

};


