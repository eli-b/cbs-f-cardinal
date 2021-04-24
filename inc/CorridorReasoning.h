#pragma once

#include "Instance.h"
#include "SingleAgentSolver.h"


class CorridorReasoning
{
public:
	bool use_corridor_reasoning;
	double accumulated_runtime = 0;
	enum calc_alt_cost_variant {NO, YES};
	calc_alt_cost_variant calc_alt_cost;

	CorridorReasoning(const vector<SingleAgentSolver*>& search_engines,
					  const vector<ConstraintTable>& initial_constraints) :
			search_engines(search_engines), initial_constraints(initial_constraints) {}

	shared_ptr<Conflict> run(const shared_ptr<Conflict>& conflict,
							 const vector<Path*>& paths,
							 bool cardinal, const CBSNode& node);

private:
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;  // The problem instance's initial constraints

	shared_ptr<Conflict> findCorridorConflict(const shared_ptr<Conflict>& conflict,
											  const vector<Path*>& paths,
											  bool cardinal, const CBSNode& node);
	int findCorridor(const shared_ptr<Conflict>& conflict,
					 const vector<Path*>& paths, int endpoints[], int endpoints_time[]); // return the length of the corridor
	int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t);
	int getExitingTime(const std::vector<PathEntry>& path, int t);
	int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);


	bool blocked(const Path& path, const Constraint& constraint);

};


