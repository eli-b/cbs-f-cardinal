#pragma once
#include "MDD.h"

class RectangleReasoning
{
public:
	RectangleReasoning(const Instance& instance) : instance(instance) {}

	std::shared_ptr<Conflict> findRectangleConflict(const vector<Path*>& paths, int timestep, 
		int a1, int a2, int loc1, const MDD* mdd1, const MDD* mdd2);


private:
	const Instance& instance;

	//Identify rectangle conflicts
	bool isRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
		const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t);// for CR and R
	bool isRectangleConflict(int s1, int s2, int g1, int g2);// for RM

																		  //Classify rectangle conflicts
	int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
		const std::pair<int, int>& g1, const std::pair<int, int>& g2);// for CR and R
	int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg);// for RM

																											  //Compute rectangle corners
	std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2);
	std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1);

	//Compute start and goal candidates for RM
	list<int> getStartCandidates(const std::vector<PathEntry>& path, int timestep);
	list<int> getGoalCandidates(const std::vector<PathEntry>& path, int timestep);


	// int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col);

	bool addModifiedBarrierConstraints(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
		const MDD* mdd1, const MDD* mdd2,
		list<Constraint>& constraint1, list<Constraint>& constraint2);

	// add a horizontal modified barrier constraint
	bool addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
		int Ri_y, int Rg_y, int Rg_t, list<Constraint>& constraints);

	// add a vertival modified barrier constraint
	bool addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
		int Ri_x, int Rg_x, int Rg_t, list<Constraint>& constraints);

	bool blocked(const Path& path, const list<Constraint>& constraints);
	bool traverse(const Path& path, int loc, int t);

};

