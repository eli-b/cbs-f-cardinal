#pragma once
#include "MDD.h"

class RectangleReasoning
{
public:
	RectangleReasoning(const Instance& instance) : instance(instance) {}

	std::shared_ptr<Conflict> findRectangleConflict(const vector<Path*>& paths, int timestep, 
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);


private:
	const Instance& instance;

	std::shared_ptr<Conflict> findRectangleConflictByRM(const vector<Path*>& paths, int timestep,
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);
	std::shared_ptr<Conflict> findRectangleConflictByGR(const vector<Path*>& paths, int timestep,
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);

	bool ExtractBarriers(const MDD& mdd, int loc, int timestep, int dir, int dir2, int start, int goal, int start_time, std::list<Constraint>& B);
	bool isEntryBarrier(const Constraint& b1, const Constraint& b2, int dir1);
	bool isExitBarrier(const Constraint& b1, const Constraint& b2, int dir1);
	pair<int, int> getIntersection(const Constraint& b1, const Constraint& b2);
	bool blockedNodes(const std::vector<PathEntry>& path,
		const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int Rg_t, int dir);
	bool isCut(const Constraint b, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg);

	void generalizedRectangle(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2, const MDD& mdd1, const MDD& mdd2,
		const std::list<Constraint>& B1, const std::list<Constraint>& B2, int timestep,
		int& best_type, std::pair<int, int>& best_Rs, std::pair<int, int>& best_Rg);

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
	list<int> getStartCandidates(const Path& path, int timestep);
	list<int> getGoalCandidates(const Path& path, int timestep);
	//Compute start and goal candidates for GR
	int getStartCandidate(const Path& path, int dir1, int dir2, int timestep);
	int getGoalCandidate(const Path& path, int dir1, int dir2, int timestep);


	// int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col);

	bool addModifiedBarrierConstraints(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
		const MDD* mdd1, const MDD* mdd2,
		list<Constraint>& constraint1, list<Constraint>& constraint2); // for RM

	// add a horizontal modified barrier constraint
	bool addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
		int Ri_y, int Rg_y, int Rg_t, list<Constraint>& constraints);

	// add a vertival modified barrier constraint
	bool addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
		int Ri_x, int Rg_x, int Rg_t, list<Constraint>& constraints);

	bool blocked(const Path& path, const list<Constraint>& constraints);
	bool traverse(const Path& path, int loc, int t);

};

