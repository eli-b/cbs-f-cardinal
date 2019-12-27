#pragma once
#include <tuple>
#include <vector>
#include <list>

#include "LLNode.h"
#include "ICBSNode.h"
#include "MDD.h"


std::shared_ptr<Conflict> findRectangleConflict(const vector<Path*>& paths, int timestep, int num_col, int map_size,
                                                int a1, int a2, int loc1, const MDD* mdd1, const MDD* mdd2);


//Identify rectangle conflicts
bool isRectangleConflict(const std::pair<int,int>& s1, const std::pair<int, int>& s2, 
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t);// for CR and R
bool isRectangleConflict(int s1, int s2, int g1, int g2, int num_col);// for RM

//Classify rectangle conflicts
int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2);// for CR and R
int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, int num_col);// for RM

//Compute rectangle corners
std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2);
std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1);

//Compute start and goal candidates for RM
std::list<int> getStartCandidates(const std::vector<PathEntry>& path, int timestep, int num_col);
std::list<int> getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int num_col);

// whether the path between loc1 and loc2 is Manhattan-optimal
bool isManhattanOptimal(int loc1, int loc2, int dt, int num_col);

// int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col);
