#pragma once
#include "CBSHeuristic.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "MutexReasoning.h"

class CBS
{
public:
	bool rectangle_reasoning; // using rectangle reasoning
	bool corridor_reasoning; // using corridor reasoning
	bool target_reasoning; // using target reasoning
	bool disjoint_splitting; // disjoint splittting
  bool mutex_reasoning; // using mutex reasoning
	bool bypass; // using Bypass1

	double runtime = 0;
	double runtime_generate_child = 0; // runtimr of generating child nodes
	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	double runtime_path_finding = 0; // runtime of finding paths for single agents
	double runtime_detect_conflicts = 0;
	double runtime_classify_conflicts = 0;
	double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

	uint64_t num_corridor = 0;
	uint64_t num_rectangle = 0;
	uint64_t num_target = 0;
	uint64_t num_standard = 0;

	uint64_t num_adopt_bypass = 0; // number of times when adopting bypasses

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;


	CBSNode* dummy_start = nullptr;
	CBSNode* goal_node = nullptr;



	bool solution_found = false;
	int solution_cost = -2;
	double min_f_val;
	double focal_list_threshold;

	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(double time_limit, int initial_h);

	CBS(const Instance& instance, double f_w,
		heuristics_type h_type, bool PC, bool sipp, int screen);
	CBS(vector<SingleAgentSolver*>& search_engines,
		const vector<ConstraintTable>& constraints,
		vector<Path>& paths_found_initially, double f_w,
		heuristics_type h_type, bool PC, int cost_upperbound, int screen);
	void clearSearchEngines();
	~CBS();

	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;
	// void saveLogs(const std::string &fileName) const;

	void clear(); // used for rapid random  restart

private:
	MDDTable mdd_helper;
	RectangleReasoning rectangle_helper;
	CorridorReasoning corridor_helper;
  MutexReasoning mutex_helper;
	CBSHeuristic heuristic_helper;



	pairing_heap< CBSNode*, compare<CBSNode::compare_node> > open_list;
	pairing_heap< CBSNode*, compare<CBSNode::secondary_compare_node> > focal_list;
	list<CBSNode*> allNodes_table;

	// vector<MDDTable> mddTable;


	bool PC; // prioritize conflicts or not

	string getSolverName() const;

	int screen;
	
	double time_limit;
	double focal_w = 1.0;
	const int cost_upperbound = INT_MAX;


	vector<ConstraintTable> initial_constraints;
	clock_t start;

	int num_of_agents;


	vector<Path*> paths;
	vector<Path> paths_found_initially;  // contain initial paths found
	// vector<MDD*> mdds_initially;  // contain initial paths found
	vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' paths and mdd


	// high level search
	bool findPathForSingleAgent(CBSNode*  node, int ag, int lower_bound = 0);
	bool generateChild(CBSNode* child, CBSNode* curr);
	bool generateRoot(int initial_h);

	//conflicts
	void findConflicts(CBSNode& curr);
	void findConflicts(CBSNode& curr, int a1, int a2);
	shared_ptr<Conflict> chooseConflict(const CBSNode &node) const;
	void classifyConflicts(CBSNode &parent);
	// void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
	// 	list<shared_ptr<Conflict>>& copy, int excluded_agent) const;
	void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
		list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agent) const;
	void removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const;
	//bool isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node);

	// add heuristics for the high-level search
	//int computeHeuristics(CBSNode& curr);
	//bool buildDependenceGraph(CBSNode& node);
	//int getEdgeWeight(int a1, int a2, CBSNode& node, bool cardinal);



	//update information
	inline void updatePaths(CBSNode* curr);
	void updateFocalList();
	inline void releaseNodes();
	//inline void releaseMDDTable();
	// void copyConflictGraph(CBSNode& child, const CBSNode& parent);

	// print and save
	void printPaths() const;
	void printResults() const;
	void printConflicts(const CBSNode &curr) const;

	bool validateSolution() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;
	inline void pushNode(CBSNode* node);
};
