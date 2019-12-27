#pragma once
#include "HLHeuristic.h"
#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"
#include <ctime>
#include "HTable.h"
#include "MDD.h"
#include "Conflict.h"

class ICBSSearch
{
public:
    bool rectangle_reasoning; // using rectangle reasoning
    bool corridor_reasoning; // using corridor reasoning
    bool target_reasoning; // using target reasoning

	double runtime = 0;
	double runtime_build_dependency_graph = 0;
	double runtime_solve_MVC = 0;
    double runtime_detect_conflicts = 0;
	double runtime_classify_conflicts = 0;

    uint64_t num_corridor = 0;
    uint64_t num_rectangle = 0;
    uint64_t num_target = 0;
    uint64_t num_standard = 0;

    uint64_t num_merge_MDDs = 0;
    uint64_t num_solve_2agent_problems = 0;

    uint64_t HL_num_expanded = 0;
    uint64_t HL_num_generated = 0;
    uint64_t LL_num_expanded = 0;
    uint64_t LL_num_generated = 0;


    int max_num_of_mdds = 10000;

	ICBSNode* dummy_start;
	ICBSNode* goal_node = nullptr;



	bool solution_found = false;
	int solution_cost = -2;;
	double min_f_val;
	double focal_list_threshold;

	// Runs the algorithm until the problem is solved or time is exhausted 
	bool runICBSSearch();

	ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, 
		heuristics_type h_type, bool PC,
		double time_limit, int screen);
	ICBSSearch(const MapLoader* ml, vector<SingleAgentICBS*>& search_engines,
	        const vector<ConstraintTable>& constraints,
		vector<vector<PathEntry>>& paths_found_initially, double f_w, int initial_h, 
		heuristics_type h_type, bool PC, int cost_upperbound, double time_limit, int screen);
	void clearSearchEngines();
	~ICBSSearch();

	// Save results
	void saveResults(const std::string &fileName, const std::string &instanceName) const;
	// void saveLogs(const std::string &fileName) const;

private:

	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	std::vector<MDDTable> mddTable;
	// int num_released_mdds = 0;
	std::vector<std::vector<HTable>> hTable;
	// int bookingHitTimes = 0;
	// double bookingSearchtime = 0;

	bool PC; // prioritize conflicts or not

    string getSolverName() const;

	int screen;
	heuristics_type h_type;
	const double time_limit;
	double focal_w = 1.0;
	const int cost_upperbound = INT_MAX;
	

	// Logs
	vector<int> sum_h_vals; // sum of heuristics for the CT nodes at level t
	vector<int> sum_f_vals; // sum of f values for the CT nodes at level t
	vector<int> num_CTnodes; // number of CT nodes at level t that has heuristics
	vector<int> sum_runtime; // sum of runtime for computing heuristics for the CT nodes at level t
	list<pair<int, int>> log_min_f; // changes of lowerbound in terms of expanded nodes: <lowerbound, #expanded nodes>


	vector<ConstraintTable> initial_constraints;
	const MapLoader* ml;
	std::clock_t start;

	int num_of_agents;


	vector<Path*> paths;
	vector<Path> paths_found_initially;  // contain initial paths found
	vector<MDD*> mdds_initially;  // contain initial paths found
	vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths and mdd


	// high level search
	bool findPathForSingleAgent(ICBSNode*  node, int ag, int lower_bound = 0);
	bool generateChild(ICBSNode* child, ICBSNode* curr);

	//conflicts
	void findConflicts(ICBSNode& curr);
	void findConflicts(ICBSNode& curr, int a1, int a2);
    std::shared_ptr<Conflict> chooseConflict(const ICBSNode &node) const;
	void classifyConflicts(ICBSNode &parent);
	void copyConflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
		std::list<std::shared_ptr<Conflict>>& copy, int excluded_agent) const;
	void copyConflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
		std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agent) const;
	void removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const;
    //bool isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node);

	// add heuristics for the high-level search
	int computeHeuristics(ICBSNode& curr);
	bool buildDependenceGraph(ICBSNode& node);
	int getEdgeWeight(int a1, int a2, ICBSNode& node, bool cardinal, bool& hit);

	// build MDD
	MDD * getMDD(ICBSNode& curr, int id);
	void releaseMDDMemory(int id);

	//update information
	// int collectConstraints(ICBSNode* curr, int agent_id, std::vector <std::list< std::pair<int, int> > >& cons_vec); // return the minimal length of the path
	inline void updatePaths(ICBSNode* curr);
	void updateFocalList();
	void updateReservationTable(CAT& res_table, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();
	void copyConflictGraph(ICBSNode& child, const ICBSNode& parent);

	// print and save
	void printPaths() const;
	void printStrategy() const;
	void printResults() const;
	void printConflicts(const ICBSNode &curr) const;
	
	bool validateSolution() const;
};

