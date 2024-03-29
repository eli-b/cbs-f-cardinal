#pragma once

#include "CBSHeuristic.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "MutexReasoning.h"
#include "ConflictAvoidanceTable.h"

class CBS
{
public:
  bool randomRoot = false; // randomize the order of the agents in the root CT node

  /////////////////////////////////////////////////////////////////////////////////////
  // stats
  double runtime = 0;
  double runtime_generate_child = 0; // runtime of generating child nodes
  double runtime_build_CT = 0; // runtime of building constraint table
  double runtime_build_CAT = 0; // runtime of building conflict avoidance table
  double runtime_path_finding = 0; // runtime of finding paths for single agents
  double runtime_detect_conflicts = 0;
  double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

  uint64_t num_corridor_conflicts = 0;
  uint64_t num_rectangle_conflicts = 0;
  uint64_t num_target_conflicts = 0;
  uint64_t num_mutex_conflicts = 0;
  uint64_t num_standard_conflicts = 0;

  uint64_t num_adopt_bypass = 0; // number of times when adopting bypasses

  uint64_t num_HL_expanded = 0;
  uint64_t num_HL_generated = 0;
  uint64_t num_LL_expanded = 0;
  uint64_t num_LL_generated = 0;

  uint64_t f_cardinal_conflicts_found = 0;
  uint64_t cardinal_target_conflicts_found = 0;
  uint64_t semi_f_cardinal_conflicts_found = 0;
  uint64_t cardinal_conflicts_found = 0;
  uint64_t semi_cardinal_target_conflicts_found = 0;
  uint64_t semi_cardinal_conflicts_found = 0;
  uint64_t non_cardinal_conflicts_found = 0;

  uint64_t num_delta_h_0 = 0;
  uint64_t num_delta_h_minus_1 = 0;
  uint64_t num_delta_h_1 = 0;
  uint64_t sum_delta_h_minus_2_or_more = 0;
  uint64_t num_delta_h_minus_2_or_more = 0;
  uint64_t sum_delta_h_2_or_more = 0;
  uint64_t num_delta_h_2_or_more = 0;

  uint64_t num_delta_g_0 = 0;
  uint64_t num_delta_g_1 = 0;
  uint64_t sum_delta_g_2_or_more = 0;
  uint64_t num_delta_g_2_or_more = 0;

  uint64_t num_delta_f_0 = 0;
  uint64_t num_delta_f_1 = 0;
  uint64_t sum_delta_f_2_or_more = 0;
  uint64_t num_delta_f_2_or_more = 0;

  uint64_t num_delta_f_0_delta_g_2_or_more = 0;  // So delta_h=-delta_g
  uint64_t num_delta_f_0_delta_g_1 = 0;  // So delta_h=-1
  uint64_t num_delta_f_0_delta_g_0 = 0;  // So delta_h=0

  uint64_t num_delta_f_1_delta_g_3_or_more = 0;  // So delta_h=-delta_g+1
  uint64_t num_delta_f_1_delta_g_2 = 0;  // So delta_h=-1
  uint64_t num_delta_f_1_delta_g_1 = 0;  // So delta_h=0
  uint64_t num_delta_f_1_delta_g_0 = 0;  // So delta_h=1

  uint64_t sum_num_conflicts = 0;  // Sum of number of conflicts encountered in each CT node
  uint64_t num_num_conflicts = 0;  // Yes, probably same as num_generated

  CBSNode* dummy_start = nullptr;
  CBSNode* goal_node = nullptr;

  bool solution_found = false;
  int solution_cost = -2;
  double min_f_val;
  double focal_list_threshold;

  string max_mem;  // Set externally

  /////////////////////////////////////////////////////////////////////////////////////////
  // set params
  void setPrioritizeConflicts(conflict_prioritization pc) {
  	PC = pc;
  	heuristic_helper->PC = pc;
  }
  void setRectangleReasoning(bool r) { rectangle_helper.use_rectangle_reasoning = r; }
  void setRectangleReasoningForHeuristic(bool r) { heuristic_helper->rectangle_reasoning = r; }
  void setCorridorReasoning(bool c) { corridor_helper.use_corridor_reasoning = c; heuristic_helper->corridor_reasoning = c; }
  void setTargetReasoning(bool t) { target_reasoning = t; }
  void setTargetReasoningForHeuristic(bool t) { heuristic_helper->target_reasoning = t; }
  void setMutexReasoning(mutex_strategy m) {mutex_helper.strategy = m; heuristic_helper->mutex_reasoning = m; }
  void setDisjointSplitting(bool d) { disjoint_splitting = d; heuristic_helper->disjoint_splitting = d; }
  void setBypass(bypass_support b) { bypass = b; heuristic_helper->bypass = b; }
  void setConflictSelectionRule(conflict_selection c) { conflict_selection_rule = c; heuristic_helper->conflict_selection_rule = c; }
  void setNodeSelectionRule(node_selection n) { node_selection_rule = n; heuristic_helper->node_selection_rule = n; }
  void setNodeLimit(int n) { node_limit = n; }
  void setSeed(int s) { seed = s; heuristic_helper->seed = seed; }

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Runs the algorithm until the problem is solved or time is exhausted
  bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST);

  CBS(const Instance& instance, bool sipp, heuristics_type heuristic, int screen);
  CBS(vector<SingleAgentSolver*>& search_engines,
    const vector<ConstraintTable>& constraints,
      vector<Path>& paths_found_initially, heuristics_type heuristic, int screen);
  void clearSearchEngines();
  ~CBS();

  // Save results
  void saveResults(const string& fileName, const string& instanceName, bool writeHeader) const;

  void clear(); // used for rapid random restart

private:
  bool target_reasoning; // using target reasoning
  bool disjoint_splitting; // disjoint splitting
  bool mutex_reasoning; // using mutex reasoning
  bypass_support bypass = bypass_support::NONE; // using g-bypass or f-bypass
  conflict_prioritization PC; // prioritize conflicts
  conflict_selection conflict_selection_rule;
  node_selection node_selection_rule;

  MDDTable mdd_helper;
  RectangleReasoning rectangle_helper;
  CorridorReasoning corridor_helper;
  MutexReasoning mutex_helper;
  CBSHeuristic* heuristic_helper;

  pairing_heap<CBSNode*, compare<CBSNode::compare_node>> open_list;
  pairing_heap<CBSNode*, compare<CBSNode::secondary_compare_node>> focal_list;
  list<CBSNode*> allNodes_table;


  string getSolverName() const;

  int screen;

  double time_limit;
  int node_limit = MAX_NODES;
  double focal_w = 1.0;
  int cost_upperbound = MAX_COST;

  int seed;

  vector<ConstraintTable> initial_constraints;
  clock_t start;

  int num_of_agents;

  vector<Path*> paths;
  vector<Path> paths_found_initially;  // contain initial paths found
  // vector<MDD*> mdds_initially;  // contain initial paths found
  vector<SingleAgentSolver*> search_engines;  // used to find (single) agents' paths and mdd

  // init helper
  void init_heuristic(heuristics_type heuristic);
  // high level search
  bool findPathForSingleAgent(CBSNode* node, int ag, int lower_bound = 0);
  bool generateChild(CBSNode* child, CBSNode* curr);
  bool generateRoot();

  //conflicts
  void findConflicts(CBSNode& curr);
  void findConflicts(CBSNode& curr, int a1, int a2);
  shared_ptr<Conflict> chooseConflict(const CBSNode& node) const;
  void classifyConflicts(CBSNode& parent);
  // void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
  //  list<shared_ptr<Conflict>>& copy, int excluded_agent) const;
  void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
             list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agent) const;
  void removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const;
  //bool isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node);

  void computeSecondaryPriorityForConflict(Conflict& conflict, CBSNode& node);

  // Conflict Avoidance Table:
  ConflictAvoidanceTable* cat;
  void buildConflictAvoidanceTable(const CBSNode &node, int exclude_agent, ConflictAvoidanceTable &cat);
  void updateCat(CBSNode *prev_node, CBSNode *curr,
                 vector<Path*>& paths, ConflictAvoidanceTable *cat);

  // LCA-Jumping
  void findShortestPathFromPrevNodeToCurr(CBSNode *curr, CBSNode* prev,
											vector<CBSNode *>& steps_up_from_prev_node_to_lowest_common_ancestor,
											vector<CBSNode *>& steps_down_from_lowest_common_ancestor_to_curr_node);

  //update information
  inline void updatePaths(CBSNode* curr);
  inline void updatePaths(CBSNode* curr, vector<Path*>& the_paths);
  void updateFocalList();
  inline void releaseNodes();
  //inline void releaseMDDTable();
  // void copyConflictGraph(CBSNode& child, const CBSNode& parent);

  // print and save
  void printPaths() const;
  void printResults() const;
  void printConflicts(const CBSNode& curr) const;

  [[nodiscard]] bool validateSolution() const;
  inline int getAgentLocation(int agent_id, size_t timestep) const;
  inline void pushNode(CBSNode* node);
  void update_delta_h_and_delta_f_stats(CBSNode* curr);
  void update_delta_g_stats(CBSNode* child);
};
