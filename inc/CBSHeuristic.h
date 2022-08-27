#pragma once

#include "MDD.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "MutexReasoning.h"
//#include "CBS.h"
#include <coin/OsiGrbSolverInterface.hpp>

enum heuristics_type { ZERO, CG, NVWCG, DG, NVWDG, WDG, NVWEWDG, HEURISTICS_COUNT };


struct HTableEntry // look-up table entry 
{
	int a1;
	int a2;
	CBSNode* n;

	HTableEntry() = default;

	HTableEntry(int a1, int a2, CBSNode* n) : a1(a1), a2(a2), n(n) {};

	struct EqNode
	{
		bool operator() (const HTableEntry& h1, const HTableEntry& h2) const
		{
			std::set<Constraint> cons1[2], cons2[2];
			const CBSNode* curr = h1.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons1[0].insert(con);
						cons2[0].insert(con);
					}
				}
				else
				{
					if (get<0>(curr->constraints.front()) == h1.a1)
						for (auto con : curr->constraints)
							cons1[0].insert(con);
					else if (get<0>(curr->constraints.front()) == h1.a2)
						for (auto con : curr->constraints)
							cons2[0].insert(con);
				}

				curr = curr->parent;
			}
			curr = h2.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons1[1].insert(con);
						cons2[1].insert(con);
					}
				}
				else
				{
					if (get<0>(curr->constraints.front()) == h2.a1)
						for (auto con : curr->constraints)
							cons1[1].insert(con);
					else if (get<0>(curr->constraints.front()) == h2.a2)
						for (auto con : curr->constraints)
							cons2[1].insert(con);
				}

				curr = curr->parent;
			}
			if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size())
				return false;

			if (!equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin()))
				return false;
			return equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin());
		}
	};


	struct Hasher
	{
		size_t operator()(const HTableEntry& entry) const
		{
			CBSNode* curr = entry.n;
			size_t cons1_hash = 0, cons2_hash = 0;
			while (curr->parent != nullptr)
			{
				if (get<0>(curr->constraints.front()) == entry.a1 ||
					get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons1_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				else if (get<0>(curr->constraints.front()) == entry.a2 ||
					get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons2_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons1_hash ^ (cons2_hash << 1);
		}
	};
};


typedef unordered_map<HTableEntry, int, HTableEntry::Hasher, HTableEntry::EqNode> HTable;

class CBSHeuristic
{
public:
	// Parameters for the WDG heuristic. TODO: Find a way to clean the API.
	bool rectangle_reasoning; // using rectangle reasoning in the subproblem solver
	bool corridor_reasoning; // using corridor reasoning in the subproblem solver
	bool target_reasoning; // using target reasoning in the subproblem solver
	mutex_strategy mutex_reasoning; // using mutex reasoning in the subproblem solver
	bool disjoint_splitting; // using disjoint splitting in the subproblem solver
	bypass_support bypass; // using g-bypassing in the subproblem solver
	conflict_prioritization PC; // using prioritizing conflicts in the subproblem solver, and possibly choosing f-cardinal conflicts
	conflict_selection conflict_selection_rule;  // for the subproblem solver
	node_selection node_selection_rule;  // for the subproblem solver and for setting the tie-breaking value on nodes
										 // after computing their heuristic
	int screen = 0;  // Mostly for subsolver
	int seed = 0;

	// Runtime stats
	double runtime_build_graph = 0;
	double runtime_solve_MVC = 0;
	double runtime_fcardinal_reasoning = 0;

	// More stats for specific subclasses. TODO: Find a way to move them to the subclasses
	uint64_t num_merge_MDDs = 0;  // Only set by the DG heuristic
	uint64_t num_solve_2agent_problems = 0;  // Only set by the WDG heuristic
	uint64_t num_memoization_hits = 0; // number of times when memoization helps - only set by the DG and WDG heuristics

	uint64_t f_cardinal_conflicts_found = 0;
	uint64_t semi_f_cardinal_g_cardinal_conflicts_found = 0;
	uint64_t g_cardinal_conflicts_checked_for_f_cardinality = 0;

	CBSHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
				 const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper) :
		CBSHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper,
			   		 true, false) {}

	virtual void copyConflictGraph(CBSNode& child, const CBSNode& parent);
	bool computeInformedHeuristics(CBSNode& curr, double time_limit);  // Called when before a node is expanded
	virtual void computeQuickHeuristics(CBSNode& curr); // this function is called when generating a CT node
	virtual void init() {}
	virtual void clear() {}
	bool shouldEvalHeuristic(CBSNode* node);
	virtual heuristics_type getType() const = 0;  // Just for printing its name. TODO: Consider just returning a string instead.

protected:
	int num_of_agents;

	double time_limit;
	double start_time;

	int node_limit = 64;  // Terminate the CBS subsolver if the number of its expanded nodes exceeds the node limit.

	OsiGrbSolverInterface mvc_model;
	typedef int OsiGrbConstraint;
	bool solved_once = false;  // If not, need to cal mvc_model.initial_solve().

	inline void remove_model_constraints(vector<vector<vector<OsiGrbConstraint>>>& Constraints,
									     const vector<vector<tuple<int,int>>>& CG, vector<bool>& CgNodeDegrees);
	inline void calc_heuristic_graph_vertex_degrees(const vector<vector<tuple<int,int>>>& graph, vector<int>& nodeDegrees);
	inline void add_constraints_for_heuristic_graph_edge(int a1, int a2, int weight,
														 int a1_cost_increase, int a2_cost_increase, int& h,
														 vector<int>& HGNodeDegrees,
														 vector<vector<vector<OsiGrbConstraint>>>& Constraints,
														 int& num_of_nontrivial_HG_edges, vector<bool>& nodeHasNontrivialEdges);
	inline void add_mvc_model_constraints_from_graph(const vector<vector<tuple<int,int>>>& HG,
													 vector<int>& CgNodeDegrees, vector<vector<vector<OsiGrbConstraint>>>& Constraints,
													 int& num_of_nontrivial_HG_edges, vector<bool>& nodeHasNontrivialEdges,
													 vector<int>& lowestCostIncrease, vector<int>& highestCostIncreases,
													 int& highestCostIncrease, int& h);
	inline void add_mvc_model_constraints_of_agent(const vector<vector<tuple<int,int>>>& graph, int i,
												   int assume_cost_increased_by,
												   vector<int>& HGNodeDegrees,
												   vector<vector<vector<OsiGrbConstraint>>>& Constraints,
												   int& num_of_nontrivial_HG_edges, vector<bool>& nodeHasNontrivialEdges);
	inline void remove_mvc_model_constraints_of_agent(vector<vector<vector<OsiGrbConstraint>>>& Constraints, int i,
												   vector<int>& HGNodeDegrees,
												   vector<bool>& nodeHasNontrivialEdges);

	const vector<Path*>& paths;  // For DG and WDG only
	const vector<SingleAgentSolver*>& search_engines;  // For WDG only
	const vector<ConstraintTable>& initial_constraints;  // For WDG only
	MDDTable& mdd_helper;  // For DG only

	CBSHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
				 const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper, bool max_vertex_weight_is_1,
				 bool need_aux_variables);
	int sizeOfAllConflictsGraphMVC(CBSNode& curr);  // For tie-breaking
	vector<vector<tuple<int,int>>> buildConflictGraph(const CBSNode& curr) const;
	int minimumVertexCover(const vector<vector<tuple<int,int>>>& HG, CBSNode& node);
	virtual int computeInformedHeuristicsValue(CBSNode& curr, double time_limit) = 0;
};


class ZeroHeuristic: public CBSHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::ZERO;
	}

	using CBSHeuristic::CBSHeuristic;

protected:
	int computeInformedHeuristicsValue(CBSNode& curr, double time_limit) override;
};

class CGHeuristic: public CBSHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::CG;
	}

	using CBSHeuristic::CBSHeuristic;

protected:
	int computeInformedHeuristicsValue(CBSNode& curr, double time_limit) override;
	virtual bool buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& CG, int& num_edges, int& max_edge_weight);
};

class DGHeuristic: public CGHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::DG;
	}

	using CGHeuristic::CGHeuristic;

	virtual void init()
	{
		lookupTable.resize(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			lookupTable[i].resize(num_of_agents);
		}
	}

	void copyConflictGraph(CBSNode& child, const CBSNode& parent) override;

	void clear() override { lookupTable.clear(); }

protected:
	vector<vector<HTable> > lookupTable;

	bool buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& DG, int& num_edges, int& max_edge_weight) override;
	virtual bool SyncMDDs(const MDD& mdd1, const MDD& mdd2);
	virtual bool dependent(int a1, int a2, CBSNode& node);
};

class WDGHeuristic: public DGHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::WDG;
	}

	WDGHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
				 const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper) :
			DGHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper,
			   			false, false) {}

	using DGHeuristic::DGHeuristic;  // To inherit the other one

protected:
	bool buildGraph(CBSNode& node, vector<vector<tuple<int,int>>>& WDG, int& num_edges, int& max_edge_weight) override;
	int solve2Agents(int a1, int a2, const CBSNode& node, bool g_cardinal_or_pseudo);
	int DPForWMVC(vector<int>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far);
};


class NVWCGHeuristic: public CGHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::NVWCG;
	}

	NVWCGHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
				   const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper) :
			CGHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper,
						false, true) {}

protected:
	bool buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& CG, int& num_edges, int& max_edge_weight) override;
};

class NVWDGHeuristic: public DGHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::NVWDG;
	}

	NVWDGHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
				   const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper) :
			DGHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper,
						false, true) {}

protected:
	bool buildGraph(CBSNode& node, vector<vector<tuple<int,int>>>& DG, int& num_edges, int& max_edge_weight) override;
};

class NVWEWDGHeuristic: public WDGHeuristic {
public:
	virtual heuristics_type getType() const
	{
		return heuristics_type::NVWEWDG;
	}

	NVWEWDGHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
					 const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper) :
			WDGHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper,
						 false, true) {}

protected:
	bool buildGraph(CBSNode& node, vector<vector<tuple<int,int>>>& WDG, int& num_edges, int& max_edge_weight) override;
};
