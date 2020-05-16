#pragma once
#include "MDD.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"

enum heuristics_type { ZERO, CG, DG, WDG, STRATEGY_COUNT };


struct HTableEntry // look-up table entry 
{
	int a1{};
	int a2{};
	CBSNode* n{};

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
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
					for (auto con : curr->constraints)
					{
						cons1[0].insert(con);
						cons2[0].insert(con);
					}
				}
				else {
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
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
					for (auto con : curr->constraints)
					{
						cons1[1].insert(con);
						cons2[1].insert(con);
					}
				}
				else {
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
	heuristics_type type;
	rectangle_strategy rectangle_reasoning; // using rectangle reasoning
	corridor_strategy corridor_reasoning; // using corridor reasoning
	bool target_reasoning; // using target reasoning
	bool mutex_reasoning; // using mutex reasoning
	bool disjoint_splitting; // disjoint splitting
	bool PC; // prioritize conflicts
	conflict_selection conflict_seletion_rule;
	node_selection node_selection_rule;

	double runtime_build_dependency_graph = 0;
	double runtime_solve_MVC = 0;

	uint64_t num_merge_MDDs = 0;
	uint64_t num_solve_2agent_problems = 0;
	uint64_t num_memoization = 0; // number of times when memoization helps

	CBSHeuristic(int num_of_agents,
							const vector<Path*>& paths,
							vector<SingleAgentSolver*>& search_engines,
							const vector<ConstraintTable>& initial_constraints,
							MDDTable& mdd_helper) : num_of_agents(num_of_agents),
		paths(paths), search_engines(search_engines), initial_constraints(initial_constraints), mdd_helper(mdd_helper) {}
	
	void init()
	{
		if (type == heuristics_type::DG || type == heuristics_type::WDG)
		{
			lookupTable.resize(num_of_agents);
			for (int i = 0; i < num_of_agents; i++)
			{
				lookupTable[i].resize(num_of_agents);
			}
		}
	}

	bool computeInformedHeuristics(CBSNode& curr, double time_limit); // this function is called when popping a CT node for the first time
	void computeQuickHeuristics(CBSNode& curr); // this function is called when generating a CT node
	void copyConflictGraph(CBSNode& child, const CBSNode& parent);
	void clear() { lookupTable.clear(); }

private:
	int screen = 0;
	int num_of_agents;
	int ILP_node_threshold = 5; // run ILP if #nodes in the conrflict graph is larger than the threshold
	int ILP_edge_threshold = 10; // run ILP if #edges in the conrflict graph is larger than the threshold
	vector<vector<HTable> > lookupTable;

	double time_limit;
	int node_limit = 64;  // terminate the sub CBS solver if the number of its expanded nodes exceeds the node limit.
	double start_time;

	const vector<Path*>& paths;
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;
	MDDTable& mdd_helper;


	vector<int> buildConflictGraph(const CBSNode& curr) const;
	void buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges);
	bool buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges);
	bool buildWeightedDependencyGraph(CBSNode& curr, vector<int>& CG);

	bool dependent(int a1, int a2, CBSNode& node); // return true if the two agents are dependent
	int solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal);
	bool SyncMDDs(const MDD &mdd1, const MDD& mdd2); 	// Match and prune MDD according to another MDD.

	int minimumVertexCover(const vector<int>& CG);
	int minimumVertexCover(const vector<int>& CG, int old_mvc, int cols, int num_of_edges);
	bool KVertexCover(const vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols);
	int greedyMatching(const vector<int>& CG, int cols);
	int minimumWeightedVertexCover(const vector<int>& CG);
	int weightedVertexCover(const vector<int>& CG);
	int DPForWMVC(vector<int>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far);
	int ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value);
	int MVConAllConflicts(CBSNode& curr);
	
};




