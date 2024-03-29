#pragma once

#include "common.h"
#include "Conflict.h"

enum conflict_prioritization { OFF, BY_G_CARDINAL, BY_F_CARDINAL };

enum node_selection { NODE_RANDOM, NODE_H, NODE_DEPTH, NODE_CONFLICTS, NODE_CONFLICTPAIRS, NODE_MVC };

enum bypass_support { NONE, G_BYPASS, F_BYPASS };

enum split_on_agent { SECOND, FIRST, UNSET };  // For disjoint splitting

class CBSNode
{
public:
	// the following is used to compare nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const 
		{
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN to compare nodes by f_val (top of the heap has min f_val)

	// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node 
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const 
		{
			if (n1->tie_breaking == n2->tie_breaking)
				return n1->time_generated >= n2->time_generated;  // Return an arbitrary consistent answer
			return n1->tie_breaking > n2->tie_breaking;
		}
	};  // used by FOCAL to compare nodes by tie_breaking value (top of the heap has min tie_breaking value)

	typedef boost::heap::pairing_heap<CBSNode*, boost::heap::compare<CBSNode::compare_node>>::handle_type open_handle_t;
	typedef boost::heap::pairing_heap<CBSNode*, boost::heap::compare<CBSNode::secondary_compare_node>>::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	// The following is used by  for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specialized template inside std namespace
	struct ICBSNodeHasher 
	{
		std::size_t operator()(const CBSNode* n) const
		{
			return std::hash<uint64_t>()(n->time_generated);
		}
	};

	// conflicts in the current paths
	list<shared_ptr<Conflict>> conflicts;
	list<shared_ptr<Conflict>> unknownConf;

	// The chosen conflict
	shared_ptr<Conflict> conflict;

	CBSNode* parent;

	boost::unordered_map<int, int> dependenceGraph; //<edge index : weight>
	// TODO: This can be deleted.
	// It's used by the DG-, EWDG- and NVWEWDG- MVC heuristics. Do you mean it doesn't save enough time to be worth it?

	list<pair<int, Path>> paths; // new paths
	list<Constraint> constraints; // new constraints


	int g_val;
	int h_val;
	int depth; // depth of this CT node
	size_t makespan = 0; // makespan over all paths
	int tie_breaking = 0; // tie breaking for node selection
	bool h_computed = false;

	uint64_t time_expanded;
	uint64_t time_generated;

	split_on_agent split_on_which_agent = split_on_agent::UNSET;

	void clear();
	void printConflictGraph(int num_of_agents) const;
};


std::ostream& operator<<(std::ostream& os, const CBSNode& node);


struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a{};
	const CBSNode* n{};
	ConstraintsHasher(int a, CBSNode* n) : a(a), n(n) {};

	struct EqNode
	{
		bool operator()(const ConstraintsHasher& c1, const ConstraintsHasher& c2) const
		{
			if (c1.a != c2.a)
				return false;
				
			std::set<Constraint> cons1, cons2;
			const CBSNode* curr = c1.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == c1.a)
				{
					for (auto con : curr->constraints)
						cons1.insert(con);
				}
				curr = curr->parent;
			}
			curr = c2.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == c2.a)
				{
					for (auto con : curr->constraints)
						cons2.insert(con);
				}

				curr = curr->parent;
			}

			return equal(cons1.begin(), cons1.end(), cons2.begin(), cons2.end());
		}
	};

	struct Hasher
	{
		std::size_t operator()(const ConstraintsHasher& entry) const
		{
			const CBSNode* curr = entry.n;
			size_t cons_hash = 0;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == entry.a)
				{
					for (auto con : curr->constraints)
					{
						cons_hash += 3 * std::hash<int>()(std::get<0>(con)) +
									 5 * std::hash<int>()(std::get<1>(con)) +
									 7 * std::hash<int>()(std::get<2>(con)) +
									 11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons_hash;
		}
	};
};
