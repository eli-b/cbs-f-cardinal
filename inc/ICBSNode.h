#pragma once
#include "common.h"
#include "Conflict.h"
// #include "MDD.h"

class ICBSNode
{
public:
	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const 
		{
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

	// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node 
	{
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const 
		{
			return n1->num_of_collisions >= n2->num_of_collisions;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	typedef boost::heap::pairing_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::pairing_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	// The following is used by  for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct ICBSNodeHasher 
	{
		std::size_t operator()(const ICBSNode* n) const {
			return std::hash<uint64_t>()(n->time_generated);
		}
	};

	// conflicts in the current paths
	list<shared_ptr<Conflict> > conflicts;
	list<shared_ptr<Conflict> > unknownConf;
	
	// The chosen conflict
	shared_ptr<Conflict> conflict;

	boost::unordered_map<int, int> conflictGraph; //<edge index, weight>
	ICBSNode* parent;

	list< pair< int, Path> > paths; // new paths
	list<Constraint> constraints; // new constraints
	

	int g_val;
	int h_val;
	int f_val;
	size_t depth; // depath of this CT node
	size_t makespan; // makespan over all paths
	int num_of_collisions; // number of conflicts in the current paths
	bool h_computed = false;

	uint64_t time_expanded;
	uint64_t time_generated;


	void clear();
	void printConflictGraph(int num_of_agents) const;
};


std::ostream& operator<<(std::ostream& os, const ICBSNode& node);


struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a{};
	const ICBSNode* n{};
	ConstraintsHasher(int a, ICBSNode* n) : a(a), n(n) {};

	struct EqNode
	{
		bool operator() (const ConstraintsHasher& c1, const ConstraintsHasher& c2) const
		{
			assert(c1.a == c2.a);
				
			std::set<Constraint> cons1, cons2;
			const ICBSNode* curr = c1.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == c1.a) {
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
					get<0>(curr->constraints.front()) == c2.a) {
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
			const ICBSNode* curr = entry.n;
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
