﻿#pragma once
#include "SingleAgentSolver.h"

// using boost::heap::fibonacci_heap;
// using boost::heap::compare;

class AStarNode: public LLNode
{
public:


	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< AStarNode*, compare<LLNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< AStarNode*, compare<LLNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;


	AStarNode() : LLNode() {}

	AStarNode(int loc, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts = 0, bool in_openlist = false) :
		LLNode(loc, g_val, h_val, parent, timestep, num_of_conflicts, in_openlist) {}

	AStarNode(const AStarNode& other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
		in_openlist = other.in_openlist;
		open_handle = other.open_handle;
		focal_handle = other.focal_handle;
		num_of_conflicts = other.num_of_conflicts;
	}
	~AStarNode() {}

	// The following is used by for generating the hash value of a nodes
	struct NodeHasher
	{
		std::size_t operator()(const AStarNode* n) const
		{
			size_t loc_hash = std::hash<int>()(n->location);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

	// The following is used for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep
	struct eqnode
	{
		bool operator()(const AStarNode* s1, const AStarNode* s2) const
		{
			return (s1 == s2) || (s1 && s2 &&
				s1->timestep == s2->timestep);
		}
	};
};


class SpaceTimeAStar: public SingleAgentSolver
{
public:
	// find path by time-space A* search
	// Returns a shortest path that satisfies the constraints of the give node  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is an underestimation of the length of the path in order to speed up the search.
	Path findPath(const ICBSNode& node, const ConstraintTable& initial_constraints,
						const vector<Path*>& paths, int agent, int lower_bound);

	string getName() const { return "AStar"; }

	SpaceTimeAStar(const Instance& instance, int agent):
		SingleAgentSolver(instance, agent) {}

private:
	// define typedefs and handles for heap
	typedef boost::heap::fibonacci_heap< AStarNode*, boost::heap::compare<AStarNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< AStarNode*, boost::heap::compare<AStarNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;

	int min_f_val; // minimal f value in OPEN
	int lower_bound; // Threshold for FOCAL

	// define typedef for hash_map
	typedef boost::unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
	hashtable_t allNodes_table;

	// Updates the path datamember
	void updatePath(const LLNode* goal, std::vector<PathEntry> &path);
	void updateFocalList();
	inline AStarNode* popNode();
	inline void pushNode(AStarNode* node);
	void releaseClosedListNodes();

};