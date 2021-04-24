#include "CBSHeuristic.h"


// Returns a vector of vectors where ret[a1_index][a2_index] == ret[a2_index][a1_index] == tuple<1,1> if a1 and a2 have
// to increase their combined cost to resolve all conflicts between themselves, tuple<0,0> otherwise.
bool DGHeuristic::buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& DG, int& num_edges, int& max_edge_weight)
{
	max_edge_weight = 1;
	num_edges = 0;
	for (auto& conflict : curr.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (conflict->priority == conflict_priority::CARDINAL ||
			conflict->priority == conflict_priority::PSEUDO_CARDINAL
			)
		{
			curr.dependenceGraph[idx] = 1;
		}
		else if (curr.dependenceGraph.find(idx) == curr.dependenceGraph.end())  // Not already found in the map
		{
			auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &curr));  // check the lookup table first
			if (got != lookupTable[a1][a2].end())  // Found in the table
			{
				num_memoization_hits++;
				curr.dependenceGraph[idx] = got->second;
			}
			else
			{
				curr.dependenceGraph[idx] = dependent(a1, a2, curr) ? 1 : 0;
				lookupTable[a1][a2][HTableEntry(a1, a2, &curr)] = curr.dependenceGraph[idx];
			}
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
	}

	// Upgrade conflicts between dependent agents to pseudo-g-cardinal, if necessary:
	for (auto& conflict : curr.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (conflict->priority > conflict_priority::CARDINAL &&  // Higher enum value means *lower* priority
			curr.dependenceGraph[idx] > 0)
		{
			if (conflict->priority == conflict_priority::CARDINAL)
				conflict->priority = conflict_priority::PSEUDO_CARDINAL; // the two agents are dependent, although resolving this conflict won't increase the cost immediately
		}
	}
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = curr.dependenceGraph.find(i * num_of_agents + j);
			if (got != curr.dependenceGraph.end() && got->second > 0)
			{
				if (get<0>(DG[i][j]) == 0)
					++num_edges;
				DG[i][j] = make_tuple(got->second, 1);
				DG[j][i] = make_tuple(got->second, 1);
			}
		}
	}
	runtime_build_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


bool DGHeuristic::dependent(int a1, int a2, CBSNode& node) // return true if the two agents are dependent
{
	const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
	const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
	if (mdd1->levels.size() > mdd2->levels.size()) // swap
		std::swap(mdd1, mdd2);
	num_merge_MDDs++;
	return !SyncMDDs(*mdd1, *mdd2);
}

bool DGHeuristic::SyncMDDs(const MDD &mdd, const MDD& other) // assume mdd.levels <= other.levels
{
	if (other.levels.size() <= 1) // Either of the MDDs was already completely pruned already
		return false;

	SyncMDD copy(mdd);
	if (copy.levels.size() < other.levels.size())
	{
		size_t i = copy.levels.size();
		copy.levels.resize(other.levels.size());
		for (; i < copy.levels.size(); i++)
		{
			SyncMDDNode* parent = copy.levels[i - 1].front();
			auto node = new SyncMDDNode(parent->location, parent);
			parent->children.push_back(node);
			copy.levels[i].push_back(node);

		}
	}
	// Cheaply find the coexisting nodes on level zero - all nodes coexist because agent starting points never collide
	copy.levels[0].front()->coexistingNodesFromOtherMdds.push_back(other.levels[0].front());

	// what if level.size() = 1?
	for (size_t i = 1; i < copy.levels.size(); i++)
	{
		for (auto node = copy.levels[i].begin(); node != copy.levels[i].end();)
		{
			// Go over all the node's parents and test their coexisting nodes' children for co-existance with this node
			for (auto parent = (*node)->parents.begin(); parent != (*node)->parents.end(); parent++)
			{
				for (const MDDNode* parentCoexistingNode : (*parent)->coexistingNodesFromOtherMdds)
				{
					for (const MDDNode* childOfParentCoexistingNode : parentCoexistingNode->children)
					{
						if ((*node)->location == childOfParentCoexistingNode->location) // vertex conflict
							continue;
						else if ((*node)->location == parentCoexistingNode->location && (*parent)->location == childOfParentCoexistingNode->location) // edge conflict
							continue;

						auto it = (*node)->coexistingNodesFromOtherMdds.cbegin();
						for (; it != (*node)->coexistingNodesFromOtherMdds.cend(); ++it)
						{
							if (*it == childOfParentCoexistingNode)
								break;
						}
						if (it == (*node)->coexistingNodesFromOtherMdds.cend())
						{
							(*node)->coexistingNodesFromOtherMdds.push_back(childOfParentCoexistingNode);
						}
					}
				}
			}
			if ((*node)->coexistingNodesFromOtherMdds.empty())
			{
				// delete the node, and continue up the levels if necessary
				SyncMDDNode* p = *node;
				node++;
				copy.deleteNode(p, i);
			}
			else
				node++;
		}
		if (copy.levels[i].empty())
		{
			copy.clear();
			return false;
		}
	}
	copy.clear();
	return true;
}

void DGHeuristic::copyConflictGraph(CBSNode& child, const CBSNode& parent)
{
  unordered_set<int> changed;
  for (const auto& p : child.paths) {
    changed.insert(p.first);
  }
  for (auto e : parent.dependenceGraph) {
    if (changed.find(e.first / num_of_agents) == changed.end() &&
			changed.find(e.first % num_of_agents) == changed.end())
      child.dependenceGraph[e.first] = e.second;

  }
}


// Returns a vector of vectors where ret[a1_index][a2_index] == tuple<W,X> if a1 and a2 have
// to increase their combined cost to resolve all conflicts between themselves,
// where X is the expected cost increase for a1 from resolving that conflict
// and W is 1 for non-corridor conflicts or the minimum cost of resolving the corridor conflict (can be more than 1),
// tuple<0,0> otherwise.
bool NVWDGHeuristic::buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& DG, int& num_edges, int& max_edge_weight) {
	bool succ = DGHeuristic::buildGraph(curr, DG, num_edges, max_edge_weight);
	if (!succ)
		return false;
	// Add the near-vertex weights
	clock_t t = clock();
	for (const auto& conflict : curr.conflicts)
	{
		if (conflict->priority == conflict_priority::CARDINAL ||
			conflict->priority == conflict_priority::PSEUDO_CARDINAL
			)
		{
			int a1 = conflict->a1;
			int a2 = conflict->a2;
			// No need to set DG[a1][a2] and DG[a2][a1] for the simple case - DGHeuristic::buildGraph already did
			uint64_t cost_increase_a1 = get<1>(DG[a1][a2]);
			uint64_t cost_increase_a2 = get<1>(DG[a2][a1]);

			if (conflict->type == conflict_type::TARGET ||
				(!target_reasoning && conflict->type == conflict_type::STANDARD &&
				 get<3>(conflict->constraint1.front()) > conflict->a1_path_cost)
				 ) {  // A g-cardinal target conflict (at least semi-f-cardinal with the MVC heuristic)
				// a1 is the agent that's at its target
				int time_step = get<3>(conflict->constraint1.front());
				int cost_increase_a1_from_this_conflict = time_step + 1 - conflict->a1_path_cost;
				if (1 + cost_increase_a1_from_this_conflict < cost_increase_a1 + cost_increase_a2)
					continue;  // A previously checked conflict would contribute more. TODO: Allow two sets of constraints from a target conflict and a corridor conflict between the same pair of agents.
				DG[a1][a2] = make_tuple(1, cost_increase_a1);
				DG[a2][a1] = make_tuple(1, 1);
			}
			else if (conflict->type == conflict_type::CORRIDOR)  // Implementing support for reasoning about corridors
																 // when corridor reasoning isn't enabled isn't worth the trouble
			{  // A cardinal corridor conflict
				int cost_increase_a1_from_this_conflict = conflict->c1_lookahead - conflict->c1;
				int cost_increase_a2_from_this_conflict = conflict->c2_lookahead - conflict->c2;
				int edge_weight = max(1, min(cost_increase_a1_from_this_conflict, cost_increase_a2_from_this_conflict));
				max_edge_weight = max(max_edge_weight, edge_weight);
				if (cost_increase_a1_from_this_conflict + cost_increase_a2_from_this_conflict < cost_increase_a1 + cost_increase_a2)
					continue;  // A previously checked conflict would contribute more. TODO: Allow two sets of constraints from a target conflict and a corridor conflict between the same pair of agents.
				DG[a1][a2] = make_tuple(edge_weight, cost_increase_a1_from_this_conflict);
				DG[a2][a1] = make_tuple(edge_weight, cost_increase_a2_from_this_conflict);
			}
		}
	}
	runtime_build_graph += (double) (clock() - t) / CLOCKS_PER_SEC;
	return true;
}
