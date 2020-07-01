#include "CBSHeuristic.h"

int ZeroHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit)
{
  return 0;
}


bool CBSHeuristic::computeInformedHeuristics(CBSNode& curr, double time_limit){
  curr.h_computed =true;
	start_time = clock();
	this->time_limit = time_limit;

  int h = computeInformedHeuristicsValue(curr, time_limit);
	if (h < 0)
		return false;

	curr.h_val = max(h, curr.h_val);

	// update tie-breaking for node selection if necessary
	if (node_selection_rule == node_selection::NODE_H)
		curr.tie_breaking = curr.h_val;

	return true;
}


void CBSHeuristic::copyConflictGraph(CBSNode& child, const CBSNode& parent)
{
	//copy conflict graph
  // Do nothing
}

bool CBSHeuristic::shouldEvalHeuristic(CBSNode* node)
{
  return !node->h_computed;
}

void CBSHeuristic::computeQuickHeuristics(CBSNode& node) // for non-root node
{
	node.h_val = max(0, node.parent->g_val + node.parent->h_val - node.g_val); // pathmax
	set<pair<int, int>> conflicting_agents;
	switch (node_selection_rule)
	{
	case node_selection::NODE_H:
		node.tie_breaking = node.h_val;
		break;
	case node_selection::NODE_DEPTH:
		node.tie_breaking = -node.depth; // we use negative depth because we prefer nodes with larger depths
		break;
	case node_selection::NODE_CONFLICTS:
		node.tie_breaking = (int) (node.conflicts.size() + node.unknownConf.size());
		break;
	case node_selection::NODE_CONFLICTPAIRS:
		for (const auto& conflict : node.unknownConf)
		{
			auto agents = make_pair(min(conflict->a1, conflict->a2), max(conflict->a1, conflict->a2));
			if (conflicting_agents.find(agents) == conflicting_agents.end())
			{
				conflicting_agents.insert(agents);
			}
		}
		node.tie_breaking = (int) (node.conflicts.size() + conflicting_agents.size());
		break;
	case node_selection::NODE_MVC:
		node.tie_breaking = MVConAllConflicts(node);
		break;
  case node_selection::NODE_RANDOM:
    break;
	}
	copyConflictGraph(node, *node.parent);
}

int CBSHeuristic::MVConAllConflicts(CBSNode& curr)
{
	auto G = buildConflictGraph(curr);
	return minimumVertexCover(G);
}
