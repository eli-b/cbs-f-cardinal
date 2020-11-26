#include "CBSHeuristic.h"

int CGHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit) {
  vector<vector<tuple<int,int>>> HG(num_of_agents);
  for (int i = 0 ; i < num_of_agents ; ++i)
  {
	  HG[i].reserve(num_of_agents);
	  for (int j = 0 ; j < num_of_agents ; ++j)
	  	HG[i][j] = make_tuple(0, 0);
  }
  int num_edges;
  int max_edge_weight;
  bool succ = buildGraph(curr, HG, num_edges, max_edge_weight);
  if (!succ)
  	return -1;
  if (num_edges == 0)
	return 0;
  if (num_edges == 1)  // In our domain the other agent in a target conflict will increase its cost by 1, so we can ignore whether it's a target conflict
	return max_edge_weight;
  int h = minimumVertexCover(HG, curr);
  return h;
}

// Returns a vector of vectors where ret[a1_index][a2_index] == ret[a2_index][a1_index] == tuple<1,1> if a1 and a2 have at
// least one cardinal conflict, tuple<0,0> otherwise
bool CGHeuristic::buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& CG, int& num_edges, int& max_edge_weight)
{
	num_edges = 0;
	max_edge_weight = 1;
	for (const auto& conflict : curr.conflicts)
    {
      if (conflict->priority == conflict_priority::CARDINAL)
        {
          int a1 = conflict->a1;
          int a2 = conflict->a2;
          if (get<0>(CG[a1][a2]) == 0)
            ++num_edges;
          CG[a1][a2] = make_tuple(1, 1);
          CG[a2][a1] = make_tuple(1, 1);
        }
    }
	runtime_build_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;

	if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // ran out of time
		return false;
	return true;
}


// Returns a vector of vectors where ret[a1_index][a2_index] == tuple<1,X> if a1 and a2 have at
// least one cardinal conflict, where X is the expected cost increase for a1 from resolving that conflict, tuple<0,0> otherwise
bool NVWCGHeuristic::buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& CG, int& num_edges, int& max_edge_weight)
{
	max_edge_weight = 1;
	num_edges = 0;
	for (const auto& conflict : curr.conflicts)
	{
		if (conflict->priority == conflict_priority::CARDINAL)
		{
			int a1 = conflict->a1;
			int a2 = conflict->a2;
			if (get<0>(CG[a1][a2]) == 0)  // First conflict between these agents that we encounter
				++num_edges;
			uint64_t cost_increase_a1 = max(1, get<1>(CG[a1][a2]));  // In case there are multiple conflicts between the two agents,
																		// take the maximum cost increase. This is a minor point because
																		// the solver would handle it even if we didn't.
			if (conflict->type == conflict_type::TARGET ||
				(!target_reasoning && conflict->type == conflict_type::STANDARD &&
				 get<3>(conflict->constraint1.front()) > conflict->a1_path_cost)
				) {  // A cardinal target conflict
				// a1 is the agent that's at its target
				int time_step = get<3>(conflict->constraint1.front());
				cost_increase_a1 = time_step + 1 - conflict->a1_path_cost;
			}
			CG[a1][a2] = make_tuple(1, cost_increase_a1);
			CG[a2][a1] = make_tuple(1, 1);
		}
	}
	runtime_build_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;

	if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // ran out of time
		return false;
	return true;
}
