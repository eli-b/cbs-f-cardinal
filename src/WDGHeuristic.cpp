#include "CBSHeuristic.h"
#include "CBS.h"


bool WDGHeuristic::buildGraph(CBSNode& node, vector<vector<tuple<int,int>>>& WDG, int& num_edges, int& max_edge_weight)
{
	num_edges = 0;
	max_edge_weight = 1;
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (node.dependenceGraph.find(idx) != node.dependenceGraph.end())
			continue;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization_hits++;
			node.dependenceGraph[idx] = got->second;
		}
		else if (rectangle_reasoning || mutex_reasoning)
		{
			node.dependenceGraph[idx] = solve2Agents(a1, a2, node, false);
			assert(node.dependenceGraph[idx] >= 0);
			if (screen == 2)
			{
				cout << "Weight of " << *conflict << " is " << node.dependenceGraph[idx] << endl;
			}
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.dependenceGraph[idx];
		}
		else
		{
			bool g_cardinal = conflict->priority == conflict_priority::G_CARDINAL;
			bool g_cardinal_or_pseudo = g_cardinal;
			if (!g_cardinal) // using merging MDD methods before running 2-agent instance
			{
				g_cardinal_or_pseudo = dependent(a1, a2, node);
			}
			if (g_cardinal_or_pseudo) // run 2-agent solver only for dependent agents
			{
				node.dependenceGraph[idx] = solve2Agents(a1, a2, node, true);
				assert(node.dependenceGraph[idx] >= 1);
				if (screen == 2)
				{
					cout << "Weight of " << *conflict << " is " << node.dependenceGraph[idx] << endl;
				}
				// Incorrect to assume a target conflict would necessarily surprise the heuristic. Compare with DGHeuristic.cpp.
			}
			else
			{
				node.dependenceGraph[idx] = 0;
			}
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.dependenceGraph[idx];
		}

		if (node.dependenceGraph[idx] == MAX_COST) // no solution
		{
			return false;  // Why? Can't this info be used in the heuristic? For example, give the node $h=\infty$?
		}

		if (node.dependenceGraph[idx] > 0)
		{
			if (conflict->priority == conflict_priority::SEMI_G_CARDINAL)
				conflict->priority = conflict_priority::PSEUDO_G_CARDINAL_SEMI_G_CARDINAL; // the two agents are dependent, although resolving this conflict won't increase the cost immediately
			if (conflict->priority == conflict_priority::NON_G_CARDINAL)
				conflict->priority = conflict_priority::PSEUDO_G_CARDINAL_NON_G_CARDINAL; // the two agents are dependent, although resolving this conflict won't increase the cost immediately
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
	}

	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = node.dependenceGraph.find(i * num_of_agents + j);
			if (got != node.dependenceGraph.end() && got->second > 0)
			{
				if (get<0>(WDG[i][j]) == 0)
					++num_edges;
				WDG[i][j] = make_tuple(got->second, 1);
				WDG[j][i] = make_tuple(got->second, 1);
				if (got->second > max_edge_weight)
					max_edge_weight = got->second;
			}
		}
	}
	runtime_build_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


// recursive component of dynamic programming for weighted vertex cover
int
WDGHeuristic::DPForWMVC(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range,
						int& best_so_far)
{
	if (sum >= best_so_far)
		return MAX_COST;
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int) x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}

	int cols = x.size();

	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = DPForWMVC(x, i + 1, sum + x[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost;
	}

	return best_so_far;
}

int WDGHeuristic::solve2Agents(int a1, int a2, const CBSNode& node, bool g_cardinal_or_pseudo)
{
	vector<SingleAgentSolver*> engines(2);
	engines[0] = search_engines[a1];
	engines[1] = search_engines[a2];
	vector<vector<PathEntry>> initial_paths(2);
	initial_paths[0] = *paths[a1];
	initial_paths[1] = *paths[a2];
	vector<ConstraintTable> constraints{
			ConstraintTable(initial_constraints[a1]),
			ConstraintTable(initial_constraints[a2]) };
	constraints[0].build(node, a1);
	constraints[1].build(node, a2);
	CBS cbs(engines, constraints, initial_paths, heuristics_type::CG, screen == 3 ? 2 : 0);
	cbs.setPrioritizeConflicts(PC);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(bypass);
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_selection_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setNodeLimit(node_limit);
	cbs.setSeed(seed);

	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	int root_g = (int) initial_paths[0].size() - 1 + (int) initial_paths[1].size() - 1;
	int lowerbound = root_g;
	int upperbound = MAX_COST;
	if (g_cardinal_or_pseudo)
		lowerbound += 1;
	cbs.solve(time_limit - runtime, lowerbound, upperbound);
	num_solve_2agent_problems++;
	int rst;
	if (cbs.runtime > time_limit - runtime || cbs.num_HL_expanded > node_limit) // time out or node out
		rst = (int) cbs.min_f_val - root_g; // using lowerbound to approximate
	else if (cbs.solution_cost < 0) // no solution
		rst = MAX_COST;
	else
	{
		rst = cbs.solution_cost - root_g;
	}
	assert(rst >= 0);
	return rst;
}


// Returns a vector of vectors where ret[a1_index][a2_index] == tuple<W,X> if a1 and a2 have
// to increase their combined cost to resolve all conflicts between themselves,
// where X is the expected cost increase for a1 from resolving the current conflict between them
// and W is the expected total cost increase to resolve all conflicts between the agents,
// tuple<0,0> otherwise.
bool NVWEWDGHeuristic::buildGraph(CBSNode& curr, vector<vector<tuple<int,int>>>& WDG, int& num_edges, int& max_edge_weight) {
	bool succ = WDGHeuristic::buildGraph(curr, WDG, num_edges, max_edge_weight);
	if (!succ)
		return false;
	// Add the near-vertex weights
	clock_t t = clock();
	for (const auto& conflict : curr.conflicts)
	{
		if (conflict->priority == conflict_priority::G_CARDINAL ||
			conflict->priority == conflict_priority::F_CARDINAL_G_CARDINAL ||
			conflict->priority == conflict_priority::SEMI_F_CARDINAL_G_CARDINAL ||
			conflict->priority == conflict_priority::PSEUDO_G_CARDINAL_SEMI_G_CARDINAL ||
			conflict->priority == conflict_priority::PSEUDO_G_CARDINAL_NON_G_CARDINAL
			)
		{
			int a1 = conflict->a1;
			int a2 = conflict->a2;
			// No need to set WDG[a1][a2] and WDG[a2][a1] for the simple case - WDGHeuristic::buildGraph already did
			int W = get<0>(WDG[a1][a2]);
			uint64_t cost_increase_a1 = get<1>(WDG[a1][a2]);
			uint64_t cost_increase_a2 = get<1>(WDG[a2][a1]);

			if (conflict->type == conflict_type::TARGET ||
				(!target_reasoning && conflict->type == conflict_type::STANDARD &&
				 get<3>(conflict->constraint1.front()) > conflict->a1_path_cost)
				 ) {  // A g-cardinal target conflict (at least semi-f-cardinal with the MVC heuristic)
				// a1 is the agent that's at its target
				int time_step = get<3>(conflict->constraint1.front());
				cost_increase_a1 = max((uint64_t)(time_step + 1 - conflict->a1_path_cost), cost_increase_a1);
				WDG[a1][a2] = make_tuple(W, cost_increase_a1);
				WDG[a2][a1] = make_tuple(W, 1);
			}
			else if (conflict->type == conflict_type::CORRIDOR)  // Implementing support for reasoning about corridors
				// when corridor reasoning isn't enabled isn't worth the trouble
			{  // A cardinal corridor conflict
				int cost_increase_a1_from_this_conflict = conflict->c1_lookahead - conflict->c1;
				int cost_increase_a2_from_this_conflict = conflict->c2_lookahead - conflict->c2;
				// Not necessary to update max_edge_weight
				if (cost_increase_a1_from_this_conflict + cost_increase_a2_from_this_conflict < cost_increase_a1 + cost_increase_a2)
					continue;  // A previously checked conflict would contribute more. TODO: Allow two sets of constraints from a target conflict and a corridor conflict between the same pair of agents.
				WDG[a1][a2] = make_tuple(W, cost_increase_a1_from_this_conflict);
				WDG[a2][a1] = make_tuple(W, cost_increase_a2_from_this_conflict);
			}
		}
	}
	runtime_build_graph += (double) (clock() - t) / CLOCKS_PER_SEC;
	return true;
}
