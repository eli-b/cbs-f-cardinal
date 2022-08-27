#include <limits>
#include "CBSHeuristic.h"
#include "coin/CoinPackedVector.hpp"
#include <gurobi_c.h>  // For setting some env parameters

int ZeroHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit)
{
  return 0;
}


// Returns false on failure
bool CBSHeuristic::computeInformedHeuristics(CBSNode& curr, double time_limit){
  curr.h_computed =true;
  start_time = clock();
  this->time_limit = time_limit;

  auto gurobi_env = mvc_model.getEnvironmentPtr();  // Note small e in type. This is the C interface
  GRBsetdblparam(gurobi_env, GRB_DBL_PAR_TIMELIMIT, time_limit);  // The time limit is not available when

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
		node.tie_breaking = sizeOfAllConflictsGraphMVC(node);
		break;
  case node_selection::NODE_RANDOM:
    break;
	}
	copyConflictGraph(node, *node.parent);
}

int CBSHeuristic::sizeOfAllConflictsGraphMVC(CBSNode& curr)
{
	auto G = buildConflictGraph(curr);
	auto t = clock();
	shared_ptr<Conflict> dummy_conflict;

	// Temporarily turn off selecting f-cardinal conflicts
	conflict_prioritization current_prioritization = this->PC;
	if (this->PC == conflict_prioritization::BY_F_CARDINAL)
		this->PC = conflict_prioritization::BY_G_CARDINAL;

	// Solve the MVC problem
	int mvc_size = minimumVertexCover(G, curr);

	// Revert the PC change
	this->PC = current_prioritization;

	runtime_solve_MVC -= (double) (clock() - t) / CLOCKS_PER_SEC;
	return mvc_size;
}


// Never to be added back for this node, so not updating the H
void CBSHeuristic::remove_model_constraints(vector<vector<vector<CBSHeuristic::OsiGrbConstraint>>>& Constraints,
											const vector<vector<tuple<int,int>>>& HG, vector<bool>& nodeHasNontrivialEdges) {
	for (int i = 0; i < num_of_agents; ++i) {
		if (!nodeHasNontrivialEdges[i])
			continue;

		// Remove all edges connected to agent i
		for (int other_agent = i + 1; other_agent < num_of_agents; ++other_agent) {
			if (get<0>(HG[i][other_agent])) {
				for (auto constraint : Constraints[i][other_agent]) { // No need to work with references - OsiGrbConstraint objects are just ints
					//spdlog::debug("Removing constraint for agents {} and {}", i, other_agent);
					mvc_model.deleteRows(1, &constraint);
				}
			}
		}
	}
}

void CBSHeuristic::add_constraints_for_heuristic_graph_edge(int a1, int a2, int weight,
															int a1_increase, int a2_increase, int& h,
															vector<int>& HGNodeDegrees,
															vector<vector<vector<OsiGrbConstraint>>>& Constraints,
															int& num_of_nontrivial_HG_edges, vector<bool>& nodeHasNontrivialEdges) {
	if ((HGNodeDegrees[a1] != 1) || (HGNodeDegrees[a2] != 1))
	{


		OsiGrbConstraint constraint = mvc_model.getNumRows();
		CoinPackedVector row;
		if (a1_increase == 1 && a2_increase == 1) // x_agent1 + x_agent2 >= weight
		{
			row.insert(a1, 1);
			row.insert(a2, 1);
			mvc_model.addRow(row, weight, mvc_model.getInfinity());
			Constraints[a1][a2].push_back(constraint);
			Constraints[a2][a1].push_back(constraint);
		}
		else {  // Slightly more complex case. Three constraints are needed: 1) x_agent1 + x_agent2 >= weight
			// 2+3) constraints that force that if an agent doesn't increase its cost enough
			//      to resolve the current conflict, the other agent has to increase its cost by its required cost increase
			//      x_agent1 >= y_1_2 * cost_increase1
			//      x_agent2 >= (1 - y_1_2) * cost_increase2
			row.insert(a1, 1);
			row.insert(a2, 1);
			mvc_model.addRow(row, weight, mvc_model.getInfinity());  // x_agent1 + x_agent2 >= weight
			Constraints[a1][a2].push_back(constraint);
			Constraints[a2][a1].push_back(constraint);
			OsiGrbConstraint constraint2 = mvc_model.getNumRows();
			CoinPackedVector row2;
			row2.insert(a1, 1);
			row2.insert(num_of_agents + num_of_agents * a1 + a2, -a1_increase);
			mvc_model.addRow(row2, 0, mvc_model.getInfinity());  // x_agent1 >= a1_increase * y_of_agent_pair
			Constraints[a1][a2].push_back(constraint2);
			Constraints[a2][a1].push_back(constraint2);
			OsiGrbConstraint constraint3 = mvc_model.getNumRows();
			CoinPackedVector row3;
			row3.insert(a2, 1);
			row3.insert(num_of_agents + num_of_agents * a1 + a2, a2_increase);
			mvc_model.addRow(row3, a2_increase, mvc_model.getInfinity());  // x_agent_with_increase >= cost_increase*(1 - y_of_agent_pair)
			Constraints[a1][a2].push_back(constraint3);
			Constraints[a2][a1].push_back(constraint3);
		}
		num_of_nontrivial_HG_edges++;
		nodeHasNontrivialEdges[a1] = true;
		nodeHasNontrivialEdges[a2] = true;
	}
	else
		h += weight;
}


// And increment h for trivial HG edges
void CBSHeuristic::add_mvc_model_constraints_from_graph(const vector<vector<tuple<int,int>>>& HG,
														vector<int>& HGNodeDegrees, vector<vector<vector<OsiGrbConstraint>>>& Constraints,
														int& num_of_nontrivial_HG_edges, vector<bool>& nodeHasNontrivialEdges,
														vector<int>& lowestCostIncrease, vector<int>& highestCostIncreases,
														int& highestCostIncrease,
														int& h) {
	for (int i = 0; i < num_of_agents ; ++i)
	{
		for (int j = i + 1; j < num_of_agents ; ++j)
		{
			int weight = get<0>(HG[i][j]);
			if (weight == 0)
				continue;
			int a1_increase = get<1>(HG[i][j]);
			if (a1_increase < lowestCostIncrease[i])
				lowestCostIncrease[i] = a1_increase;
			if (a1_increase > highestCostIncreases[i])
			{
				highestCostIncreases[i] = a1_increase;
				if (a1_increase > highestCostIncrease)
					highestCostIncrease = a1_increase;
			}
			int a2_increase = get<1>(HG[j][i]);
			if (a2_increase < lowestCostIncrease[j])
				lowestCostIncrease[j] = a2_increase;
			if (a2_increase > highestCostIncreases[j])
			{
				highestCostIncreases[j] = a2_increase;
				if (a2_increase > highestCostIncrease)
					highestCostIncrease = a2_increase;
			}
			add_constraints_for_heuristic_graph_edge(i, j, weight, a1_increase, a2_increase,
				h, HGNodeDegrees, Constraints, num_of_nontrivial_HG_edges, nodeHasNontrivialEdges);
		}
	}
}


void CBSHeuristic::add_mvc_model_constraints_of_agent(const vector<vector<tuple<int,int>>>& CG, int i,
													  int assume_cost_increased_by,
													  vector<int>& HGNodeDegrees,
													  vector<vector<vector<OsiGrbConstraint>>>& Constraints,
													  int& num_of_nontrivial_HG_edges, vector<bool>& nodeHasNontrivialEdges) {
	for (int j = 0; j < num_of_agents ; ++j)
	{
		if (j == i)
			continue;
		int weight = get<0>(CG[i][j]);
		int a1_increase = get<1>(CG[i][j]);
		int a2_increase = get<1>(CG[j][i]);
		if (a1_increase >= assume_cost_increased_by)  // Cost increased enough to count
		{
			if (weight <= assume_cost_increased_by)
				continue;
			weight -= assume_cost_increased_by;
			if (a1_increase - assume_cost_increased_by >= 1)
				a1_increase -= assume_cost_increased_by;
			else
				a1_increase = 1;
		}
		int pseudo_h = 0;
		add_constraints_for_heuristic_graph_edge(i, j, weight, a1_increase, a2_increase,
												 pseudo_h, HGNodeDegrees, Constraints,
												 num_of_nontrivial_HG_edges, nodeHasNontrivialEdges);
	}
}


void CBSHeuristic::calc_heuristic_graph_vertex_degrees(const vector<vector<tuple<int,int>>>& graph, vector<int>& nodeDegrees) {
	for (int i = 0; i < num_of_agents; ++i)
	{
		for (int j = i + 1; j < num_of_agents; ++j)
		{
			if (get<0>(graph[i][j]) > 0)
			{
				nodeDegrees[i]++;
				nodeDegrees[j]++;
			}
		}
	}
}

// compute a possibly-weighted MVC for the given graph
//// compute a possibly-weighted MVC for the given graph, and also choose a more promising conflict if one is found
// HG[i][j] = <x,y>. x is the weight of the edge between i and j, y is the weight near i.
// TODO: Separate the second part into a new function
int CBSHeuristic::minimumVertexCover(const vector<vector<tuple<int,int>>>& HG, CBSNode& node)
{
	clock_t t = clock();
	int h = 0;
	try
	{
		// Build the cardinal conflict graph
		vector<vector<vector<OsiGrbConstraint>>> Constraints(num_of_agents);  // Sometimes more than one constraint is required for an edge
		vector<int> HGNodeDegrees(num_of_agents, 0);  // Including trivial edges, multiple conflicts between save agents only counted once
		vector<bool> nodeHasNontrivialEdges(num_of_agents, false);
		vector<int> lowestCostIncrease(num_of_agents, std::numeric_limits<int>::max());
		vector<int> highestCostIncreases(num_of_agents, 0);
		int highestCostIncrease = 0;
		int NumNontrivialHGEdges = 0;
		for (int i = 0; i < num_of_agents; i++)
		{
			Constraints[i].resize(num_of_agents);
		}

		calc_heuristic_graph_vertex_degrees(HG, HGNodeDegrees);

		add_mvc_model_constraints_from_graph(HG, HGNodeDegrees, Constraints,
											 NumNontrivialHGEdges,
											 nodeHasNontrivialEdges, lowestCostIncrease, highestCostIncreases,
											 highestCostIncrease, h);

		if (NumNontrivialHGEdges == 0)
		{  // Then there are no f-cardinal conflicts to be found. Resolving a trivial edge will decrease the H exactly by the increase in cost.
			runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
			return h;
		}

		if ((NumNontrivialHGEdges == 2) &&
			// Exactly three vertices and two edges - two edges that share a single vertex.
			// (We already handled trivial edges earlier)
			PC != conflict_prioritization::BY_F_CARDINAL)  // Otherwise we still need to run the model
		{
			// Find the middle agent
            for (int i = 0; i < num_of_agents; i++) {
                if (HGNodeDegrees[i] == 2) {  // That's the middle agent
                	int max_weight = std::numeric_limits<int>::min();
					int max_cost_increase = std::numeric_limits<int>::min();
					for (int j = 0; j < num_of_agents; ++j)
					{
						if (j == i)
							continue;
						if (get<0>(HG[i][j]) > max_weight)
							max_weight = get<0>(HG[i][j]);
						if (get<1>(HG[i][j]) > max_cost_increase)
							max_cost_increase = get<1>(HG[i][j]);
					}
                    if (max_cost_increase == 1) {
                    	h += max_weight;
						remove_model_constraints(Constraints, HG, nodeHasNontrivialEdges);  // For the two edges
						runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
						return h;
                    }
                }
            }
		}

		// Solve LP relaxation
        if (!solved_once)
		{
			mvc_model.initialSolve();
			solved_once = true;
		}
        else
            mvc_model.resolve();
        // Really solve
        mvc_model.branchAndBound();
        // Get the size of the mvc
        int nontrivial_mvc_size = (int) mvc_model.getObjValue();
		h += nontrivial_mvc_size;

		runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
		t = clock();

		if (PC == conflict_prioritization::BY_F_CARDINAL &&    // Look for an f-cardinal conflict among the g-cardinal conflicts,
			NumNontrivialHGEdges != 0)  // there's a chance of finding one (removing trivial edges always decreases the h by exactly their weight).
											  // Although, lookahead may still find f-cardinal conflicts behind trivial
											  // edges, thanks to disjoint splitting, length constraints, or other reasons.
		{
			// Find an edge that increases the cost without decreasing the size of the mvc -
			// splitting on it will increase the F of one of the children, because that node's cost will increase (by 1, usually)
			// and its h will stay the same. That's better than splitting on conflicts where the F of both children stays the same.

			// We could have a larger than 1 cost decrease from removing a vertex. For example, if 10 agents
            // have a conflict with the same agent 3 timesteps after it reaches its goal, removing that agent would decrease
            // the h by 4.
			auto orig_conflict = node.conflict;
            int best_h_decrease = std::numeric_limits<int>::max();  // A lower decrease is better
			bool f_cardinal_found = false;
			for (int i = 0; i < num_of_agents && !f_cardinal_found; ++i)
			{
				if (nodeHasNontrivialEdges[i] == false)  // Trivial edges always increase the h by exactly their weight
					continue;

				// Remove all nontrivial edges connected to agent i (trivial edges do not get into the model in the first place)
				remove_mvc_model_constraints_of_agent(Constraints, i, HGNodeDegrees, nodeHasNontrivialEdges);

				//  Assume the agent increased its cost by the minimum amount that would resolve at least the current conflict.
				//  Add new weaker constraints if necessary.
				add_mvc_model_constraints_of_agent(HG, i, lowestCostIncrease[i], HGNodeDegrees, Constraints,
												   NumNontrivialHGEdges, nodeHasNontrivialEdges);

				// Re-optimize
				mvc_model.resolve();
                mvc_model.branchAndBound();
                int nontrivial_mvc_size_without_the_edges_of_the_node = (int) mvc_model.getObjValue();
				int h_decrease = nontrivial_mvc_size - nontrivial_mvc_size_without_the_edges_of_the_node;
				if (h_decrease < best_h_decrease)
				{
					best_h_decrease = h_decrease;
				}

				if (h_decrease < lowestCostIncrease[i])  // Delta-f > 0
				{
					for (auto& conflict: node.conflicts)
					{
						if (conflict->a1 != i && conflict->a2 != i)
							continue;
						++g_cardinal_conflicts_checked_for_f_cardinality;  // NO! Counting each conflict twice!
						int j;
						if (conflict->a1 != i)
							j = conflict->a1;
						else
							j = conflict->a2;
						if (get<1>(HG[i][j]) > lowestCostIncrease[i])
							continue;  // This conflict has not been resolved by the imagined cost increase
						if (get<1>(HG[j][i]) > 1 && conflict->priority == conflict_priority::G_CARDINAL)
						{   // Our agent is the non-target side of a g-cardinal target conflict - we found a full f-cardinal conflict!
							// Can't improve that, skip adding the edges back - we're going to remove them all anyway.
							f_cardinal_found = true;
							conflict->priority = conflict_priority::F_CARDINAL_G_CARDINAL;
							node.conflict = conflict;
							++f_cardinal_conflicts_found;
							break;
						}
						if (conflict->priority == conflict_priority::G_CARDINAL) {  // But not a target conflict from the other side
							conflict->priority = conflict_priority::SEMI_F_CARDINAL_G_CARDINAL;
							++semi_f_cardinal_g_cardinal_conflicts_found;
							if (highestCostIncrease == 1) {
								// No g-cardinal target conflicts => in particular, the chosen conflict isn't a g-cardinal target conflict.
								// g-cardinal target conflicts are better than semi-f-cardinal conflicts, but none exist, so it's better to
								// switch to a semi-f-cardinal g-cardinal conflict
								// TODO: Just check the chosen conflict directly!
								node.conflict = conflict;
								f_cardinal_found = true;

								// For disjoint splitting: branch on the this agent. Adding a positive constraint on
								// it might increase the cost of other agents besides the conflicting agent and
								// make the conflict fully f-cardinal
								if (conflict->a1 == i)
									node.split_on_which_agent = split_on_agent::FIRST;
								else
									node.split_on_which_agent = split_on_agent::SECOND;
								break;
							}
						}

					}
				}
				else
					g_cardinal_conflicts_checked_for_f_cardinality += HGNodeDegrees[i];  // What about edges that don't come from g-cardinal conflicts

				if (!f_cardinal_found)
				{
					// Remove the weakened edges, if any were added
					remove_mvc_model_constraints_of_agent(Constraints, i, HGNodeDegrees, nodeHasNontrivialEdges);
					// Add all the edges back (since the agent has a nontrivial edge, all its edges are nontrivial so we should add them all)
					add_mvc_model_constraints_of_agent(HG, i, 0, HGNodeDegrees, Constraints,
													   NumNontrivialHGEdges, nodeHasNontrivialEdges);
				}
			}

			if (f_cardinal_found)
			{
//				if (curr.conflict == orig_conflict)
//					spdlog::info("The already chosen conflict {}"
//								 " will increase the F value of at least one of its children", *curr.conflict);
//				else
//					spdlog::info("Switched from conflict {} to conflict {}"
//								 " - it will increase the F value of at least one of its children", *orig_conflict,
//								 *curr.conflict);
			}
			else {
				if (node.conflict->priority > conflict_priority::F_CARDINAL_G_CARDINAL)  // Can improve
				{
					for (auto& conflict: node.conflicts) {
						if (conflict->priority < node.conflict->priority)
						{
							node.conflict = conflict;
//							spdlog::info("Switched from conflict {} to conflict {}"
//								 " - it will increase the F value of at least one of its children", *orig_conflict,
//								 *curr.conflict);
							break;
						}
					}
				}
			}
		}
		runtime_fcardinal_reasoning += (double) (clock() - t) / CLOCKS_PER_SEC;

		remove_model_constraints(Constraints, HG, nodeHasNontrivialEdges);  // In preparation for next call

		double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1;
		return h;
	}
	catch (CoinError& error)
	{
		std::cerr << error.message() << std::endl;
		runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
		throw;
	}
}


// Returns a vector of vectors where ret[a1_index][a2_index] == ret[a2_index][a1_index] == tuple<1,1> if they have at
// least one conflict of any type, tuple<0,0> otherwise
vector<vector<tuple<int,int>>> CBSHeuristic::buildConflictGraph(const CBSNode& curr) const
{
	vector<vector<tuple<int,int>>> G(num_of_agents);
	for (int i = 0 ; i < num_of_agents ; ++i)
		G[i].reserve(num_of_agents);
	for (const auto& conflict : curr.conflicts)
    {
        int a1 = conflict->a1;
        int a2 = conflict->a2;
	    G[a1][a2] = make_tuple(1, 1);
	    G[a2][a1] = make_tuple(1, 1);
    }
	return G;
}

CBSHeuristic::CBSHeuristic(int num_of_agents, const vector<Path*>& paths, vector<SingleAgentSolver*>& search_engines,
						   const vector<ConstraintTable>& initial_constraints, MDDTable& mdd_helper,
						   bool max_vertex_weight_is_1, bool need_aux_variables) :
						   num_of_agents(num_of_agents), paths(paths), search_engines(search_engines),
						   mvc_model(false),  // Use an env that's global, not local to this instance
						   initial_constraints(initial_constraints), mdd_helper(mdd_helper)
{
	try
	{
		auto gurobi_env = mvc_model.getEnvironmentPtr();  // Note small e in type. This is the C interface
		GRBsetintparam(gurobi_env, GRB_INT_PAR_OUTPUTFLAG, 0);  // Turn off logging
		// (note the coin message handler's level should be
		// set to 0 otherwise this is overridden)
		GRBsetintparam(gurobi_env, GRB_INT_PAR_THREADS, 1);  // Use a single thread - we have generally simple models,
		// and there's an overhead to running extra threads.
		// It's also a more fair comparison with older algorithms.
		auto coin_message_handler = mvc_model.messageHandler();
		coin_message_handler->setLogLevel(0);

		// Init the model's columns (variables)
		CoinPackedVector empty;  // Copied from https://github.com/coin-or/Osi/blob/releases/0.108.6/Osi/src/OsiCommonTest/OsiSolverInterfaceTest.cpp:339
		for (int i = 0; i < num_of_agents; ++i)
		{
			if (max_vertex_weight_is_1)
				mvc_model.addCol(empty, 0, 1, 1);
			else
				mvc_model.addCol(empty, 0, mvc_model.getInfinity(), 1);
			mvc_model.setInteger(i);
		}
		// Add num_of_agents^2 aux binary variables after the decision variables. They don't participate in the objective function.
		if (need_aux_variables)
		{
			for (int i = 0; i < num_of_agents * num_of_agents; ++i)
			{
				mvc_model.addCol(empty, 0, 1, 0);
				mvc_model.setInteger(num_of_agents + i);
			}
		}
	}
	catch (CoinError& error)
	{
		std::cerr << error.message() << std::endl;
		throw;
	}
}

void CBSHeuristic::remove_mvc_model_constraints_of_agent(vector<vector<vector<OsiGrbConstraint>>>& Constraints, int i,
														 vector<int>& HGNodeDegrees,
														 vector<bool>& nodeHasNontrivialEdges)
{
	for (int other_agent = 0; other_agent < num_of_agents; ++other_agent)
	{
		if (other_agent == i)
			continue;
		if (nodeHasNontrivialEdges[other_agent] == false)  // Trivial edges always increase the h by exactly their weight
			continue;

		for (auto constraint : Constraints[i][other_agent])
		{ // No need to work with references - GrbConstr objects are just pointers - cheap to copy
			mvc_model.deleteRows(1, &constraint);
		}
		Constraints[i][other_agent].clear();
		Constraints[other_agent][i].clear();
	}
}
