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

	// Solve the MVC problem
	int mvc_size = minimumVertexCover(G, curr);

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
			// 2+3) constraints that force that if the agent that's at its target doesn't increase its cost enough,
			//      to resolve the current conflict, the other agent has to increase its cost by the whole weight.
			row.insert(a1, 1);
			row.insert(a2, 1);
			mvc_model.addRow(row, weight, mvc_model.getInfinity());
			Constraints[a1][a2].push_back(constraint);
			Constraints[a2][a1].push_back(constraint);
			OsiGrbConstraint constraint2 = mvc_model.getNumRows();
			CoinPackedVector row2;
			int agent_with_increase;
			int agent_without_increase;
			if (a1_increase > 1) {
				agent_with_increase = a1;
				agent_without_increase = a2;
			}
			else {
				agent_with_increase = a2;
				agent_without_increase = a1;
			}
			int cost_increase = a1_increase > 1? a1_increase : a2_increase;
			row2.insert(agent_without_increase, 1);
			row2.insert(num_of_agents + num_of_agents * agent_with_increase + agent_without_increase, -weight);
			mvc_model.addRow(row2, 0, mvc_model.getInfinity());  // x_agent_without_increase >= weight * y_of_agent_pair
			Constraints[a1][a2].push_back(constraint2);
			Constraints[a2][a1].push_back(constraint2);
			OsiGrbConstraint constraint3 = mvc_model.getNumRows();
			CoinPackedVector row3;
			row3.insert(agent_with_increase, 1);
			row3.insert(num_of_agents + num_of_agents * agent_with_increase + agent_without_increase, cost_increase);
			mvc_model.addRow(row3, cost_increase, mvc_model.getInfinity());  // x_agent_with_increase >= cost_increase*(1 - y_of_agent_pair)
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
			// We assume at most one of a1_increase and a2_increase is not 1
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
// HG[i][j] = <x,y>. x is the weight of the edge between i and j, y is the weight near i.
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
		{
			runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
			return h;
		}
		if (NumNontrivialHGEdges == 2)
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
	if (need_aux_variables) {
		for (int i = 0; i < num_of_agents * num_of_agents; ++i)
		{
			mvc_model.addCol(empty, 0, 1, 0);
			mvc_model.setInteger(num_of_agents + i);
		}
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
