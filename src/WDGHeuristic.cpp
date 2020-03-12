#include "CBSHeuristic.h"
#include "CBS.h"


int WDGHeuristic::computeHeuristics(CBSNode& curr, double time_limit)
{
	// create conflict graph
	start_time = clock();
	this->time_limit = time_limit;
	vector<int> CG(num_of_agents * num_of_agents, 0);
	int num_of_CGedges = 0;
  bool succeed = buildDependenceGraph(curr);
  if (!succeed)
    return -1;
  for (int i = 0; i < num_of_agents; i++)
		{
			for (int j = i + 1; j < num_of_agents; j++)
        {
          auto got = curr.conflictGraph.find(i * num_of_agents + j);
          if (got != curr.conflictGraph.end() && got->second > 0)
            {
              CG[i * num_of_agents + j] = got->second;
              CG[j * num_of_agents + i] = got->second;
              num_of_CGedges++;
            }
        }
		}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	
	auto t = clock();
	int rst;
  rst = weightedVertexCover(CG);
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;

	return rst;
}

// branch and bound
// enumerate all possible assignments and return the best one
int WDGHeuristic::weightedVertexCover(const std::vector<int>& CG)
{
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], CG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}		
				else if (CG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], CG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num  == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(CG[indices[0] * num_of_agents + indices[1]], CG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> x(num);
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(CG[indices[j] * num_of_agents + indices[k]], CG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		int best_so_far = INT_MAX;
		rst += weightedVertexCover(x, 0, 0, G, range, best_so_far);
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}

	//test
	/*std::vector<int> x(N, 0);
	std::vector<int> range(N, 0);
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			range[i] = std::max(range[i], CG[i * N + j]);
			range[j] = std::max(range[j], CG[i * N + j]);
		}
	}
	int best_so_far = INT_MAX;
	int rst2 = weightedVertexCover(x, 0, 0, CG, range, best_so_far);
	if( rst != rst2)
		std::cout << "ERROR" <<std::endl;*/

	return rst;
}

// recusive component of weighted vertex cover
int WDGHeuristic::weightedVertexCover(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range, int& best_so_far)
{
	if (sum >= best_so_far)
		return INT_MAX;
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int)x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = weightedVertexCover(x, i + 1, sum, CG, range, best_so_far);
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
		int rst = weightedVertexCover(x, i + 1, sum + x[i], CG, range, best_so_far);
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

int WDGHeuristic::getEdgeWeight(int a1, int a2, CBSNode& node, bool cardinal)
{
  HTableEntry newEntry(a1, a2, &node);
  HTable::const_iterator got = lookupTable[a1][a2].find(newEntry);

  if (got != lookupTable[a1][a2].end())
		{
			num_memoization++;
			return got->second;
		}


	// runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (screen > 2)
	{
		cout << "Agents " << a1 << " and " << a2 << " in node " << node.time_generated << " : ";
	}
	int rst = 0;
	if (cardinal)
		rst = 1;
	else if (!mutex_reasoning) // no mutex reasoning, so we might miss some cardinal conflicts
	{
    // merging MDDs
    rst = canMergeMDD(a1, a2, node)? 0 : 1;
	}
	if (rst > 0)
	{
    rst = computePairwiseDelta(a1, a2, node);
	}
	lookupTable[a1][a2][newEntry] = rst;
	return rst;
}

int WDGHeuristic::computePairwiseDelta(int a1, int a2, CBSNode& node){
  // Compute the difference between minimum cost and sum of cost in CT nodes.
  int rst;
	int cost_shortestPath = (int)paths[a1]->size() + (int)paths[a2]->size() - 2;
  vector<SingleAgentSolver*> engines(2);
  engines[0] = search_engines[a1];
  engines[1] = search_engines[a2];
  vector<vector<PathEntry>> initial_paths(2);
  initial_paths[0] = *paths[a1];
  initial_paths[1] = *paths[a2];
  int upperbound = (int)initial_paths[0].size() + (int)initial_paths[1].size() + delta_limit;
  vector<ConstraintTable> constraints{
                                      ConstraintTable(initial_constraints[a1]),
                                      ConstraintTable(initial_constraints[a2]) };
  constraints[0].build(node, a1);
  constraints[1].build(node, a2);
  CBS cbs(engines, constraints, initial_paths, upperbound, heuristics_type::CG, screen);
  cbs.setPrioritizeConflicts(PC);
  cbs.setDisjointSplitting(disjoint_splitting);
  cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
  cbs.setRectangleReasoning(rectangle_reasoning);
  cbs.setCorridorReasoning(corridor_reasoning);
  cbs.setTargetReasoning(target_reasoning);
  cbs.setMutexReasoning(mutex_reasoning);
  cbs.setConflictSelectionRule(conflict_seletion_rule);
  cbs.setNodeSelectionRule(node_selection_fule);

  double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
  cbs.solve(time_limit - runtime, max(rst, 0));
  if (cbs.runtime >= time_limit - runtime) // time out
    rst = (int)cbs.min_f_val - cost_shortestPath; // using lowerbound to approximate
  else if (cbs.solution_cost  < 0) // no solution
    rst = cbs.solution_cost;
  else
    rst = cbs.solution_cost - cost_shortestPath;
  num_solve_2agent_problems++;
  return rst;
}

bool WDGHeuristic::buildDependenceGraph(CBSNode& node)
{

	for (const auto& conflict : node.conflicts)
    {
      int a1 = min(conflict->a1, conflict->a2);
      int a2 = max(conflict->a1, conflict->a2);
      int idx = a1 * num_of_agents + a2;
      double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
      if (runtime > time_limit)
        return false; // run out of time
      if (conflict->p == conflict_priority::CARDINAL)
        {
          int w = getEdgeWeight(a1, a2, node, true);
          if (w < 0) // no solution
            return false;
      
          node.conflictGraph[idx] = w;
        }
      else
        {
          if (node.conflictGraph.find(idx) == node.conflictGraph.end())
            {
              int w = getEdgeWeight(a1, a2, node, false);
              if (w < 0) //no solution
                return false;
              node.conflictGraph[idx] = w;
            }
        }
    }
	return true;
}
