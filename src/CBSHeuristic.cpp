#include "CBSHeuristic.h"
#include "CBS.h"
#include <queue>


#define MAX_RUNTIME4PAIR 6000

int CBSHeuristic::computeHeuristics(CBSNode& curr, double time_limit)
{
	if (type == heuristics_type::ZERO)
	{
		return 0;
	}
	// create conflict graph
	start_time = clock();
	this->time_limit = time_limit;
	vector<int> CG(num_of_agents * num_of_agents, 0);
	int num_of_CGedges = 0;
	if (type == heuristics_type::DG || type == heuristics_type::WDG)
	{
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
	}
	else
	{
		for (const auto& conflict : curr.conflicts)
		{
			if (conflict->p == conflict_priority::CARDINAL)
			{
				int a1 = conflict->a1;
				int a2 = conflict->a2;
				if (!CG[a1 * num_of_agents + a2])
				{
					CG[a1 * num_of_agents + a2] = true;
					CG[a2 * num_of_agents + a1] = true;
					num_of_CGedges++;
				}
			}
		}
	}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	
	auto t = clock();
	int rst;
	if (type == heuristics_type::WDG)
	{
		rst = weightedVertexCover(CG);
	}
	else
	{
		// Minimum Vertex Cover
		if (curr.parent == nullptr) // root node of CBS tree
			rst = minimumVertexCover(CG, -1, num_of_agents, num_of_CGedges);
		else
			rst = minimumVertexCover(CG, curr.parent->h_val, num_of_agents, num_of_CGedges);
	}
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;

	return rst;
}


int CBSHeuristic::getEdgeWeight(int a1, int a2, CBSNode& node, bool cardinal)
{
	HTableEntry newEntry(a1, a2, &node);
	if (type != heuristics_type::CG)
	{
		HTable::const_iterator got = lookupTable[a1][a2].find(newEntry);

		if (got != lookupTable[a1][a2].end())
		{
			num_memoization++;
			return got->second;
		}

	}

	int cost_shortestPath = (int)paths[a1]->size() + (int)paths[a2]->size() - 2;
	// runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (screen > 2)
	{
		cout << "Agents " << a1 << " and " << a2 << " in node " << node.time_generated << " : ";
	}
	int rst = 0;
	if (cardinal)
		rst = 1;
	else if (!mutex_reasoning && // no mutex reasoning, so we might miss some cardinal conflicts
		(type == heuristics_type::DG || type == heuristics_type::WDG))
	{
		// get mdds

		const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
		const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
		if (mdd1->levels.size() > mdd2->levels.size()) // swap
		{
			const MDD* temp = mdd1;
			mdd1 = mdd2;
			mdd2 = temp;
		}
		if (!SyncMDDs(*mdd1, *mdd2))
			rst = 1;
		else
			rst = 0;
		num_merge_MDDs++;
	}
	if (type == heuristics_type::WDG && rst > 0)
	{
		vector<SingleAgentSolver*> engines(2);
		engines[0] = search_engines[a1];
		engines[1] = search_engines[a2];
		vector<vector<PathEntry>> initial_paths(2);
		initial_paths[0] = *paths[a1];
		initial_paths[1] = *paths[a2];
		int upperbound = (int)initial_paths[0].size() + (int)initial_paths[1].size() + 10;
		vector<ConstraintTable> constraints{
			ConstraintTable(initial_constraints[a1]),
			ConstraintTable(initial_constraints[a2]) };
		constraints[0].build(node, a1);
		constraints[1].build(node, a2);
		CBS cbs(engines, constraints, initial_paths, upperbound, screen);
		cbs.setPrioritizeConflicts(PC);
		cbs.setHeuristicType(heuristics_type::CG);
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
	}
	lookupTable[a1][a2][newEntry] = rst;
	return rst;
}



bool CBSHeuristic::buildDependenceGraph(CBSNode& node)
{
	// extract all constraints
	/*vector<list<Constraint>> constraints = initial_constraints;
	ICBSNode* curr = &node;
	while (curr != dummy_start)
	{
	if (get<3>(curr->constraints.front()) == constraint_type::LENGTH)
	{
	if (get<0>(curr->constraints.front()) >=0)
	{
	}
	}
	else
	{
	constraints[curr->agent_id].insert(constraints[curr->agent_id].end(),
	curr->constraints.begin(), curr->constraints.end());
	}
	for (const auto& constraint : curr->constraints)
	{
	constraints[curr->agent_id].push_back(constraint);
	}
	curr = curr->parent;
	}

	if (screen == 2)
	{
	for (size_t i = 0; i < constraints.size(); i++)
	{
	if (constraints[i].empty())
	continue;
	std::cout << "Constraints for agent " << i << ":";
	for (auto constraint: constraints[i])
	std::cout << constraint;
	std::cout <<std::endl;
	}
	}*/

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
			if (type == heuristics_type::DG)
			{
				node.conflictGraph[idx] = 1;
			}
			else if (node.conflictGraph.find(idx) == node.conflictGraph.end())
			{
				int w = getEdgeWeight(a1, a2, node, true);
				if (w < 0) // no solution
					return false;

				node.conflictGraph[idx] = w;
			}
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


int CBSHeuristic::minimumVertexCover(const std::vector<int>& CG, int old_mvc, int cols, int num_of_CGedges)
{
	if (num_of_CGedges < 2)
		return num_of_CGedges;
	// Compute #CG nodes that have edges
	int num_of_CGnodes = 0;
	for (int i = 0; i <  cols; i++)
	{
		for (int j = 0; j <  cols; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	if (old_mvc == -1)
	{
		for (int i = 1; i < num_of_CGnodes; i++)
			if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i, cols))
				return i;
        cerr << "ERROR" << endl;
        exit (-1);
	}
	else
	{
		if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols))
			return old_mvc - 1;
		else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols))
			return old_mvc;
		else
			return old_mvc + 1;
	}
}

// Whether there exists a k-vertex cover solution
bool CBSHeuristic::KVertexCover(const std::vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols)
{
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return true; // run out of time
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k)
		return false;

	std::vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < cols - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < cols && flag; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		std::vector<int> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < cols; j++)
		{
			if (CG_copy[node[i] * cols + j] > 0)
			{
				CG_copy[node[i] * cols + j] = 0;
				CG_copy[j * cols + node[i]] = 0;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols))
			return true;
	}
	return false;
}

int CBSHeuristic::greedyMatching(const std::vector<int>& CG,  int cols)
{
	int rst = 0;
	std::vector<bool> used(cols, false);
	while(1)
	{
		int maxWeight = 0;
		int ep1, ep2;
		for (int i = 0; i < cols; i++)
		{
			if(used[i])
				continue;
			for (int j = i + 1; j < cols; j++)
			{
				if (used[j])
					continue;
				else if (maxWeight < CG[i * cols + j])
				{
					maxWeight = CG[i * cols + j];
					ep1 = i;
					ep2 = j;
				}
			}
		}
		if (maxWeight == 0)
			return rst;
		rst += maxWeight;
		used[ep1] = true;
		used[ep2] = true;
	}
}

// branch and bound
// enumerate all possible assignments and return the best one
int CBSHeuristic::weightedVertexCover(const std::vector<int>& CG)
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
int CBSHeuristic::weightedVertexCover(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range, int& best_so_far)
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


void CBSHeuristic::copyConflictGraph(CBSNode& child, const CBSNode& parent)
{
	//copy conflict graph
	if (type == heuristics_type::DG || type == heuristics_type::WDG)
	{
		unordered_set<int> changed;
		for (const auto& p : child.paths) {
			changed.insert(p.first);
		}
		for (auto e : parent.conflictGraph) {
			if (changed.find(e.first / num_of_agents) == changed.end() &&
				changed.find(e.first % num_of_agents) == changed.end())
				child.conflictGraph[e.first] = e.second;
		}

	}
}



bool CBSHeuristic::SyncMDDs(const MDD &mdd, const MDD& other) // assume mdd.levels <= other.levels
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
				//bool validParent = false;
				for (const MDDNode* parentCoexistingNode : (*parent)->coexistingNodesFromOtherMdds)
				{
					for (const MDDNode* childOfParentCoexistingNode : parentCoexistingNode->children)
					{
						if ((*node)->location == childOfParentCoexistingNode->location) // vertex conflict
							continue;
						else if ((*node)->location == parentCoexistingNode->location && (*parent)->location == childOfParentCoexistingNode->location) // edge conflict
							continue;
						//validParent = true;

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
				//if (!validParent)
				//{
				//	// delete the edge, and continue up the levels if necessary
				//	SyncMDDNode* p = *parent;
				//	parent = (*node)->parents.erase(parent);
				//	p->children.remove((*node));
				//	if (p->children.empty())
				//		copy.deleteNode(p);
				//}
				//else
				//{
				//	parent++;
				//}
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
