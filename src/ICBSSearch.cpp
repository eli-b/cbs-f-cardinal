#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "ICBSSearch.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"

#define MAX_RUNTIME4PAIR 6000

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr)
{
	for (int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != nullptr)
	{
		for (auto it = curr->paths.begin(); it != curr->paths.end(); ++it)
		{
			if (!updated[it->first])
			{
				paths[it->first] = &(it->second);
				updated[it->first] = true;
			}
		}
		curr = curr->parent;
	}
}
/*
// return the minimal length of the path
int ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id, std::vector <std::list< std::pair<int, int> > >& cons_vec)
{
std::clock_t t1 = std::clock();
// extract all constraints on agent_id
int minLength = 0;
list < Constraint > constraints;
int max_timestep = -1;
while (curr != dummy_start)
{
if (curr->agent_id == agent_id)
{
for (Constraint constraint : curr->constraints)
{
constraints.push_back(constraint);
if (get<2>(constraint) > max_timestep) // calc constraints' max_timestep
max_timestep = get<2>(constraint);
if(get<2>(constraint) > minLength && get<0>(constraint) < 0) // this is a path length constraint
minLength = get<2>(constraint);
else if (get<2>(constraint) >= minLength &&
get<0>(constraint) == paths[agent_id]->back().location && get<1>(constraint) < 0) // or this is a vertex constraint at the goal location
minLength = get<2>(constraint) + 1;
}
}
curr = curr->parent;
}
for (Constraint constraint : initial_constraints[agent_id])
{
constraints.push_back(constraint);
if (get<2>(constraint) > max_timestep) // calc constraints' max_timestep
max_timestep = get<2>(constraint);
if (get<2>(constraint) > minLength && get<0>(constraint) < 0) // this is a path length constraint
minLength = get<2>(constraint);
else if (get<2>(constraint) >= minLength &&
get<0>(constraint) == paths[agent_id]->back().location && get<1>(constraint) < 0) // or this is a vertex constraint at the goal location
minLength = get<2>(constraint) + 1;
}

// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
//std::vector <std::list< std::pair<int, int> > > cons_vec;
cons_vec.resize(max_timestep + 1);

for (auto & constraint : constraints)
{
if (get<0>(constraint) < 0) // barrier constraint
{
int x1 = (-get<0>(constraint) - 1) / ml->cols, y1 = (-get<0>(constraint) - 1) % ml->cols;
int x2 = get<1>(constraint) / ml->cols, y2 = get<1>(constraint) % ml->cols;
if (x1 == x2)
{
if (y1 < y2)
for (int i = 0; i <= y2 - y1; i++)
cons_vec[get<2>(constraint) - i].push_back(make_pair(x1 *  ml->cols + y2 - i, -1));
else
for (int i = 0; i <= y1 - y2; i++)
cons_vec[get<2>(constraint) - i].push_back(make_pair(x1 *  ml->cols + y2 + i, -1));
}
else // y1== y2
{
if (x1 < x2)
for (int i = 0; i <= x2 - x1; i++)
cons_vec[get<2>(constraint) - i].push_back(make_pair((x2 - i) *  ml->cols + y1, -1));
else
for (int i = 0; i <= x1 - x2; i++)
cons_vec[get<2>(constraint) - i].push_back(make_pair((x2 + i) *  ml->cols + y1, -1));
}
}
else
(cons_vec)[get<2>(constraint)].push_back(make_pair(get<0>(constraint), get<1>(constraint)));
}
runtime_updatecons += (double)(std::clock() - t1) * 1000.0 / CLOCKS_PER_SEC;
return minLength;

}
*/


int ICBSSearch::computeHeuristics(ICBSNode& curr)
{
	if (h_type == heuristics_type::NONE)
	{
		return 0;
	}
	// create conflict graph
	clock_t t = std::clock();
	vector<int> CG(num_of_agents * num_of_agents, 0);
	int num_of_CGedges = 0;
	if (h_type == heuristics_type::DG || h_type == heuristics_type::WDG)
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
	runtime_build_dependency_graph += (double)(std::clock() - t) / CLOCKS_PER_SEC;

	t = clock();
	int rst;
	if (h_type == heuristics_type::WDG)
	{
		rst = weightedVertexCover(CG, num_of_agents);
	}
	else
	{
		// Minimum Vertex Cover
		if (curr.parent == nullptr) // root node of CBS tree
			rst = minimumVertexCover(CG, -1, num_of_agents, num_of_CGedges);
		else
			rst = minimumVertexCover(CG, curr.parent->h_val, num_of_agents, num_of_CGedges);
	}
	runtime_solve_MVC += (double)(std::clock() - t) / CLOCKS_PER_SEC;

	return rst;
}

int ICBSSearch::getEdgeWeight(int a1, int a2, ICBSNode& node, bool cardinal)
{
	HTableEntry newEntry(a1, a2, &node);
	if (h_type != heuristics_type::CG)
	{
		HTable::const_iterator got = hTable[a1][a2].find(newEntry);

		if (got != hTable[a1][a2].end())
		{
			num_memoization++;
			return got->second;
		}

	}

	int cost_shortestPath = (int)paths[a1]->size() + (int)paths[a2]->size() - 2;
	runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
	int scr = 0;
	if (screen > 2)
	{
		scr = 2;
		std::cout << "Agents " << a1 << " and " << a2 << " in node " << node.time_generated << " : ";
	}
	int rst = 0;
	if (cardinal)
		rst = 1;
	else if (h_type == heuristics_type::DG || h_type == heuristics_type::WDG)
	{
		// get mdds

		const MDD* mdd1 = getMDD(node, a1);
		const MDD* mdd2 = getMDD(node, a2);

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
	if (h_type == heuristics_type::WDG && rst > 0)
	{
		vector<SingleAgentSolver*> engines(2);
		engines[0] = search_engines[a1];
		engines[1] = search_engines[a2];
		vector<vector<PathEntry>> initial_paths(2);
		initial_paths[0] = *paths[a1];
		initial_paths[1] = *paths[a2];
		double cutoffTime = std::min(MAX_RUNTIME4PAIR * 1.0, time_limit - runtime);
		int upperbound = (int)initial_paths[0].size() + (int)initial_paths[1].size() + 10;
		vector<ConstraintTable> constraints{
			ConstraintTable(initial_constraints[a1]),
			ConstraintTable(initial_constraints[a2])};
		constraints[0].build(node, a1);
		constraints[1].build(node, a2);
		ICBSSearch solver(engines, constraints, initial_paths, 1.0, heuristics_type::CG, true, upperbound, scr);
		solver.disjoint_splitting = disjoint_splitting;
		solver.bypass = false; // I guess that bypassing does not help two-agent path finding???
		solver.rectangle_reasoning = rectangle_reasoning;
		solver.corridor_reasoning = corridor_reasoning;
		solver.target_reasoning = target_reasoning;
		solver.runICBSSearch(cutoffTime, max(rst, 0));
		if (solver.runtime >= cutoffTime) // time out
			rst = (int)solver.min_f_val - cost_shortestPath; // using lowerbound to approximate
		else if (solver.solution_cost  < 0) // no solution
			rst = solver.solution_cost;
		else
			rst = solver.solution_cost - cost_shortestPath;
		num_solve_2agent_problems++;
	}
	hTable[a1][a2][newEntry] = rst;
	return rst;
}

bool ICBSSearch::buildDependenceGraph(ICBSNode& node)
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
		if (conflict->p == conflict_priority::CARDINAL)
		{
			if (h_type == heuristics_type::DG)
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


// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
	std::list<std::shared_ptr<Conflict>>& copy, int excluded_agent) const
{
	for (const auto & conflict : conflicts)
	{
		if (conflict->a1 != excluded_agent && conflict->a2 != excluded_agent)
		{
			copy.push_back(conflict);
		}
	}
}

void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<Conflict >>& conflicts,
	std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agents) const
{
	for (auto conflict : conflicts)
	{
		bool found = false;
		for (auto a : excluded_agents)
		{
			if (conflict->a1 == a || conflict->a2 == a)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			copy.push_back(conflict);
		}
	}
}


void ICBSSearch::findConflicts(ICBSNode& curr, int a1, int a2)
{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (size_t timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2)
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			if (target_reasoning && paths[a1]->size() == timestep + 1)
			{
				conflict->targetConflict(a1, a2, loc1, timestep);
			}
			else if (target_reasoning && paths[a2]->size() == timestep + 1)
			{
				conflict->targetConflict(a2, a1, loc1, timestep);
			}
			else
			{
				conflict->vertexConflict(a1, a2, loc1, timestep);
			}
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict);
		}
		else if (timestep < min_path_length - 1
			&& loc1 == paths[a2]->at(timestep + 1).location
			&& loc2 == paths[a1]->at(timestep + 1).location)
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1);
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict); // edge conflict
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				std::shared_ptr<Conflict> conflict(new Conflict());
				if (target_reasoning)
					conflict->targetConflict(a1_, a2_, loc1, timestep);
				else
					conflict->vertexConflict(a1_, a2_, loc1, timestep);
				assert(!conflict->constraint1.empty());
				assert(!conflict->constraint2.empty());
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict			
			}
		}
	}
}


void ICBSSearch::findConflicts(ICBSNode& curr)
{
	clock_t t = clock();
	if (curr.parent != nullptr)
	{
		// Copy from parent、
		list<int> new_agents;
		for (auto p : curr.paths)
		{
			new_agents.push_back(p.first);
		}
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

		// detect new conflicts
		for (list<int>::iterator it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (a1 == a2)
					continue;
				bool skip = false;
				for (list<int>::iterator it2 = new_agents.begin(); it2 != it; ++it2)
				{
					if (*it2 == a2)
					{
						skip = true;
						break;
					}
				}
				findConflicts(curr, a1, a2);
			}
		}
	}
	else
	{
		for (int a1 = 0; a1 < num_of_agents; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
	curr.num_of_collisions = (int)(curr.unknownConf.size() + curr.conflicts.size());
	runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

MDD * ICBSSearch::getMDD(ICBSNode& node, int id)
{
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		auto got = mddTable[c.a].find(c);
		if (got != mddTable[c.a].end())
		{
			if (got->second->levels.size() != paths[id]->size())
			{
				auto existing_node = got->first;
				auto existing_mdd = got->second;
				cout << *existing_node.n << endl << existing_mdd->levels.size();
				cout << endl;
			}
			assert(got->second->levels.size() == paths[id]->size());
			return got->second;
		}
		releaseMDDMemory(id);
	}

	clock_t t = std::clock();
	MDD * mdd = new MDD();
	ConstraintTable ct(initial_constraints[id]);
	ct.build(node, id);
	mdd->buildMDD(ct, paths[id]->size(), search_engines[id]);
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		mddTable[c.a][c] = mdd;
	}
	runtime_build_MDDs += (double)(clock() - t) / CLOCKS_PER_SEC;
	return mdd;
}


std::shared_ptr<Conflict> ICBSSearch::chooseConflict(const ICBSNode &node) const
{
	if (screen == 3)
		printConflicts(node);
	std::shared_ptr<Conflict> choose;
	if (node.conflicts.empty() && node.unknownConf.empty())
		return nullptr;
	else if (!node.conflicts.empty())
	{
		choose = node.conflicts.back();
		for (const auto& conflict : node.conflicts)
		{
			if (*choose < *conflict) // choose the conflict with higher priority
				choose = conflict;
		}
	}
	else
	{
		choose = node.unknownConf.back();
		for (const auto& conflict : node.unknownConf)
		{
			if (conflict->t < choose->t) // choose the conflict that happens earlier
				choose = conflict;
		}
	}
	return choose;
}



void ICBSSearch::classifyConflicts(ICBSNode &parent)
{
	time_t t = clock();
	// Classify all conflicts in unknownConf
	while (!parent.unknownConf.empty())
	{
		std::shared_ptr<Conflict> con = parent.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int a, loc1, loc2, timestep;
		constraint_type type;
		std::tie(a, loc1, loc2, timestep, type) = con->constraint1.back();
		parent.unknownConf.pop_front();


		bool cardinal1 = false, cardinal2 = false;
		if (timestep >= (int)paths[a1]->size())
			cardinal1 = true;
		else if (!paths[a1]->at(0).single)
		{
			MDD* mdd = getMDD(parent, a1);
			for (size_t i = 0; i < mdd->levels.size(); i++)
				paths[a1]->at(i).single = mdd->levels[i].size() == 1;
			if (mddTable.empty())
				delete mdd;
		}
		if (timestep >= (int)paths[a2]->size())
			cardinal2 = true;
		else if (!paths[a2]->at(0).single)
		{
			MDD* mdd = getMDD(parent, a2);
			for (size_t i = 0; i < mdd->levels.size(); i++)
				paths[a2]->at(i).single = mdd->levels[i].size() == 1;
			if (mddTable.empty())
				delete mdd;
		}

		if (type == constraint_type::EDGE) // Edge conflict
		{
			cardinal1 = paths[a1]->at(timestep).single && paths[a1]->at(timestep - 1).single;
			cardinal2 = paths[a2]->at(timestep).single && paths[a2]->at(timestep - 1).single;
		}
		else // vertex conflict or target conflict
		{
			if (!cardinal1)
				cardinal1 = paths[a1]->at(timestep).single;
			if (!cardinal2)
				cardinal2 = paths[a2]->at(timestep).single;
		}

		if (cardinal1 && cardinal2)
		{
			con->p = conflict_priority::CARDINAL;
		}
		else if (cardinal1 || cardinal2)
		{
			con->p = conflict_priority::SEMI;
		}
		else
		{
			con->p = conflict_priority::NON;
		}

		if (con->p == conflict_priority::CARDINAL && h_type == heuristics_type::NONE)
		{
			parent.conflicts.push_back(con);
			return;
		}

		if (con->type == conflict_type::TARGET)
		{
			parent.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		if (corridor_reasoning)
		{
			auto corridor = corridor_helper.findCorridorConflict(con, paths, cardinal1 && cardinal2, parent);
			if (corridor != nullptr)
			{
				corridor->p = con->p;
				parent.conflicts.push_back(corridor);
				continue;
			}
		}

		if (con->type == conflict_type::STANDARD &&
			((int)paths[con->a1]->size() <= con->t || (int)paths[con->a2]->size() <= con->t)) //conflict happens after agent reaches its goal
		{
			parent.conflicts.push_back(con);
			continue;
		}


		if (rectangle_reasoning // rectangle reasoning
			&& type == constraint_type::VERTEX) // vertex conflict
		{
			auto mdd1 = getMDD(parent, a1);
			auto mdd2 = getMDD(parent, a2);

			auto rectangle = rectangle_helper.findRectangleConflict(paths, timestep, a1, a2, mdd1, mdd2);
			if (rectangle != nullptr)
			{
				parent.conflicts.push_back(rectangle);
				continue;
			}
		}
		parent.conflicts.push_back(con);
	}

	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(parent.conflicts);

	runtime_classify_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
}

void ICBSSearch::removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	unordered_map<int, std::shared_ptr<Conflict> > keep;
	std::list<std::shared_ptr<Conflict>> to_delete;
	for (const auto& conflict : conflicts)
	{
		int a1 = conflict->a1, a2 = conflict->a2;
		int key = a1 * num_of_agents + a2;
		auto p = keep.find(key);
		if (p == keep.end())
		{
			keep[key] = conflict;
		}
		else if (*(p->second) < *conflict)
		{
			to_delete.push_back(p->second);
			keep[key] = conflict;
		}
		else
		{
			to_delete.push_back(conflict);
		}
	}

	for (const auto& conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}

bool ICBSSearch::findPathForSingleAgent(ICBSNode*  node, int ag, int lowerbound)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);
	// find a path
	Path new_path = search_engines[ag]->findPath(*node, initial_constraints[ag], paths, ag, lowerbound);
	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
	runtime_build_CT += search_engines[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines[ag]->runtime_build_CAT;
	runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
	if (!new_path.empty())
	{
		if (isSamePath(*paths[ag], new_path))
		{
			cerr << "Should not find the same path!" << endl;
			exit(-1);
		}
		node->paths.emplace_back(ag, new_path);
		node->g_val = node->g_val - (int)paths[ag]->size() + (int)new_path.size();
		paths[ag] = &node->paths.back().second;
		node->makespan = std::max(node->makespan, new_path.size() - 1);
		return true;
	}
	else
	{
		return false;
	}
}

bool ICBSSearch::generateChild(ICBSNode*  node, ICBSNode* parent)
{
	clock_t t1 = clock();
	node->parent = parent;
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;
	int agent, x, y, t;
	constraint_type type;
	assert(node->constraints.size() > 0);
	tie(agent, x, y, t, type) = node->constraints.front();

	if (type == constraint_type::LEQLENGTH)
	{
		assert(node->constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
			{
				continue;
			}
			for (int i = t; i < (int)paths[ag]->size(); i++)
			{
				if (paths[ag]->at(i).location == x)
				{
					int lowerbound = (int)paths[ag]->size() - 1;
					if (!findPathForSingleAgent(node, ag, lowerbound))
					{
						runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
						return false;
					}
					break;
				}
			}
		}
	}
	else if (type == constraint_type::POSITIVE_VERTEX)
	{
		assert(node->constraints.size() == 1);
		for (const auto& constraint : node->constraints)
		{
			tie(agent, x, y, t, type) = constraint;
			for (int ag = 0; ag < num_of_agents; ag++)
			{
				if (ag == agent)
				{
					continue;
				}
				if (getAgentLocation(ag, t) == x)
				{
					int lowerbound = (int)paths[ag]->size() - 1;
					if (!findPathForSingleAgent(node, ag, lowerbound))
					{
						runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
						return false;
					}
				}
			}
		}

	}
	else if (type == constraint_type::POSITIVE_EDGE)
	{
		assert(node->constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
			{
				continue;
			}
			int curr = getAgentLocation(ag, t);
			int prev = getAgentLocation(ag, t - 1);
			if (prev == x || curr == y ||
				(prev == y && curr == x))
			{
				int lowerbound = (int)paths[ag]->size() - 1;
				if (!findPathForSingleAgent(node, ag, lowerbound))
				{
					runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
					return false;
				}
			}
		}

	}
	else
	{
		int lowerbound;
		if (parent->conflict->t >= (int)paths[agent]->size()) //conflict happens after agent reaches its goal
			lowerbound = parent->conflict->t + 1;
		else if (parent->conflict->p == conflict_priority::CARDINAL && 
			parent->conflict->type != conflict_type::CORRIDOR)
			lowerbound = (int)paths[agent]->size();
		else
			lowerbound = (int)paths[agent]->size() - 1;
		if (!findPathForSingleAgent(node, agent, lowerbound))
		{
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	//Estimate h value
	node->h_val = 0;
	/*if (parent->h_val  == 0);
	else if (parent->conflictGraph.empty())
	{
	node->h_val = parent->h_val - 1; // stronger pathmax
	}
	else
	{
	int maxWeight = 0;
	boost::unordered_map<int, int>::iterator got;
	for (auto e : parent->conflictGraph)
	{
	if ((e.first / num_of_agents == agent || e.first % num_of_agents == agent) && e.second > maxWeight)
	{
	maxWeight = e.second;
	if (maxWeight >= parent->h_val)
	break;
	}
	}
	if (maxWeight < parent->h_val)
	node->h_val = parent->h_val - maxWeight; // stronger pathmax
	}*/
	node->h_val = std::max(node->h_val, parent->f_val - node->g_val); // pathmax
	node->f_val = node->g_val + node->h_val;

	findConflicts(*node);

	copyConflictGraph(*node, *node->parent);

	assert(!node->paths.empty());
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}

inline void ICBSSearch::pushNode(ICBSNode* node)
{
	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);
}

void ICBSSearch::copyConflictGraph(ICBSNode& child, const ICBSNode& parent)
{
	//copy conflict graph
	if (h_type == heuristics_type::DG || h_type == heuristics_type::WDG)
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

void ICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (const auto & t : *paths[i])
			cout << t.location  << "->";
		cout << std::endl;
	}
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList()
{
	ICBSNode* open_head = open_list.top();
	if (open_head->f_val > min_f_val)
	{
		if (screen == 3)
		{
			cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
		}
		min_f_val = open_head->f_val;
		double new_focal_list_threshold = min_f_val * focal_w;
		for (ICBSNode* n : open_list)
		{
			if (n->f_val > focal_list_threshold &&
				n->f_val <= new_focal_list_threshold)
				n->focal_handle = focal_list.push(n);
		}
		focal_list_threshold = new_focal_list_threshold;
		if (screen == 3)
		{
			cout << focal_list.size() << endl;
		}
	}
}


void ICBSSearch::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Optimal,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	std::cout << solution_cost << "," << runtime << "," <<
		HL_num_expanded << "," << LL_num_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
		min_f_val << "," << dummy_start->g_val << "," << dummy_start->f_val << "," <<
		std::endl;
}

void ICBSSearch::saveResults(const std::string &fileName, const std::string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
			"solution cost,min f value,root g value, root f value," <<
			"#adopt bypasses," <<
			"standard conflicts,rectangle conflicts,corridor conflicts,target conflicts," <<
			"#merge MDDs,#solve 2 agents,#memoization," <<
			"runtime of building heuristic graph,runtime of solving MVC," <<
			"runtime of detecting conflicts,runtime of classifying conflicts," <<
			"runtime of building MDDs,runtime of building constraint tables,runtime of building CATs," <<
			"runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,solver name,instance name" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," << HL_num_expanded << "," << HL_num_generated << "," << LL_num_expanded << "," << LL_num_generated << "," <<

		solution_cost << "," << min_f_val << "," << dummy_start->g_val << "," << dummy_start->f_val << "," <<

		num_adopt_bypass << "," <<

		num_standard << "," << num_rectangle << "," << num_corridor << "," << num_target << "," <<

		num_merge_MDDs << "," << num_solve_2agent_problems << "," << num_memoization << "," <<
		runtime_build_dependency_graph << "," << runtime_solve_MVC << "," <<

		runtime_detect_conflicts << "," << runtime_classify_conflicts << "," <<
		runtime_build_MDDs << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << runtime_generate_child << "," <<

		runtime_preprocessing << "," << getSolverName() << "," << instanceName << endl;
	stats.close();
}

void ICBSSearch::printConflicts(const ICBSNode &curr) const
{
	for (const auto& conflict : curr.conflicts)
	{
		std::cout << *conflict << std::endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
		std::cout << *conflict << std::endl;
	}
}


string ICBSSearch::getSolverName() const
{
	string name;
	if (disjoint_splitting)
		name += "Disjoint ";
	switch (h_type)
	{
	case heuristics_type::NONE:
		if (PC)
			name += "ICBS";
		else
			name += "CBS";
		break;
	case heuristics_type::CG:
		name += "CG";
		break;
	case heuristics_type::DG:
		name += "DG";
		break;
	case heuristics_type::WDG:
		name += "WDG";
		break;
	case STRATEGY_COUNT:
		break;
	}
	if (rectangle_reasoning)
		name += "+R";
	if (corridor_reasoning)
		name += "+C";
	if (target_reasoning)
		name += "+T";
	if (bypass)
		name += "+BP";
	name += " with " + search_engines[0]->getName();
	return name;
}

bool ICBSSearch::runICBSSearch(double time_limit, int initial_h)
{
	this->time_limit = time_limit;
	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": ";
	}
	// set timer
	start = std::clock();

	generateRoot(initial_h);

	while (!open_list.empty() && !solution_found)
	{
		updateFocalList();
		if (min_f_val >= cost_upperbound)
		{
			solution_cost = (int)min_f_val;
			solution_found = false;
			break;
		}
		runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
		{  // timeout
			solution_cost = -1;
			solution_found = false;
			break;
		}
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr);

		if (screen > 1)
			std::cout << endl << "Pop " << *curr << endl;

		if (curr->num_of_collisions == 0) //no conflicts
		{// found a solution (and finish the while look)
			solution_found = true;
			solution_cost = curr->g_val;
			goal_node = curr;
			break;
		}

		if (PC) // priortize conflicts
			classifyConflicts(*curr);

		if (!curr->h_computed) // heuristics has not been computed yet
		{
			curr->h_computed = true;
			int h = computeHeuristics(*curr);

			if (h < 0) // no solution, so prune this node
			{
				curr->clear();
				continue;
			}

			curr->h_val = std::max(h, curr->h_val); // use consistent h values
			curr->f_val = curr->g_val + curr->h_val;

			if (screen == 2 && h_type != heuristics_type::NONE)
				curr->printConflictGraph(num_of_agents);

			if (curr->f_val > focal_list_threshold)
			{
				if (screen == 2)
				{
					std::cout << "	Reinsert " << *curr << endl;
				}
				curr->open_handle = open_list.push(curr);
				continue;
			}
		}

		//Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		bool foundBypass = true;
		while (foundBypass)
		{
			foundBypass = false;
			ICBSNode* child[2] = { new ICBSNode() , new ICBSNode() };

			curr->conflict = chooseConflict(*curr);

			if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
			{
				int first = (bool)(rand() % 2);
				if (first) // disjoint splitting on the first agent
				{
					child[0]->constraints = curr->conflict->constraint1;
					int a, x, y, t;
					constraint_type type;
					tie(a, x, y, t, type) = curr->conflict->constraint1.back();
					if (type == constraint_type::VERTEX)
					{
						child[1]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
					}
					else
					{
						assert(type == constraint_type::EDGE);
						child[1]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
					}
				}
				else // disjoint splitting on the second agent
				{
					child[1]->constraints = curr->conflict->constraint2;
					int a, x, y, t;
					constraint_type type;
					tie(a, x, y, t, type) = curr->conflict->constraint2.back();
					if (type == constraint_type::VERTEX)
					{
						child[0]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
					}
					else
					{
						assert(type == constraint_type::EDGE);
						child[0]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
					}
				}
			}
			else
			{
				child[0]->constraints = curr->conflict->constraint1;
				child[1]->constraints = curr->conflict->constraint2;
			}

			if (screen > 1)
				cout << "	Expand " << *curr << endl <<
				"	on " << *(curr->conflict) << endl;

			bool solved[2] = { false, false };
			vector<vector<PathEntry>*> copy(paths);

			for (int i = 0; i < 2; i++)
			{
				if (i > 0)
					paths = copy;
				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
				{
					delete (child[i]);
					continue;
				}
				if (child[i]->f_val == min_f_val && child[i]->num_of_collisions == 0) //no conflicts
				{// found a solution (and finish the while look)
					break;
				}
				else if (bypass && child[i]->g_val == curr->g_val && child[i]->num_of_collisions < curr->num_of_collisions) // Bypass1
				{
					if (i == 1 && !solved[0])
						continue;
					foundBypass = true;
					num_adopt_bypass++;
					curr->conflicts = child[i]->conflicts;
					curr->unknownConf = child[i]->unknownConf;
					curr->num_of_collisions = child[i]->num_of_collisions;
					curr->conflict = nullptr;
					for (const auto& path : child[i]->paths) // update paths
					{
						auto p = curr->paths.begin();
						while (p != curr->paths.end())
						{
							if (path.first == p->first)
							{
								p->second = path.second;
								paths[p->first] = &p->second;
								break;
							}
							++p;
						}
						if (p == curr->paths.end())
						{
							curr->paths.emplace_back(path);
							paths[path.first] = &curr->paths.back().second;
						}
					}
					if (screen > 1)
					{
						std::cout << "	Update " << *curr << endl;
					}
					break;
				}
			}
			if (foundBypass)
			{
				for (int i = 0; i < 2; i++)
				{
					delete (child[i]);
					child[i] = nullptr;
				}
			}
			else
			{
				for (int i = 0; i < 2; i++)
				{
					if (solved[i])
					{
						pushNode(child[i]);
						if (screen > 1)
						{
							std::cout << "		Generate " << *child[i] << endl;
						}
					}
				}
				if (curr->conflict->type == conflict_type::CORRIDOR)
					num_corridor++;
				else if (curr->conflict->type == conflict_type::STANDARD)
					num_standard++;
				else if (curr->conflict->type == conflict_type::RECTANGLE)
					num_rectangle++;
				else if (curr->conflict->type == conflict_type::TARGET)
					num_target++;
				curr->clear();
			}
		}
	}  // end of while loop


	runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
	if (solution_found && !validateSolution())
	{
		std::cout << "Solution invalid!!!" << std::endl;
		printPaths();
		exit(-1);
	}
	if (screen > 0) // 1 or 2
		printResults();
	return solution_found;
}

void ICBSSearch::releaseMDDMemory(int id)
{
	if (id < 0 || mddTable.empty() || (int)mddTable[id].size() < max_num_of_mdds)
		return;
	int minLength = INT_MAX;
	for (auto mdd : mddTable[id])
	{
		if ((int)mdd.second->levels.size() < minLength)
			minLength = mdd.second->levels.size();
	}
	for (MDDTable::iterator mdd = mddTable[id].begin(); mdd != mddTable[id].end();)
	{
		if ((int)mdd->second->levels.size() == minLength)
		{
			delete mdd->second;
			mdd = mddTable[id].erase(mdd);
			// num_released_mdds++;
		}
		else
		{
			mdd++;
		}
	}
}


ICBSSearch::ICBSSearch(vector<SingleAgentSolver*>& search_engines,
	const vector<ConstraintTable>& initial_constraints,
	vector<Path>& paths_found_initially, double f_w,
	heuristics_type h_type, bool PC,
	int cost_upperbound, int screen) :
	PC(PC), screen(screen), h_type(h_type), focal_w(f_w), cost_upperbound(cost_upperbound),
	initial_constraints(initial_constraints), paths_found_initially(paths_found_initially),
	search_engines(search_engines), 
	rectangle_helper(search_engines[0]->instance),
	corridor_helper(search_engines[0]->instance, initial_constraints, search_engines[0]->getName() == "SIPP")
{
	num_of_agents = search_engines.size();
}

ICBSSearch::ICBSSearch(const Instance& instance, double f_w, heuristics_type h_type,
	bool PC, bool sipp, int screen) :
	PC(PC), screen(screen), h_type(h_type), focal_w(f_w),
	num_of_agents(instance.getDefaultNumberOfAgents()),
	rectangle_helper(instance),
	corridor_helper(instance, initial_constraints, sipp)
{
	clock_t t = std::clock();
	initial_constraints.resize(num_of_agents, 
		ConstraintTable(instance.num_of_cols, instance.map_size));

	search_engines = vector < SingleAgentSolver* >(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (sipp)
			search_engines[i] = new SIPP(instance, i);
		else
			search_engines[i] = new SpaceTimeAStar(instance, i);

		initial_constraints[i].goal_location = search_engines[i]->goal_location;
	}
	runtime_preprocessing = (double)(std::clock() - t) / CLOCKS_PER_SEC;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}

bool ICBSSearch::generateRoot(int initial_h)
{
	dummy_start = new ICBSNode();
	dummy_start->g_val = 0;
	paths.resize(num_of_agents, nullptr);
	hTable.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		hTable[i].resize(num_of_agents);
	}

	if (rectangle_reasoning || h_type == heuristics_type::DG || h_type == heuristics_type::WDG)
		mddTable.resize(num_of_agents);
	// initialize paths_found_initially
	if (paths_found_initially.empty())
	{
		paths_found_initially.resize(num_of_agents);

		//generate random permuattion of agent indices
		vector<int> agents(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			agents[i] = i;
		}
		auto rng = std::default_random_engine{};
		std::shuffle(std::begin(agents), std::end(agents), rng);

		for (auto i : agents)
		{
			//CAT cat(dummy_start->makespan + 1);  // initialized to false
			//updateReservationTable(cat, i, *dummy_start);
			paths_found_initially[i] = search_engines[i]->findPath(*dummy_start, initial_constraints[i], paths, i, 0);
			if (paths_found_initially[i].empty())
			{
				cout << "No path exists for agent " << i << endl;
				return false;
			}

			paths[i] = &paths_found_initially[i];
			dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
			dummy_start->g_val += (int)paths_found_initially[i].size() - 1;
			LL_num_expanded += search_engines[i]->num_expanded;
			LL_num_generated += search_engines[i]->num_generated;
		}
	}
	else
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			paths[i] = &paths_found_initially[i];
			dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
			dummy_start->g_val += paths_found_initially[i].size() - 1;
		}
	}

	// generate dummy start and update data structures		
	dummy_start->h_val = initial_h;
	dummy_start->f_val = dummy_start->g_val;

	dummy_start->depth = 0;

	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);

	HL_num_generated++;
	dummy_start->time_generated = HL_num_generated;
	allNodes_table.push_back(dummy_start);
	findConflicts(*dummy_start);

	min_f_val = dummy_start->f_val;
	focal_list_threshold = min_f_val * focal_w;
	//if(h_type == heuristics_type::DG || h_type == heuristics_type::PAIR)
	//	dummy_start->conflictGraph.resize(num_of_agents * num_of_agents, -1);
	/*if (h_type == heuristics_type::DG && !EPEA4PAIR)
	mdds_initially.resize(num_of_agents);*/
	return true;
}

inline void ICBSSearch::releaseClosedListNodes()
{
	for (auto node : allNodes_table)
		delete node;
	allNodes_table.clear();
}

inline void ICBSSearch::releaseMDDTable()
{
	if (!mddTable.empty())
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			for (auto mdd : mddTable[i])
			{
				delete mdd.second;
			}
		}
	}
	mddTable.clear();
}

inline void ICBSSearch::releaseOpenListNodes()
{
	while (!open_list.empty())
	{
		ICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}

ICBSSearch::~ICBSSearch()
{
	releaseClosedListNodes();
	releaseMDDTable();
}

void ICBSSearch::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
}


bool ICBSSearch::validateSolution() const
{
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					std::cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << std::endl;
					return false;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						loc1 << "-->" << loc2 << ") at timestep " << timestep << std::endl;
					return false;
				}
			}
			if (paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						std::cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << std::endl;
						return false; // It's at least a semi conflict			
					}
				}
			}
		}
	}
	return true;
}

inline int ICBSSearch::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
	return paths[agent_id]->at(t).location;
}


// used for rapid random  restart
void ICBSSearch::clear()
{
	releaseClosedListNodes();
	releaseMDDTable();
	open_list.clear();
	focal_list.clear();
	hTable.clear();
	paths.clear();
	paths_found_initially.clear();
	dummy_start = nullptr;
	goal_node = nullptr;
	solution_found = false;
	solution_cost = -2;
}
