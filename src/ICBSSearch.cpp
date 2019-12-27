﻿#include "ICBSSearch.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"

#define MAX_RUNTIME4PAIR 6000

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr)
{
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr->parent != nullptr)
	{
        for (auto it = curr->paths.begin(); it != curr->paths.end() ; ++it)
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
    runtime_build_dependency_graph +=  (double)(std::clock() - t) / CLOCKS_PER_SEC;

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

int ICBSSearch::getEdgeWeight(int a1, int a2, ICBSNode& node, bool cardinal, bool& hit)
{
	HTableEntry newEntry(a1, a2, &node);
	if (h_type != heuristics_type::CG)
	{
		HTable::const_iterator got = hTable[a1][a2].find(newEntry);

		if (got != hTable[a1][a2].end())
		{
			hit = true;
			return got->second;
		}
		
	}
	hit = false;

	int cost_shortestPath = (int)paths[a1]->size() + (int)paths[a2]->size() - 2;
	runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
	int scr = 0;
	if(screen > 2)
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
		if(!SyncMDDs(*mdd1, *mdd2))
			rst = 1;
		else 
			rst = 0;
		num_merge_MDDs++;
	}
	if (h_type == heuristics_type::WDG && rst > 0)
	{
		vector<SingleAgentICBS*> engines(2);
		engines[0] = search_engines[a1];
		engines[1] = search_engines[a2];
		vector<vector<PathEntry>> initial_paths(2);
		initial_paths[0] = *paths[a1];
		initial_paths[1] = *paths[a2];
		double cutoffTime = std::min(MAX_RUNTIME4PAIR * 1.0, time_limit - runtime);
		int upperbound = (int)initial_paths[0].size() + (int)initial_paths[1].size() + 10;
        vector<ConstraintTable> cons(2);
        /*ConstraintTable ct1(initial_constraints[a1]);
		ct1.goal_location = search_engines[a1]->goal_location;
        node.getConstraintTable(ct1, a1, ml->cols, ml->map_size());
        ConstraintTable ct2(initial_constraints[a2]);
		ct2.goal_location = search_engines[a2]->goal_location;
        node.getConstraintTable(ct2, a2, ml->cols, ml->map_size());*/
		cons[0].goal_location = search_engines[a1]->goal_location;
		node.getConstraintTable(cons[0], a1, ml->cols, ml->map_size());
		cons[1].goal_location = search_engines[a2]->goal_location;
		node.getConstraintTable(cons[1], a2, ml->cols, ml->map_size());
		ICBSSearch solver(ml, engines, cons, initial_paths, 1.0, max(rst, 0), heuristics_type::CG, true, upperbound, cutoffTime, scr);

		solver.max_num_of_mdds = this->max_num_of_mdds;
        solver.rectangle_reasoning = rectangle_reasoning;
        solver.corridor_reasoning = corridor_reasoning;
        solver.target_reasoning = target_reasoning;
		solver.runICBSSearch();
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
                bool hit;
                int w = getEdgeWeight(a1, a2, node, true, hit);
                if (w < 0) // no solution
                    return false;

                node.conflictGraph[idx] = w;
            }
        }
	    else
	    {
            if (node.conflictGraph.find(idx) == node.conflictGraph.end())
            {
                bool hit;
                int w = getEdgeWeight(a1, a2, node, false, hit);
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

			curr.unknownConf.push_back(conflict);
		}
		else if (timestep < min_path_length - 1
			&& loc1 == paths[a2]->at(timestep + 1).location
			&& loc2 == paths[a1]->at(timestep + 1).location)
		{
			std::shared_ptr<Conflict> conflict(new Conflict());
			conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1);
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
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict			
			}
		}
	}
}


void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != NULL)
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
}

MDD * ICBSSearch::getMDD(ICBSNode& node, int id)
{
	if(!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
        auto got = mddTable[c.a].find(c);
		if (got != mddTable[c.a].end())
		{
			return got->second;
		}
	}
	MDD * mdd = new MDD();

    ConstraintTable ct(initial_constraints[id]);
    node.getConstraintTable(ct, id, ml->cols, ml->map_size());
	mdd->buildMDD(ct, ml->map_size(), paths[id]->size(),*search_engines[id]);
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		mddTable[c.a][c] = mdd;
	}
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
        int loc1, loc2, timestep;
        constraint_type type;
        std::tie(loc1, loc2, timestep, type) = con->constraint1.back();
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
		} else // vertex conflict or target conflict
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
            auto corridor = findCorridorConflict(con, paths, initial_constraints, cardinal1 && cardinal2, &parent,
                    ml->get_map(), ml->cols, ml->map_size());
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


        if (rectangle_reasoning // rectangle reasoning using MDDs
			&& type == constraint_type::VERTEX) // vertex conflict
		{
            auto mdd1 = getMDD(parent, a1);
            auto mdd2 = getMDD(parent, a2);

            auto rectangle = findRectangleConflict(paths, timestep, ml->cols, ml->map_size(), a1, a2, loc1, mdd1, mdd2);
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
	// extract all constraints on agent ag
	ConstraintTable ct(initial_constraints[ag]);
	node->getConstraintTable(ct, ag, ml->cols, ml->map_size());
	
	// build reservation table
	CAT cat(node->makespan + 1);  // initialized to false
	updateReservationTable(cat, ag, *node);
	// find a path
    vector<PathEntry> newPath;
	bool foundSol = search_engines[ag]->findPath(newPath, ct, cat, lowerbound);
	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;

	if (foundSol)
	{
		if (isSamePath(*paths[ag], newPath))
		{
			cerr << "Should not find the same path!" << endl;
			exit(-1);
		}
        node->paths.emplace_back(ag, newPath);
        node->g_val = node->g_val - (int)paths[ag]->size() + (int)newPath.size();
        paths[ag] = &node->paths.back().second;
        node->makespan = std::max(node->makespan, newPath.size() - 1);
        return true;
	}
	else
	{
		return false;
	}
}

bool ICBSSearch::generateChild(ICBSNode*  node, ICBSNode* parent)
{
	node->parent = parent;
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;

	std::clock_t t1;

	t1 = std::clock();

    if (std::get<0>(node->constraints.front()) >= 0 &&
        std::get<3>(node->constraints.front()) == constraint_type::LENGTH)
    {
        int x, agent, t;
        constraint_type type;
        tie(x, agent, t, type) = node->constraints.front();
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
						return false;
					break;
				}
            }
        }
    }
    else
    {
        int lowerbound;
        if (parent->conflict->t >= (int)paths[node->agent_id]->size()) //conflict happens after agent reaches its goal
            lowerbound = parent->conflict->t + 1;
        else if (parent->conflict->p == conflict_priority::CARDINAL)
            lowerbound = (int)paths[node->agent_id]->size();
        else
            lowerbound = (int)paths[node->agent_id]->size() - 1;
        if (!findPathForSingleAgent(node, node->agent_id, lowerbound))
            return false;
    }
	
	//Estimate h value
	node->h_val = 0;
	if (parent->h_val  == 0);
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
			if ((e.first / num_of_agents == node->agent_id || e.first % num_of_agents == node->agent_id) && e.second > maxWeight)
			{
				maxWeight = e.second;
				if (maxWeight >= parent->h_val)
					break;
			}
		}
		if (maxWeight < parent->h_val)
			node->h_val = parent->h_val - maxWeight; // stronger pathmax
	}
	node->h_val = std::max(node->h_val, parent->f_val - node->g_val); // pathmax
	node->f_val = node->g_val + node->h_val;

	t1 = std::clock();
	findConflicts(*node);
    runtime_detect_conflicts += (double)(std::clock() - t1) / CLOCKS_PER_SEC;
	copyConflictGraph(*node, *node->parent);

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);

	

	return true;
}

void ICBSSearch::copyConflictGraph(ICBSNode& child, const ICBSNode& parent)
{
	//copy conflict graph
	if (h_type == heuristics_type::DG || h_type == heuristics_type::WDG)
	{
	    unordered_set<int> changed;
	    for (const auto& p: child.paths) {
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
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (auto & t : *paths[i])
			std::cout << "(" << t.location / search_engines[0]->num_col << "," <<
				t.location % search_engines[0]->num_col << ")->";
		std::cout << std::endl;
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

void ICBSSearch::updateReservationTable(CAT& cat, int exclude_agent, const ICBSNode &node) 
{
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != nullptr)
		{
			for (size_t timestep = 0; timestep < node.makespan + 1; timestep++) 
			{
				if (timestep >= paths[ag]->size())
				{
					cat[timestep].insert(paths[ag]->back().location);
				}
				else// otherwise, return its location for that timestep
				{
					int id = paths[ag]->at(timestep).location;
					cat[timestep].insert(id);
					if (timestep > 0 && paths[ag]->at(timestep - 1).location != id)
					{
						int prev_id = paths[ag]->at(timestep - 1).location;
						cat[timestep].insert((1 + id) * ml->cols * ml->rows + prev_id);
					}
				}
			}
		}
	}
}

void ICBSSearch::printResults() const
{
	if(solution_cost >= 0) // solved
		cout << "Optimal,";
	else if(solution_cost == -1) // time_out
		cout << "Timeout,";
	else if(solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	std::cout << std::setprecision(2) << runtime << "," <<
		HL_num_expanded  << "," << // HL_num_generated << "," <<
		// LL_num_expanded << "," << LL_num_generated << "," <<
		solution_cost << "," << min_f_val << "," <<
		dummy_start->g_val << "," << dummy_start->f_val << "," <<
		std::endl;
}

void ICBSSearch::saveResults(const std::string &fileName, const std::string &instanceName) const
{
	ofstream stats(fileName, ios::app);
	stats << runtime << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		solution_cost << "," << min_f_val << "," <<
		dummy_start->g_val << "," << dummy_start->f_val << "," <<
        num_merge_MDDs << "," << num_solve_2agent_problems << "," <<
        runtime_build_dependency_graph << "," << runtime_solve_MVC << "," <<
        runtime_detect_conflicts << "," << runtime_classify_conflicts << "," <<
        num_standard << "." << num_rectangle << "," << num_corridor << "," << num_target << "," <<
        getSolverName() << endl;
	stats.close();
}

void ICBSSearch::printConflicts(const ICBSNode &curr) const
{
	for (const auto& conflict: curr.conflicts)
	{
		std::cout << *conflict << std::endl;
	}
	for (const auto& conflict: curr.unknownConf)
	{
        std::cout << *conflict << std::endl;
	}
}


string ICBSSearch::getSolverName() const
{
    string name;
	switch (h_type)
	{
        case heuristics_type::NONE:
            if(PC)
                name = "ICBS";
            else
                name = "CBS";
            break;
        case heuristics_type::CG:
            name = "CG";
            break;
        case heuristics_type::DG:
            name = "DG";
            break;
        case heuristics_type::WDG:
            name = "WDG";
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
	return name;
}

bool ICBSSearch::runICBSSearch() 
{
	if(screen > 0) // 1 or 2
    {
	    string name = getSolverName();
	    name.resize(15, ' ');
        cout << name << ": ";
    }
	// set timer
	start = std::clock();

	// start is already in the open_list
	while (!focal_list.empty() && !solution_found) 
	{
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

		if (curr->num_of_collisions == 0) //no conflicts
		{// found a solution (and finish the while look)
			solution_found = true;
			solution_cost = curr->g_val;
			goal_node = curr;
			break;
		}
		else if (h_type == heuristics_type::NONE) // No heuristics
		{
			if(PC) // priortize conflicts
				classifyConflicts(*curr);
			curr->conflict = chooseConflict(*curr);
		}
		else if(curr->conflict == nullptr) //use h value, and h value has not been computed yet
		{
			if (screen == 3)
			{
				std::cout << std::endl << "****** Compute h for #" << curr->time_generated << " with f= " << curr->g_val <<
					"+" << curr->h_val << " (";
				for (int i = 0; i < num_of_agents; i++)
					std::cout << paths[i]->size() - 1 << ", ";
				std::cout << ") and #conflicts = " << curr->num_of_collisions << std::endl;
			}

			if (PC) // priortize conflicts
				classifyConflicts(*curr);

			int h = computeHeuristics(*curr);

			if (h < 0) // no solution, so prune this node
			{
				curr->clear();
				if (open_list.empty())
				{
					solution_found = false;
					break;
				}
				updateFocalList();
				continue;
			}

			curr->h_val = std::max(h, curr->h_val); // use consistent h values
			curr->f_val = curr->g_val + curr->h_val;

			if(screen == 2)
				curr->printConflictGraph(num_of_agents);

			curr->conflict = chooseConflict(*curr);

			if (curr->f_val > focal_list_threshold)
			{	
				if (screen == 3)
				{
					std::cout << "Reinsert the node with f =" << curr->g_val << "+" << curr->h_val << std::endl;
				}

				curr->open_handle = open_list.push(curr);
				updateFocalList();
				continue;
			}
		}


		 //Expand the node
		HL_num_expanded++;

		curr->time_expanded = HL_num_expanded;
		if(screen > 1)
			std::cout << "Expand Node " << curr->time_generated << " ( " << curr->f_val << "= " << curr->g_val << " + " <<
				curr->h_val << " ) on conflict " << *curr->conflict << std::endl;
		auto n1 = new ICBSNode();
		auto n2 = new ICBSNode();
			
		n1->agent_id = curr->conflict->a1;
		n2->agent_id = curr->conflict->a2;
        n1->constraints = curr->conflict->constraint1;
        n2->constraints = curr->conflict->constraint2;

        if (curr->conflict->type == conflict_type::CORRIDOR)
            num_corridor++;
        else if (curr->conflict->type == conflict_type::STANDARD)
            num_standard++;
        else if (curr->conflict->type == conflict_type::RECTANGLE)
            num_rectangle++;
        else if (curr->conflict->type == conflict_type::TARGET)
            num_target++;

		bool Sol1 = false, Sol2 = false;
		vector<vector<PathEntry>*> copy(paths);
		Sol1 = generateChild(n1, curr);
		if (screen > 1 && Sol1)
		{
			std::cout << "Generate #" << n1->time_generated
				<< " with cost " << n1->g_val
				<< " and " << n1->num_of_collisions << " conflicts " << std::endl;
		}
		if (Sol1 && n1->f_val == min_f_val && n1->num_of_collisions == 0) //no conflicts
		{// found a solution (and finish the while look)
			solution_found = true;
			solution_cost = n1->g_val;
			goal_node = n1;
			break;
		}
		else if(!Sol1)
		{
			delete (n1);
			n1 = nullptr;
		}
		paths = copy;
		Sol2 = generateChild(n2, curr);
		if (screen > 1 && Sol2)
		{
			std::cout << "Generate #" << n2->time_generated
				<< " with cost " << n2->g_val
				<< " and " << n2->num_of_collisions << " conflicts " << std::endl;

		}
		if (Sol2 && n2->f_val == min_f_val && n2->num_of_collisions == 0) //no conflicts
		{// found a solution (and finish the while look)
			solution_found = true;
			solution_cost = n2->g_val;
			goal_node = n2;
			break;
		}
		else if(!Sol2)
		{
			delete (n2);
			n2 = nullptr;
		}

		curr->clear();

		releaseMDDMemory(curr->agent_id);

		if (open_list.empty())
		{
			solution_found = false;
			break;
		}
		updateFocalList();

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


ICBSSearch::ICBSSearch(const MapLoader* ml, vector<SingleAgentICBS*>& search_engines,
        const vector<ConstraintTable>& constraints,
	vector<Path>& paths_found_initially, double f_w, int initial_h,
	heuristics_type h_type, bool PC,
	int cost_upperbound, double time_limit, int screen):
        PC(PC), screen(screen), h_type(h_type), time_limit(time_limit), focal_w(f_w), cost_upperbound(cost_upperbound),
	    initial_constraints(constraints), ml(ml), paths_found_initially(paths_found_initially),
        search_engines(search_engines)
{
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;
	
	num_of_agents = search_engines.size();

	solution_found = false;
	solution_cost = -2;

	// generate dummy start and update data structures	
	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;
	dummy_start->g_val = 0;
	paths.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) 
	{
		paths[i] = &paths_found_initially[i];
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
		dummy_start->g_val += paths_found_initially[i].size() - 1;
	}
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

	if (rectangle_reasoning)
		mddTable.resize(num_of_agents);
}

ICBSSearch::ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, heuristics_type h_type,
	bool PC, double time_limit, int screen):
        PC(PC), screen(screen), h_type(h_type), time_limit(time_limit), focal_w(f_w),
        ml(&ml), num_of_agents(al.num_of_agents)
{
	initial_constraints.resize(num_of_agents);

	search_engines = vector < SingleAgentICBS* >(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) 
	{
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
        initial_constraints[i].goal_location = goal_loc;
		ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.get_moves_offset());
		search_engines[i] = new SingleAgentICBS(init_loc, goal_loc, ml.get_map(), ml.rows*ml.cols,
			ml.get_moves_offset(), ml.cols);
		ch.getHVals(search_engines[i]->my_heuristic);
	}

	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;


	// initialize paths_found_initially
	paths.resize(num_of_agents, nullptr);
	paths_found_initially.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
        ConstraintTable ct;
        CAT cat(dummy_start->makespan + 1);  // initialized to false
		updateReservationTable(cat, i, *dummy_start);

		if (!search_engines[i]->findPath(paths_found_initially[i], ct , cat, 0))
			cout << "NO SOLUTION EXISTS";

		paths[i] = &paths_found_initially[i];
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);

		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
	}



	// generate dummy start and update data structures
	dummy_start->g_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		dummy_start->g_val += (int)paths[i]->size() - 1;
	dummy_start->h_val = 0;
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

	if (screen >= 2) // print start and goals
	{
		al.printAgentsInitGoal();
	}

	hTable.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		hTable[i].resize(num_of_agents);
	}

	if (rectangle_reasoning || h_type == heuristics_type::DG || h_type == heuristics_type::WDG)
		mddTable.resize(num_of_agents);
}

inline void ICBSSearch::releaseClosedListNodes() 
{
	for (auto node: allNodes_table)
		delete node;
}

inline void ICBSSearch::releaseOpenListNodes()
{
	while(!open_list.empty())
	{
		ICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}

ICBSSearch::~ICBSSearch()
{
	releaseClosedListNodes();
	/*if (!mdds_initially.empty())
	{
		for (auto mdd : mdds_initially)
		{
			if (mdd != NULL)
				delete mdd;
		}
	}*/
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
}

void ICBSSearch::clearSearchEngines()
{
	for (auto s: search_engines)
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
					std::cout << "Agents "  << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << std::endl; 
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
