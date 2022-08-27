#include <random>      // std::mt19937
#include <algorithm>    // std::shuffle
#include <chrono>       // std::chrono::system_clock
#include <cstdlib>      //  std::rand
#include "CBS.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void CBS::updatePaths(CBSNode* curr)
{
	updatePaths(curr, this->paths);
}

inline void CBS::updatePaths(CBSNode* curr, vector<Path*>& the_paths)
{
	for (int i = 0; i < num_of_agents; i++)
		the_paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != nullptr)
	{
		for (auto it = curr->paths.begin(); it != curr->paths.end(); ++it)
		{
			if (!updated[it->first])
			{
				the_paths[it->first] = &(it->second);
				updated[it->first] = true;
			}
		}
		curr = curr->parent;
	}
}


// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
/*void CBS::copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
	list<shared_ptr<Conflict>>& copy, int excluded_agent) const
{
	for (const auto & conflict : conflicts)
	{
		if (conflict->a1 != excluded_agent && conflict->a2 != excluded_agent)
		{
			copy.push_back(conflict);
		}
	}
}*/

void CBS::copyConflicts(const list<shared_ptr<Conflict >>& conflicts,
						list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agents) const
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


void CBS::findConflicts(CBSNode& curr, int a1, int a2)
{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (size_t timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2)
		{
			shared_ptr<Conflict> conflict = make_shared<Conflict>();
			if (target_reasoning && paths[a1]->size() - 1 == timestep)
			{
				conflict->targetConflict(a1, a2, loc1, timestep, paths[a1]->size() - 1);
			}
			else if (target_reasoning && paths[a2]->size() - 1 == timestep)
			{
				conflict->targetConflict(a2, a1, loc1, timestep, paths[a2]->size() - 1);
			}
			else
			{
				if (paths[a1]->size() - 1 == timestep)
					conflict->vertexConflict(a1, a2, loc1, timestep, paths[a1]->size() - 1);
				else
					conflict->vertexConflict(a2, a1, loc1, timestep, paths[a2]->size() - 1);
			}
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict);
		}
		else if (timestep < min_path_length - 1
				 && loc1 == paths[a2]->at(timestep + 1).location
				 && loc2 == paths[a1]->at(timestep + 1).location)
		{
			shared_ptr<Conflict> conflict = make_shared<Conflict>();
			conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1, paths[a1]->size() - 1);
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict); // edge conflict
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;  // The one with the shorter path
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;  // The one with the longer path
		int loc1 = paths[a1_]->back().location;
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				shared_ptr<Conflict> conflict = make_shared<Conflict>();
				if (target_reasoning)
					conflict->targetConflict(a1_, a2_, loc1, timestep, paths[a1_]->size() - 1);
				else
					conflict->vertexConflict(a1_, a2_, loc1, timestep, paths[a1_]->size() - 1);
				assert(!conflict->constraint1.empty());
				assert(!conflict->constraint2.empty());
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict			
			}
		}
	}
}


void CBS::findConflicts(CBSNode& curr)
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
	// curr.tie_breaking = (int)(curr.unknownConf.size() + curr.conflicts.size());
	runtime_detect_conflicts += (double) (clock() - t) / CLOCKS_PER_SEC;
}


shared_ptr<Conflict> CBS::chooseConflict(const CBSNode& node) const
{
	if (screen == 3)
		printConflicts(node);
	shared_ptr<Conflict> choose;
	if (node.conflicts.empty() && node.unknownConf.empty())
		return nullptr;
	else if (!node.conflicts.empty())
	{
		choose = node.conflicts.back();
		for (const auto& conflict : node.conflicts)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	else
	{
		choose = node.unknownConf.back();
		for (const auto& conflict : node.unknownConf)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	return choose;
}


void CBS::computeSecondaryPriorityForConflict(Conflict& conflict, CBSNode& node)
{
	conflict.secondary_priority = 0;
	switch (conflict_selection_rule)
	{
	case conflict_selection::RANDOM:
		break;
	case conflict_selection::EARLIEST:
		switch (conflict.type)
		{
		case conflict_type::STANDARD:
		case conflict_type::RECTANGLE:
		case conflict_type::TARGET:
		case conflict_type::MUTEX:
			conflict.secondary_priority = get<3>(conflict.constraint1.front());
			break;
		case conflict_type::CORRIDOR:
			conflict.secondary_priority = min(get<2>(conflict.constraint1.front()),
											  get<3>(conflict.constraint1.front()));
			break;
		}
		break;
	case conflict_selection::CONFLICTS:
		for (const auto& c : node.conflicts)
		{
			if (c->a1 == conflict.a1 || c->a2 == conflict.a1 || c->a1 == conflict.a2 || c->a2 == conflict.a2)
				conflict.secondary_priority++;
		}
		for (const auto& c : node.unknownConf)
		{
			if (c->a1 == conflict.a1 || c->a2 == conflict.a1 || c->a1 == conflict.a2 || c->a2 == conflict.a2)
				conflict.secondary_priority++;
		}
		break;
	case conflict_selection::MCONSTRAINTS:
		for (auto curr = &node; curr != nullptr; curr = curr->parent)
		{
			for (const auto& constraint : curr->constraints)
			{
				if (get<0>(constraint) == conflict.a1 || get<0>(constraint) == conflict.a2)
					conflict.secondary_priority--;
			}
		}
		break;
	case conflict_selection::FCONSTRAINTS:
		for (auto curr = &node; curr != nullptr; curr = curr->parent)
		{
			for (const auto& constraint : curr->constraints)
			{
				if (get<0>(constraint) == conflict.a1 || get<0>(constraint) == conflict.a2)
					conflict.secondary_priority++;
			}
		}
		break;
	case conflict_selection::WIDTH:
		conflict.secondary_priority = mdd_helper.getAverageWidth(node, conflict.a1, paths[conflict.a1]->size()) +
									  mdd_helper.getAverageWidth(node, conflict.a2, paths[conflict.a2]->size());
		break;
	case conflict_selection::SINGLETONS:
		break;
	}
	return;
}


void CBS::classifyConflicts(CBSNode& node)
{
	// Classify all conflicts in unknownConf
	while (!node.unknownConf.empty())
	{
		shared_ptr<Conflict> con = node.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int a, loc1, loc2, timestep;
		constraint_type type;
		tie(a, loc1, loc2, timestep, type) = con->constraint1.back();
		node.unknownConf.pop_front();


		bool cardinal1 = false, cardinal2 = false;
		if (timestep >= (int) paths[a1]->size())
			cardinal1 = true;
		else //if (!paths[a1]->at(0).is_single())
		{
			mdd_helper.findSingletons(node, a1, *paths[a1]);
		}
		if (timestep >= (int) paths[a2]->size())
			cardinal2 = true;
		else //if (!paths[a2]->at(0).is_single())
		{
			mdd_helper.findSingletons(node, a2, *paths[a2]);
		}

		if (type == constraint_type::EDGE) // Edge conflict
		{
			cardinal1 = paths[a1]->at(timestep).is_single() && paths[a1]->at(timestep - 1).is_single();
			cardinal2 = paths[a2]->at(timestep).is_single() && paths[a2]->at(timestep - 1).is_single();
		}
		else // vertex conflict or target conflict
		{
			if (!cardinal1)
				cardinal1 = paths[a1]->at(timestep).is_single();
			if (!cardinal2)
				cardinal2 = paths[a2]->at(timestep).is_single();
		}

		if (cardinal1 && cardinal2)
		{
			con->priority = conflict_priority::G_CARDINAL;
		}
		else if (cardinal1 || cardinal2)
		{
			con->priority = conflict_priority::SEMI_G_CARDINAL;
		}
		else
		{
			con->priority = conflict_priority::NON_G_CARDINAL;
		}

		// Mutex reasoning
		if (mutex_helper.strategy != mutex_strategy::N_MUTEX)
		{
			// TODO mutex reasoning is per agent pair, don't do duplicated work...
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());

			auto mutex_conflict = mutex_helper.run(paths, a1, a2, node, mdd1, mdd2);

			if (mutex_conflict != nullptr)
			{
				computeSecondaryPriorityForConflict(*mutex_conflict, node);
				node.conflicts.push_back(mutex_conflict);
				continue;
			}
		}

		// Target Reasoning
		if (con->type == conflict_type::TARGET)
		{
			computeSecondaryPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		if (corridor_helper.use_corridor_reasoning)
		{
			auto corridor_conflict = corridor_helper.run(con, paths, cardinal1 && cardinal2, node);
			if (corridor_conflict != nullptr)
			{
				corridor_conflict->priority = con->priority;
				if (corridor_conflict->priority == conflict_priority::SEMI || corridor_conflict->priority == conflict_priority::NON)
					if (corridor_conflict->c1_lookahead - corridor_conflict->c1 > 0 &&
							corridor_conflict->c2_lookahead - corridor_conflict->c2 > 0)
						corridor_conflict->priority = PSEUDO_CARDINAL;  // We currently only identify (generalized) cardinal corridor conflicts
				computeSecondaryPriorityForConflict(*corridor_conflict, node);
				node.conflicts.push_back(corridor_conflict);
				continue;
			}
		}

		// Rectangle reasoning
		if (rectangle_helper.use_rectangle_reasoning &&
			(int) paths[con->a1]->size() > timestep &&
			(int) paths[con->a2]->size() > timestep && //conflict happens before both agents reach their goal locations
			type == constraint_type::VERTEX && // vertex conflict
			con->priority != conflict_priority::G_CARDINAL) // not caridnal vertex conflict
		{
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
			auto rectangle = rectangle_helper.run(paths, timestep, a1, a2, mdd1, mdd2);
			if (rectangle != nullptr)
			{
				computeSecondaryPriorityForConflict(*rectangle, node);
				node.conflicts.push_back(rectangle);
				continue;
			}
		}

		computeSecondaryPriorityForConflict(*con, node);
		node.conflicts.push_back(con);
	}


	// remove conflicts that cannot be chosen, to save some memory
	// except for the cases when we use num of conflicts as the tie-breaking rule for node selection
	// since we need to know the total number of conflicts in this case.
	if (node_selection_rule != node_selection::NODE_CONFLICTS)
		removeLowPriorityConflicts(node.conflicts);
}

void CBS::removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	unordered_map<int, shared_ptr<Conflict>> keep;
	list<shared_ptr<Conflict>> to_delete;
	for (const auto& conflict : conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2), a2 = max(conflict->a1, conflict->a2);
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

bool CBS::findPathForSingleAgent(CBSNode* node, int ag, int lowerbound)
{
	clock_t t = clock();
	this->cat.removePath(*this->paths[ag]);
	runtime_build_CAT += (double) (clock() - t) / CLOCKS_PER_SEC;

	// find a path
	t = clock();
	Path new_path = search_engines[ag]->findPath(*node, initial_constraints[ag], this->cat, ag, lowerbound);
	runtime_path_finding += (double) (clock() - t) / CLOCKS_PER_SEC;

	t = clock();
	this->cat.addPath(*this->paths[ag], ag);
	runtime_build_CAT += (double) (clock() - t) / CLOCKS_PER_SEC;

	num_LL_expanded += search_engines[ag]->num_expanded;
	num_LL_generated += search_engines[ag]->num_generated;
	runtime_build_CT += search_engines[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines[ag]->runtime_build_CAT;
	if (!new_path.empty())
	{
		assert(!isSamePath(*paths[ag], new_path));
		node->paths.emplace_back(ag, new_path);
		node->g_val = node->g_val - (int) paths[ag]->size() + (int) new_path.size();
		paths[ag] = &node->paths.back().second;
		node->makespan = max(node->makespan, new_path.size() - 1);
		return true;
	}
	else
	{
		return false;
	}
}

bool CBS::generateChild(CBSNode* node, CBSNode* parent)
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
			for (int i = t; i < (int) paths[ag]->size(); i++)
			{
				if (paths[ag]->at(i).location == x)
				{
					int lowerbound = (int) paths[ag]->size() - 1;
					if (!findPathForSingleAgent(node, ag, lowerbound))
					{
						runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
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
					if (!findPathForSingleAgent(node, ag, (int) paths[ag]->size() - 1))
					{
						runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
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
				if (!findPathForSingleAgent(node, ag, (int) paths[ag]->size() - 1))
				{
					runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
					return false;
				}
			}
		}

	}
	else
	{
		int lowerbound = (int) paths[agent]->size() - 1;
		if (!findPathForSingleAgent(node, agent, lowerbound))
		{
			runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	update_delta_g_stats(node);

	assert(!node->paths.empty());
	findConflicts(*node);
	sum_num_conflicts += node->conflicts.size();
	++num_num_conflicts;
	heuristic_helper->computeQuickHeuristics(*node);
	runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
	return true;
}

inline void CBS::pushNode(CBSNode* node)
{
	// update handles
	node->open_handle = open_list.push(node);
	num_HL_generated++;
	node->time_generated = num_HL_generated;
	if (node->g_val + node->h_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);
}


void CBS::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			 paths[i]->size() - 1 << "): ";
		for (const auto& t : *paths[i])
			cout << t.location << "->";
		cout << endl;
	}
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void CBS::updateFocalList()
{
	CBSNode* open_head = open_list.top();
	if (open_head->g_val + open_head->h_val > min_f_val)
	{
		if (screen == 3)
		{
			cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
		}
		min_f_val = open_head->g_val + open_head->h_val;
		double new_focal_list_threshold = min_f_val * focal_w;
		for (CBSNode* n : open_list)
		{
			if (n->g_val + n->h_val > focal_list_threshold &&
				n->g_val + n->h_val <= new_focal_list_threshold)
				n->focal_handle = focal_list.push(n);
		}
		focal_list_threshold = new_focal_list_threshold;
		if (screen == 3)
		{
			cout << focal_list.size() << endl;
		}
	}
}


void CBS::printResults() const
{
	cout << "status,runtime,HL expanded, HL generated,LL expanded,LL generated,min f-val,root g-val,root f-val" << endl;
	if (solution_cost >= 0) // solved
		cout << "Optimal,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << solution_cost << "," << runtime << "," <<
		 num_HL_expanded << "," << num_HL_generated << "," <<
	     num_LL_expanded << "," << num_LL_generated << "," <<
		 min_f_val << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
		 endl;
}

void CBS::saveResults(const string& fileName, const string& instanceName, bool writeHeader) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist && writeHeader)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
				 "solution cost,min f-value,root g-value,root f-value," <<
				 "time limit," <<
				 "#adopt bypasses," <<
				 "standard conflicts,rectangle conflicts,corridor conflicts,target conflicts,mutex conflicts," <<
				 "deltaH_0,deltaH_-1,deltaH_1,sum_deltaH_le_-2,count_deltaH_le_-2,sum_deltaH_ge_2,count_deltaH_ge_2," <<
				 "deltaG_0,deltaG_1,sum_deltaG_ge_2,count_deltaG_ge_2," <<
				 "deltaF_0,deltaF_1,sum_deltaF_ge_2,count_deltaF_ge_2," <<
				 "deltaF_0_deltaG_2+,deltaF_0_deltaG_1,deltaF_0_deltaG_0," <<
				 "deltaF_1_deltaG_3+,deltaF_1_deltaG_2,deltaF_1_deltaG_1,deltaF_1_deltaG_0," <<
				 "sum_num_conflicts,count_num_conflicts," <<
				 "#merge MDDs,#solve 2 agents,#memoization," <<
				 "runtime of building heuristic graph,runtime of solving MVC,runtime of f-cardinal reasoning," <<
				 "f-cardinal conflicts found,semi-f-cardinal g-cardinal conflicts found,g-cardinal conflicts checked for f-cardinality," <<
				 "runtime of detecting conflicts," <<
				 "runtime of rectangle conflicts,runtime of corridor conflicts,runtime of mutex conflicts," <<
				 "runtime of building MDDs,runtime of building constraint tables,runtime of building CATs," <<
				 "runtime of path finding,runtime of generating child nodes," <<
				 "preprocessing runtime,Max Mem (kB),solver name,instance name,number of agents" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," <<
		  num_HL_expanded << "," << num_HL_generated << "," <<
		  num_LL_expanded << "," << num_LL_generated << "," <<
		  solution_cost << "," << min_f_val << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
		  time_limit << "," <<
		  num_adopt_bypass << "," <<
		  num_standard_conflicts << "," << num_rectangle_conflicts << "," << num_corridor_conflicts << "," << num_target_conflicts << "," <<
		  num_mutex_conflicts << "," <<
		  num_delta_h_0 << "," << num_delta_h_minus_1 << "," << num_delta_h_1 << "," <<
		  sum_delta_h_minus_2_or_more << "," << num_delta_h_minus_2_or_more << "," <<
		  sum_delta_h_2_or_more << "," << num_delta_h_2_or_more << "," <<
		  num_delta_g_0 << "," << num_delta_g_1 << "," << sum_delta_g_2_or_more << "," << num_delta_g_2_or_more << "," <<
		  num_delta_f_0 << "," << num_delta_f_1 << "," << sum_delta_f_2_or_more << "," << num_delta_f_2_or_more << "," <<
		  num_delta_f_0_delta_g_2_or_more << "," << num_delta_f_0_delta_g_1 << "," << num_delta_f_0_delta_g_0 << "," <<
		  num_delta_f_1_delta_g_3_or_more << "," << num_delta_f_1_delta_g_2 << "," << num_delta_f_1_delta_g_1 << "," << num_delta_f_1_delta_g_0 << "," <<
		  sum_num_conflicts << "," << num_num_conflicts << "," <<
		  heuristic_helper->num_merge_MDDs << "," <<
		  heuristic_helper->num_solve_2agent_problems << "," <<
		  heuristic_helper->num_memoization_hits << "," <<
		  heuristic_helper->runtime_build_graph << "," <<
		  heuristic_helper->runtime_solve_MVC << "," <<
		  heuristic_helper->runtime_fcardinal_reasoning << "," <<
	      heuristic_helper->f_cardinal_conflicts_found << "," << heuristic_helper->semi_f_cardinal_g_cardinal_conflicts_found << "," <<
		  heuristic_helper->g_cardinal_conflicts_checked_for_f_cardinality << "," <<
		  runtime_detect_conflicts << "," <<
		  rectangle_helper.accumulated_runtime << "," << corridor_helper.accumulated_runtime << "," << mutex_helper.accumulated_runtime << "," <<
		  mdd_helper.accumulated_runtime << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		  runtime_path_finding << "," << runtime_generate_child << "," <<

		  runtime_preprocessing << "," << max_mem << "," << getSolverName() << "," << instanceName << "," << num_of_agents << endl;
	stats.close();
}


void CBS::printConflicts(const CBSNode& curr) const
{
	for (const auto& conflict : curr.conflicts)
	{
		cout << *conflict << endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
		cout << *conflict << endl;
	}
}


string CBS::getSolverName() const
{
	string name;
	if (disjoint_splitting)
		name += "Disjoint ";

	if (PC == conflict_prioritization::OFF)
		name += "CBS";
	else if (PC == conflict_prioritization::BY_G_CARDINAL)
		name += "ICBS";
	else if (PC == conflict_prioritization::BY_F_CARDINAL)
		name += "f-ICBS";

	switch (heuristic_helper->getType())
	{
	case heuristics_type::ZERO:
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
	case heuristics_type::NVWCG:
		name += "NVWCG";
		break;
	case heuristics_type::NVWDG:
		name += "NVWDG";
		break;
	case heuristics_type::NVWEWDG:
		name += "NVWEWDG";
		break;
	case HEURISTICS_COUNT:
		break;
	}
	if (rectangle_helper.use_rectangle_reasoning)
		name += "+R";
	if (corridor_helper.use_corridor_reasoning)
		name += "+C";
	if (target_reasoning)
		name += "+T";
	if (mutex_helper.strategy != mutex_strategy::N_MUTEX)
		name += "+MP";

	if (bypass == bypass_support::G_BYPASS)
		name += "+BP";
	else if (bypass == bypass_support::F_BYPASS)
		name += "+fBP";

	name += " with " + search_engines[0]->getName();
	return name;
}

bool CBS::solve(double time_limit, int cost_lowerbound, int cost_upperbound)
{
	this->min_f_val = cost_lowerbound;
	this->cost_upperbound = cost_upperbound;
	this->time_limit = time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": " << endl;
	}

	// set timer
	start = clock();

	generateRoot();

	CBSNode* prev_node = nullptr;

	while (!open_list.empty() && !solution_found)
	{
		updateFocalList();
		if (min_f_val >= cost_upperbound)
		{
			solution_cost = (int) min_f_val;
			solution_found = false;
			break;
		}
		runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
		if (runtime > time_limit || num_HL_expanded > node_limit)
		{  // time/node out
			solution_cost = -1;
			solution_found = false;
			break;
		}
		CBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr);

		if (screen > 1)
			cout << endl << "Pop " << *curr << endl;

		if (curr->unknownConf.size() + curr->conflicts.size() == 0) // no conflicts
		{  // found a solution (and finish the while look)
			solution_found = true;
			solution_cost = curr->g_val;
			goal_node = curr;
			break;
		}

		if (PC != conflict_prioritization::OFF) // prioritize conflicts
			classifyConflicts(*curr);

		if (!curr->h_computed) // heuristics has not been computed yet
		{
			runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
			bool succ = heuristic_helper->computeInformedHeuristics(*curr, time_limit - runtime);
			runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
			if (runtime > time_limit)
			{  // timeout
				solution_cost = -1;
				solution_found = false;
				break;
			}
			if (!succ) // no solution (a timeout probably occurred), so prune this node
			{
				curr->clear();
				continue;
			}
			update_delta_h_and_delta_f_stats(curr);

			if ((!focal_list.empty() && curr->g_val + curr->h_val > focal_list_threshold) ||  // There's something better in FOCAL
				(focal_list.empty() && !open_list.empty() && CBSNode::compare_node()(curr, open_list.top()))  // FOCAL is empty but there's something better in OPEN
			   )
			{
				// reinsert the node
				curr->open_handle = open_list.push(curr);
				auto open_head = open_list.top();
				if (curr->g_val + curr->h_val <= focal_list_threshold)
					curr->focal_handle = focal_list.push(curr);
				if (screen == 2)
				{
					cout << "	Reinsert " << *curr << endl;
				}
				continue;
			}
		}

		// Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;
		bool foundBypass = false;
		do
		{
			CBSNode* child[2] = { new CBSNode(), new CBSNode() };

			curr->conflict = chooseConflict(*curr);

			if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
			{
				int first;
				if (curr->split_on_which_agent == split_on_agent::UNSET)
					first = (bool) (std::rand() % 2);
				else
					first = curr->split_on_which_agent;
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
			vector<Path*> copy(paths);
			updateCat(prev_node, curr, this->paths, &cat);

			for (int i = 0; i < 2; i++)
			{
				if (i > 0)
					paths = copy;
				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
				{
					delete child[i];
					continue;
				}
				if (child[i]->g_val + child[i]->h_val == min_f_val && curr->unknownConf.size() + curr->conflicts.size() == 0) // no conflicts
				{  // found a solution (and finish the while look)
					break;
				}
				else if (bypass != bypass_support::NONE &&
						 (child[i]->g_val == curr->g_val && child[i]->tie_breaking < curr->tie_breaking)) // g-bypass
				{
					if (i == 1 && !solved[0])
						continue;
					foundBypass = true;
					num_adopt_bypass++;
					curr->conflicts = child[i]->conflicts;
					curr->unknownConf = child[i]->unknownConf;
					curr->tie_breaking = child[i]->tie_breaking;
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
						cout << "	Update " << *curr << endl;
					}
					break;
				}
			}
			if (foundBypass)
			{
				for (int i = 0; i < 2; i++)
				{
					delete child[i];
					child[i] = nullptr;
				}
				if (PC != conflict_prioritization::OFF) // prioritize conflicts
					classifyConflicts(*curr); // classify the new-detected conflicts
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
							cout << "		Generate " << *child[i] << endl;
						}
					}
				}
				switch (curr->conflict->type)
				{
				case conflict_type::RECTANGLE:
					num_rectangle_conflicts++;
					break;
				case conflict_type::CORRIDOR:
					num_corridor_conflicts++;
					break;
				case conflict_type::TARGET:
					num_target_conflicts++;
					break;
				case conflict_type::STANDARD:
					num_standard_conflicts++;
					break;
				case conflict_type::MUTEX:
					num_mutex_conflicts++;
					break;
				}
				curr->clear();
			}
		} while (foundBypass);

		prev_node = curr;
	}  // end of while loop


	runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
	if (solution_found && !validateSolution())
	{
		cout << "Solution invalid!!!" << endl;
		printPaths();
		exit(-1);
	}

	if (screen > 0) // 1 or 2
		printResults();
	return solution_found;
}


CBS::CBS(vector<SingleAgentSolver*>& search_engines,
		 const vector<ConstraintTable>& initial_constraints,
         vector<Path>& paths_found_initially,
         heuristics_type heuristic,
         int screen) :
		screen(screen), focal_w(1),
		initial_constraints(initial_constraints), paths_found_initially(paths_found_initially),
		search_engines(search_engines),
		mdd_helper(initial_constraints, search_engines),
		rectangle_helper(search_engines[0]->instance),
		mutex_helper(search_engines[0]->instance, initial_constraints),
		corridor_helper(search_engines, initial_constraints)
{
	num_of_agents = (int) search_engines.size();
	init_heuristic(heuristic);
	mutex_helper.search_engines = search_engines;
}

CBS::CBS(const Instance& instance, bool sipp, heuristics_type heuristic, int screen) :
		screen(screen), focal_w(1),
		num_of_agents(instance.getDefaultNumberOfAgents()),
		mdd_helper(initial_constraints, search_engines),
		rectangle_helper(instance),
		mutex_helper(instance, initial_constraints),
		corridor_helper(search_engines, initial_constraints)
{
	clock_t t = clock();
	initial_constraints.resize(num_of_agents,
							   ConstraintTable(instance.num_of_cols, instance.map_size));

	search_engines.resize(num_of_agents);

	if (sipp) {
		for (int i = 0; i < num_of_agents; i++)
			search_engines[i] = new SIPP(instance, i);
		cat = new ;
	}
	else {
		for (int i = 0; i < num_of_agents; i++)
			search_engines[i] = new SpaceTimeAStar(instance, i);
		cat = new ConflictAvoidanceTable(instance->get)
	}

	for (int i = 0; i < num_of_agents; i++)
	{
		initial_constraints[i].goal_location = search_engines[i]->goal_location;
	}
	runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

	init_heuristic(heuristic);

	mutex_helper.search_engines = search_engines;

	corridor_helper.calc_alt_cost = CorridorReasoning::calc_alt_cost_variant::NO;
	if (heuristic == heuristics_type::NVWCG || heuristic == heuristics_type::NVWDG || heuristic == heuristics_type::NVWEWDG)
		corridor_helper.calc_alt_cost = CorridorReasoning::calc_alt_cost_variant::YES;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}

void CBS::init_heuristic(heuristics_type heuristic){
  if (heuristic == heuristics_type::ZERO) {
    heuristic_helper = new ZeroHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
										 initial_constraints, mdd_helper);
  } else if (heuristic == heuristics_type::CG) {
    heuristic_helper = new CGHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
									   initial_constraints, mdd_helper);
  } else if (heuristic == heuristics_type::NVWCG) {
	  heuristic_helper = new NVWCGHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
										 	initial_constraints, mdd_helper);
  } else if (heuristic == heuristics_type::DG) {
    heuristic_helper = new DGHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
									   initial_constraints, mdd_helper);
  } else if (heuristic == heuristics_type::NVWDG) {
	  heuristic_helper = new NVWDGHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
										 initial_constraints, mdd_helper);
  } else if (heuristic == heuristics_type::WDG) {
    heuristic_helper = new WDGHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
										initial_constraints, mdd_helper);
  }
  else if (heuristic == heuristics_type::NVWEWDG) {
  	heuristic_helper = new NVWEWDGHeuristic(search_engines[0]->instance.getDefaultNumberOfAgents(), paths, search_engines,
											initial_constraints, mdd_helper);
  }
}

bool CBS::generateRoot()
{
	dummy_start = new CBSNode();
	dummy_start->g_val = 0;
	paths.resize(num_of_agents, nullptr);

	mdd_helper.init(num_of_agents);
	heuristic_helper->init();

	// initialize paths_found_initially
	if (paths_found_initially.empty())
	{
		paths_found_initially.resize(num_of_agents);

		// generate a random permutation of agent indices
		vector<int> agents(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			agents[i] = i;
		}

		if (randomRoot)
		{
			std::mt19937 g(seed);
			std::shuffle(std::begin(agents), std::end(agents), g);
		}

		for (auto i : agents)
		{
			paths_found_initially[i] = search_engines[i]->findPath(*dummy_start, initial_constraints[i], paths, i, 0);
			if (paths_found_initially[i].empty())
			{
				cout << "No path exists for agent " << i << endl;
				return false;
			}
			paths[i] = &paths_found_initially[i];
			dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
			dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
			num_LL_expanded += search_engines[i]->num_expanded;
			num_LL_generated += search_engines[i]->num_generated;
			cat.addPath(paths_found_initially[i], i);
		}
	}
	else
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			paths[i] = &paths_found_initially[i];
			dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
			dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
		}
	}

	// generate dummy start and update data structures		
	dummy_start->h_val = 0;
	dummy_start->depth = 0;
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);

	num_HL_generated++;
	dummy_start->time_generated = num_HL_generated;
	allNodes_table.push_back(dummy_start);
	findConflicts(*dummy_start);
	sum_num_conflicts += dummy_start->conflicts.size();
	++num_num_conflicts;
	// We didn't compute the node-selection tie-breaking value for the root node
	// since it does not need it.
	min_f_val = max(min_f_val, (double) dummy_start->g_val);
	focal_list_threshold = min_f_val * focal_w;

	if (screen >= 2) // print start and goals
	{
		printPaths();
	}

	return true;
}

inline void CBS::releaseNodes()
{
	open_list.clear();
	focal_list.clear();
	for (auto node : allNodes_table)
		delete node;
	allNodes_table.clear();
}


/*inline void CBS::releaseOpenListNodes()
{
	while (!open_list.empty())
	{
		CBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}*/

CBS::~CBS()
{
	releaseNodes();
	mdd_helper.clear();
}

void CBS::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}


bool CBS::validateSolution() const
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
					cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
					return false;
				}
				else if (timestep < min_path_length - 1
						 && loc1 == paths[a2]->at(timestep + 1).location
						 && loc2 == paths[a1]->at(timestep + 1).location)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						 loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
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
						cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
						return false; // It's at least a semi conflict			
					}
				}
			}
		}
	}
	return true;
}

inline int CBS::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t) 0);
	return paths[agent_id]->at(t).location;
}


// used for rapid random  restart
void CBS::clear()
{
	mdd_helper.clear();
	heuristic_helper->clear();
	releaseNodes();
	paths.clear();
	paths_found_initially.clear();
	dummy_start = nullptr;
	goal_node = nullptr;
	solution_found = false;
	solution_cost = -2;
}

void CBS::update_delta_h_and_delta_f_stats(CBSNode* curr)
{
	if (curr->parent != nullptr) {
		int64_t delta_h = curr->h_val - curr->parent->h_val;
		uint64_t delta_f = (curr->g_val + curr->h_val) - (curr->parent->g_val + curr->parent->h_val);

		if (delta_h == 0)
			++num_delta_h_0;
		else if (delta_h == 1)
			++num_delta_h_1;
		else if (delta_h == -1)
			++num_delta_h_minus_1;
		else if (delta_h < -1) {
			sum_delta_h_minus_2_or_more += delta_h;
			++num_delta_h_minus_2_or_more;
		} else {
			sum_delta_h_2_or_more += delta_h;
			++num_delta_h_2_or_more;
		}

		if (delta_f == 0) {
			++num_delta_f_0;
			// delta_h == 1 is impossible because the cost can't decrease
			if (delta_h == 0)
				++num_delta_f_0_delta_g_0;  // We resolved a non-cardinal conflict (or semi-cardinal from the
				// non-cardinal side) and didn't get a new cardinal conflict in the
				// new path (more likely)
			else if (delta_h == -1)
				++num_delta_f_0_delta_g_1;  // We resolved a cardinal conflict from a side that was in the MVC
			else if (delta_h <= -2)
				++num_delta_f_0_delta_g_2_or_more;
		} else if (delta_f == 1) {
			++num_delta_f_1;
			if (delta_h == 1)
				++num_delta_f_1_delta_g_0;  // We resolved a non-cardinal (or semi-cardinal from the
				// non-cardinal side) conflict and got a new cardinal conflict
				// (less likely)
			else if (delta_h == 0)
				++num_delta_f_1_delta_g_1;  // We resolved a semi-cardinal conflict from the cardinal side
				// or a cardinal conflict from a side that wasn't in the MVC
			else if (delta_h == -1)
				++num_delta_f_1_delta_g_2;  // We resolved a cardinal goal conflict from the goal side and the
				// goal side was in the MVC or
				// resolved a cardinal conflict in a disjoint way and got lucky
			else if (delta_h <= -2)
				++num_delta_f_1_delta_g_3_or_more;
		} else {
			sum_delta_f_2_or_more += delta_f;
			++num_delta_f_2_or_more;
		}
	}
}

void CBS::update_delta_g_stats(CBSNode* child)
{
	uint64_t delta_g = child->g_val - child->parent->g_val;
	if (delta_g == 0)
		++num_delta_g_0;
	else if (delta_g == 1)
		++num_delta_g_1;
	else {
		sum_delta_g_2_or_more += delta_g;
		++num_delta_g_2_or_more;
	}
}


// build conflict avoidance table
// update cat: Set cat[time_step][location].vertex or .edge[direction] to the number of other agents that plan to use it
// Assume this->paths contains the paths of the given node
void CBS::buildConflictAvoidanceTable(const CBSNode &node, int exclude_agent, ConflictAvoidanceTable &cat)
{
	if (node.makespan == 0)
		return;
	for (int ag = 0; ag < num_of_agents; ag++)
	{
		if (ag != exclude_agent &&
			this->paths[ag]->size() != 0  // Happens when computing the initial paths for the root node
				)
		{
			cat.addPath(*this->paths[ag], ag);
		}
	}
}


void CBS::findShortestPathFromPrevNodeToCurr(CBSNode *curr, CBSNode* prev,
											 vector<CBSNode *>& steps_up_from_prev_node_to_lowest_common_ancestor,
											 vector<CBSNode *>& steps_down_from_lowest_common_ancestor_to_curr_node) {
	// TODO: Consider implementing the online lowest common ancestor algorithm from https://hackage.haskell.org/package/lca
	std::set<CBSNode *> branch_of_curr;
	CBSNode *node = curr;
	while (node != nullptr) {
		if (node == prev) {  // happy case - node is a direct descendant of prev
			std::reverse(steps_down_from_lowest_common_ancestor_to_curr_node.begin(), steps_down_from_lowest_common_ancestor_to_curr_node.end());
			return;
		}
		steps_down_from_lowest_common_ancestor_to_curr_node.push_back(node);
		branch_of_curr.insert(node);
		node = node->parent;
	}
	node = prev;
	auto end_of_branch_of_curr = branch_of_curr.end();
	while (node != nullptr) {
		if (branch_of_curr.find(node) != end_of_branch_of_curr) {
			while (steps_down_from_lowest_common_ancestor_to_curr_node.back() != node) {
				steps_down_from_lowest_common_ancestor_to_curr_node.pop_back();
			}
			steps_down_from_lowest_common_ancestor_to_curr_node.pop_back();  // this common parent itself shouldn't be a step
			std::reverse(steps_down_from_lowest_common_ancestor_to_curr_node.begin(), steps_down_from_lowest_common_ancestor_to_curr_node.end());
			return;
		}
		steps_up_from_prev_node_to_lowest_common_ancestor.push_back(node);
		node = node->parent;
	}
	cout << "Lowest common ancestor not found!!" << endl;
	std::abort();
}


void CBS::updateCat(CBSNode *prev_node, CBSNode *curr,
					vector<Path*>& paths, ConflictAvoidanceTable *cat)
{
	if (prev_node != nullptr)
	{
		auto lca_jumping_start = std::clock();
		auto wall_lca_jumping_start = std::chrono::system_clock::now();

		vector<CBSNode*> steps_up_from_prev_node_to_lowest_common_ancestor;
		vector<CBSNode*> steps_down_from_lowest_common_ancestor_to_curr_node;
		findShortestPathFromPrevNodeToCurr(curr, prev_node,
										   steps_up_from_prev_node_to_lowest_common_ancestor,
										   steps_down_from_lowest_common_ancestor_to_curr_node);
		if (screen == 3)
			cout << "Updating CAT from #" << prev_node->time_generated << " at depth "<< prev_node->depth << " to #" <<
				    curr->time_generated << " at depth " << curr->depth << " in " <<
				    steps_up_from_prev_node_to_lowest_common_ancestor.size() << " steps up and " <<
					steps_down_from_lowest_common_ancestor_to_curr_node.size()<< " steps down" << endl;

		// Prepare the CAT:
		// for every step up from prev_node to curr (might be empty):
		//   for every path in new_paths that wasn't already deleted from the CAT:
		//	  cat.removePath(path, agent_id);
		// for every step down from prev_node to curr (can't be empty):
		//   for every path in new_paths that wasn't already deleted from the CAT:
		//	  cat.removePath(prev_path, agent_id);
		// for every path we removed:
		//   cat.addPath(curr node's path for the same agent, agent_id)

		// Remove paths from the CAT
		std::set<int> ids_of_agents_whose_paths_we_removed;
		auto end_of_ids_of_agents_whose_paths_we_removed = ids_of_agents_whose_paths_we_removed.end();
		for (auto node : steps_up_from_prev_node_to_lowest_common_ancestor)
		{
			for (auto& agent_id_and_new_path : node->paths)
			{
				if (ids_of_agents_whose_paths_we_removed.find(agent_id_and_new_path.first) ==
					end_of_ids_of_agents_whose_paths_we_removed)
				{  // not already deleted
					cat->removePath(agent_id_and_new_path.second);
					ids_of_agents_whose_paths_we_removed.insert(agent_id_and_new_path.first);
				}
			}
		}
		CBSNode* lowestCommonAncestor = steps_down_from_lowest_common_ancestor_to_curr_node.front()->parent;
		vector<Path*> lca_paths(num_of_agents, nullptr);
		updatePaths(lowestCommonAncestor, lca_paths);
		for (auto node : steps_down_from_lowest_common_ancestor_to_curr_node)
		{
			for (const auto& agent_id_and_new_path : node->paths)
			{
				if (ids_of_agents_whose_paths_we_removed.find(agent_id_and_new_path.first) ==
					end_of_ids_of_agents_whose_paths_we_removed)
				{  // not already deleted
					cat->removePath(*lca_paths[agent_id_and_new_path.first]);
					ids_of_agents_whose_paths_we_removed.insert(agent_id_and_new_path.first);
				}
			}
		}
		// Add the current node's paths to the CAT for each path we removed
		for (auto agent_id : ids_of_agents_whose_paths_we_removed)
		{
			cat->addPath(*paths[agent_id]);
		}
		// The CAT is finally ready
	}
}