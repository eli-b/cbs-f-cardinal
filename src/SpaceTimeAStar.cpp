#include "SpaceTimeAStar.h"


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
	path.resize(goal->g_val + 1);
	const LLNode* curr = goal;
	while (curr != nullptr) 
	{
		path[curr->g_val].location = curr->location;
		path[curr->g_val].single = false;
		curr = curr->parent;
	}
}


// find path by time-space A* search
// Returns a shortest path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
Path SpaceTimeAStar::findPath(const CBSNode& node, const ConstraintTable& initial_constraints,
	const vector<Path*>& paths, int agent, int lowerbound)
{
	Path path;
	num_expanded = 0;
	num_generated = 0;

	// build constraint table
	auto t = clock();
	ConstraintTable constraint_table(initial_constraints);
	constraint_table.build(node, agent);
	runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
    if (constraint_table.constrained(start_location, 0))
    {
        return path;
    }

	int holding_time = constraint_table.getHoldingTime();
	t = clock();
	constraint_table.buildCAT(agent, paths, node.makespan + 1);
	runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;
	 // generate start and add it to the OPEN & FOCAL list
	auto start = new AStarNode(start_location, 0, my_heuristic[start_location], nullptr, 0, 0, false);
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table.insert(start);
	min_f_val = (int)start->getFVal();
	lower_bound = max(holding_time, max(min_f_val, max(constraint_table.length_min, lowerbound)));

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto* curr = popNode();

		// check if the popped node is a goal
        if (curr->location == goal_location && 
			curr->timestep >= max(constraint_table.length_min, holding_time) &&
			(curr->parent == nullptr || curr->parent->location != goal_location))
        {
            updatePath(curr, path);
            break;
        }

        if (curr->timestep >= constraint_table.length_max)
            continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			if (max((int)node.makespan + 1, constraint_table.latest_timestep) <= curr->timestep)
            {
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                 constraint_table.constrained(curr->location, next_location, next_timestep))
				 continue;
			
			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			int next_h_val = my_heuristic[next_location];
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;
			int next_internal_conflicts = curr->num_of_conflicts + 
				constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);
				
			// generate (maybe temporary) node
			auto next = new AStarNode(next_location, next_g_val, next_h_val,	
															curr, next_timestep, next_internal_conflicts, false);

			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
            if (it == allNodes_table.end() ||
                (next_location == goal_location && constraint_table.length_min > 0))
			{
				pushNode(next);
				if (it == allNodes_table.end())
					allNodes_table.insert(next); 
				else
					goal_nodes.push_back(next);
				continue;
            } 
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == next->getFVal() &&
					existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
			{  
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
				}
				else
				{
					bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
					bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
					bool update_open = false;
					if ((next_g_val + next_h_val) <= lower_bound)
					{  // if the new f-val qualify to be in FOCAL
						if (existing_next->getFVal() > lower_bound)
							add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
						else
							update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
					}
					if (existing_next->getFVal() > next_g_val + next_h_val)
						update_open = true;

					existing_next->copy(*next);	// update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle);  // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down			
				}
			} 
			
			delete(next);  // not needed anymore -- we already generated it before
		}  // end for loop that generates successors
	}  // end while loop
	  
	releaseNodes();
	return path;
}


inline AStarNode* SpaceTimeAStar::popNode()
{
	auto node = focal_list.top(); focal_list.pop();
	open_list.erase(node->open_handle);
	node->in_openlist = false;
	num_expanded++;
	return node;
}


inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;
	if (node->getFVal() <= lower_bound)
		node->focal_handle = focal_list.push(node);		
}


void SpaceTimeAStar::updateFocalList()
{
	auto open_head = open_list.top();
	if (open_head->getFVal() > min_f_val)
	{
		int new_min_f_val = (int)open_head->getFVal();
		int new_lower_bound = max(lower_bound, new_min_f_val);
		for (auto n : open_list)
		{
			if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
				n->focal_handle = focal_list.push(n);
		}
		min_f_val = new_min_f_val;
		lower_bound = new_lower_bound;
	}
}


void SpaceTimeAStar::releaseNodes()
{
	open_list.clear();
	focal_list.clear();
	for (auto node: allNodes_table)
		delete node;
	allNodes_table.clear();
	for (auto node : goal_nodes)
		delete node;
	goal_nodes.clear();
}

