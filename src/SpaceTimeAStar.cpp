#include "SpaceTimeAStar.h"


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry>& path)
{
	path.resize(goal->g_val + 1);
	const LLNode* curr = goal;
	while (curr != nullptr) 
	{
		path[curr->g_val].location = curr->location;
		// path[curr->g_val].single = false;
		path[curr->g_val].mdd_width = 0;
		curr = curr->parent;
	}
}


// find path by time-space A* search
// Returns a shortest path that satisfies the constraints of the given node while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
Path SpaceTimeAStar::findPath(const CBSNode& node, const ConstraintTable& initial_constraints,
							  const vector<Path*>& paths, const ConflictAvoidanceTable& cat, int agent, int lowerbound)
{
	num_expanded = 0;
	num_generated = 0;
	// build constraint table
	auto start_time = clock();
	ConstraintTable constraint_table(initial_constraints);
	constraint_table.build(node, agent);
	runtime_build_CT = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	if (constraint_table.length_min >= MAX_TIMESTEP || constraint_table.length_min > constraint_table.length_max ||  // the agent cannot reach
																													 // its goal location
		constraint_table.constrained(start_location, 0)) // the agent cannot stay at its start location
	{
		return Path();
	}

	if (constraint_table.getNumOfLandmarks() > 0)
	{
		// find the new constrained time range, so that, later, we only need to replan the path segment that covers this time range
		int t_min = MAX_TIMESTEP, t_max = 0;
		for (const auto constraint : node.constraints)
		{
			int prev, curr;
			auto [a, x, y, t, type] = constraint;
			switch (type)
			{
			case constraint_type::LEQLENGTH:
				if (agent != a)
				{
					for (int i = t; i < (int)paths[agent]->size(); i++)
					{
						if (paths[agent]->at(i).location == x)
						{
							t_max = MAX_TIMESTEP;
							t_min = min(t_min, i);
							break;
						}
					}
				}
				break;
			case constraint_type::POSITIVE_VERTEX:
				if (agent != a)
				{
					if ((paths[agent]->size() > t && paths[agent]->at(t).location == x) || paths[agent]->back().location == x)
					{
						t_max = max(t_max, t);
						t_min = min(t_min, t);
					}
				}
				break;
			case constraint_type::POSITIVE_EDGE:
				if (agent != a)
				{
					curr = (paths[agent]->size() <= t) ? paths[agent]->back().location : paths[agent]->at(t).location;
					prev = (paths[agent]->size() <= t - 1) ? paths[agent]->back().location : paths[agent]->at(t - 1).location;
					if (prev == x || curr == y || (prev == y && curr == x))
					{
						t_max = max(t_max, t);
						t_min = min(t_min, t - 1);
					}
				}
				break;
			case constraint_type::GLENGTH:
				if (agent == a)
				{
					t_max = MAX_TIMESTEP;
				}
				break;
			case constraint_type::BARRIER:
				if (agent == a)
				{
					auto states = constraint_table.decodeBarrier(x, y, t);
					for (const auto& state : states)
					{
						curr = (paths[agent]->size() <= state.second) ? paths[agent]->back().location : paths[agent]->at(state.second).location;
						if (curr == state.first)
						{
							t_max = max(t_max, state.second);
							t_min = min(t_min, state.second);
						}
					}
				}
				break;
			case constraint_type::RANGE_VERTEX:
				if (agent == a)
				{
					for (int i = y; i <= t; i++)
					{
						curr = (paths[agent]->size() <= i) ? paths[agent]->back().location : paths[agent]->at(i).location;
						if (curr == x)
						{
							t_max = max(t_max, i);
							t_min = min(t_min, i);
						}
					}
				}
				break;
			case constraint_type::RANGE_EDGE:
				if (agent == a)
				{
					for (int i = 1; i <= t; i++)
					{
						prev = (paths[agent]->size() <= i - 1) ? paths[agent]->back().location : paths[agent]->at(i - 1).location;
						curr = (paths[agent]->size() <= i) ? paths[agent]->back().location : paths[agent]->at(i).location;
						if (curr == y && prev == x)
						{
							t_max = max(t_max, i);
							t_min = min(t_min, i);
						}
					}
				}
				break;
			case constraint_type::VERTEX:
				if (agent == a)
				{
					curr = (paths[agent]->size() <= t) ? paths[agent]->back().location : paths[agent]->at(t).location;
					if (curr == x)
					{
						t_max = max(t_max, t);
						t_min = min(t_min, t);
					}
				}
				break;
			case constraint_type::EDGE:
				if (agent == a)
				{
					curr = (paths[agent]->size() <= t) ? paths[agent]->back().location : paths[agent]->at(t).location;
					prev = (paths[agent]->size() <= t - 1) ? paths[agent]->back().location : paths[agent]->at(t - 1).location;
					if (prev == x && curr == y)
					{
						t_max = max(t_max, t);
						t_min = min(t_min, t - 1);
					}
				}
				break;
			}
		}
		assert(t_min <= t_max);
		pair<int, int> start_state(start_location, 0), goal_state(goal_location, MAX_TIMESTEP);
		for (const auto landmark : constraint_table.getLandmarks())
		{
			if (start_state.second < landmark.first&& landmark.first < t_min)
			{
				start_state = make_pair(landmark.second, landmark.first);
			}
			else if (t_max < landmark.first && landmark.first < goal_state.second)
			{
				goal_state = make_pair(landmark.second, landmark.first);
			}
		}
		if (goal_state.second == MAX_TIMESTEP)
		{
			// debug
			// auto test_path = findShortestPath(constraint_table, cat, make_pair(start_location, 0), lowerbound);

			auto path_segment = findShortestPath(constraint_table, cat, start_state, lowerbound - start_state.second);
			if (path_segment.empty())
			{
				// assert(test_path.empty());
				return Path();
			}

			if (start_state.second == 0)
				return path_segment;
			Path path;
			path.insert(path.begin(), paths[agent]->begin(), paths[agent]->begin() + start_state.second);
			path.insert(path.end(), path_segment.begin(), path_segment.end());

			//debug 
			// assert(path.size() == test_path.size());
			return path;
		}
		else
		{
			// debug
			// auto test_path = findShortestPath(constraint_table, cat, make_pair(start_location, 0), lowerbound);

			auto path_segment = findPath(constraint_table, cat, start_state, goal_state);
			
			if (path_segment.empty())
			{
				// assert(test_path.empty());
				return Path();
			}
			// assert(test_path.size() == paths[agent]->size());
			Path path(*paths[agent]);
			assert(path_segment.size() == 1 + goal_state.second - start_state.second &&
						path[start_state.second].location == path_segment.front().location &&
						path[goal_state.second].location == path_segment.back().location);
			for (int t = 1; t < (int)path_segment.size() - 1; t++)
			{
				path[start_state.second + t] = path_segment[t];
			}
			return path;
		}
	}
	else
		return findShortestPath(constraint_table, cat, make_pair(start_location, 0), lowerbound);
}


Path SpaceTimeAStar::findShortestPath(ConstraintTable& constraint_table, const ConflictAvoidanceTable& cat,
									  const pair<int, int> start_state, int lowerbound)
{
	// generate start and add it to the OPEN & FOCAL list
	Path path;
	auto start = new AStarNode(start_state.first,  // location
													0,  // g val 
													my_heuristic[start_state.first],  // h val
													nullptr,  // parent
													start_state.second,  // timestep
													0, false);

	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table.insert(start);
	min_f_val = (int) start->getFVal();
	int holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
	lower_bound = max(holding_time - start_state.second, max(min_f_val, lowerbound));

	// Note: Even if constraint_table.length_max is less than infinity, it doesn't mean any path length of length less
	//       than or equal to that is allowed. It's still required to be minimal. The LEQLENGTH constraint could have
	//       come from a conflict between an agent staying at its goal long after it has reached it, and another agent.
	//       The agent staying at its goal is not suddently allowed to increase its cost.

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto* curr = popNode();

		// check if the popped node is a goal
		if (curr->location == goal_location && // arrive at the goal location
			!curr->wait_at_goal && // not wait at the goal location
			curr->timestep >= holding_time) // the agent can hold the goal location afterward
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
			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
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
										  cat.num_conflicts_for_step(curr->location, next_location, next_timestep);


			// generate (maybe temporary) node
			auto next = new AStarNode(next_location, next_g_val, next_h_val,
									  curr, next_timestep, next_internal_conflicts, false);
			if (next_location == goal_location && curr->location == goal_location)
				next->wait_at_goal = true;

			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				pushNode(next);
				allNodes_table.insert(next);
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

					existing_next->copy(*next);  // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle);  // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down			
				}
			}
			delete next;  // not needed anymore -- we already generated it before
		}  // end for loop that generates successors
	}  // end while loop

	releaseNodes();
	return path;
}


Path SpaceTimeAStar::findPath(ConstraintTable& constraint_table, const ConflictAvoidanceTable& cat,
							  const pair<int, int> start_state, const pair<int, int> goal_state)
{
	// generate start and add it to the OPEN & FOCAL list
	Path path;
	auto start = new AStarNode(start_state.first,  // location
		0,  // g val 
		compute_heuristic(start_state.first, goal_state.first),  // h val
		nullptr,  // parent
		start_state.second,  // timestep
		0, false);
	if (start->timestep + start->h_val > goal_state.second)
		return path;
	num_generated++;
	start->focal_handle = focal_list.push(start);
	allNodes_table.insert(start);
	// min_f_val = (int)start->getFVal();

	while (!focal_list.empty())
	{
		auto* curr = focal_list.top(); 
		focal_list.pop();
		curr->in_openlist = false;

		// check if the popped node is a goal
		if (curr->location == goal_state.first && // arrive at the goal location
			curr->timestep == goal_state.second) // at the corresponding timestep
		{
			updatePath(curr, path);
			break;
		}

		num_expanded++;
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;

			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			int next_h_val = compute_heuristic(next_location, goal_state.first);
			if (next_timestep + next_h_val > goal_state.second)
				continue;
			int next_internal_conflicts = curr->num_of_conflicts +
										  cat.num_conflicts_for_step(curr->location, next_location, next_timestep);


			// generate (maybe temporary) node
			auto next = new AStarNode(next_location, next_g_val, next_h_val,
				curr, next_timestep, next_internal_conflicts, false);
			
			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				num_generated++;
				next->focal_handle = focal_list.push(next);
				next->in_openlist = true;
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->num_of_conflicts > next->num_of_conflicts) // if there's fewer conflicts
			{
				existing_next->copy(*next);	// update existing node
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					next->focal_handle = focal_list.push(existing_next);
					existing_next->in_openlist = true;
				}
				else
				{
					focal_list.update(existing_next->focal_handle);		
				}
			}
			delete next;  // not needed anymore -- we already generated it before
		}  // end for loop that generates successors
	}  // end while loop

	releaseNodes();
	return path;
}


// <upper_bound> is non-inclusive. Travel time must be less.
int SpaceTimeAStar::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
	if (constraint_table.length_min >= MAX_TIMESTEP ||
		constraint_table.length_min > constraint_table.length_max ||  // the agent cannot reach its goal location
		constraint_table.constrained(start, 0))  // the agent cannot stay at its start location
	{
		return MAX_TIMESTEP;
	}
	int length = MAX_TIMESTEP;
	auto root = new AStarNode(start, 0, compute_heuristic(start, end), nullptr, 0);
	root->open_handle = open_list.push(root);  // add root to heap
	allNodes_table.insert(root);       // add root to hash_table (nodes)
	AStarNode* curr = nullptr;
	while (!open_list.empty())
	{
		curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (curr->location == next_location)
				{
					continue;
				}
				next_timestep--;
			}
			if (!constraint_table.constrained(next_location, next_timestep) &&
				!constraint_table.constrained(curr->location, next_location, next_timestep))
			{  // if that move is not blocked
				int next_h_val = compute_heuristic(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new AStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep);
				auto it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					allNodes_table.insert(next);
				}
				else
				{  // update existing node's g_val if needed (only in the heap)
					delete next;  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.increase(existing_next->open_handle);
					}
				}
			}
		}
	}
	releaseNodes();
	return length;
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
		int new_min_f_val = (int) open_head->getFVal();
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
}

