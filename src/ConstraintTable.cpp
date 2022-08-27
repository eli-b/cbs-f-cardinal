#include "ConstraintTable.h"
#include "Instance.h" // For WAIT_MOVE
#include <new>


// t_max is non-inclusive
void ConstraintTable::insert(size_t from, size_t to, int t_min, int t_max)
{
	insert(getEdgeIndex(from, to), t_min, t_max);
}

// t_max is non-inclusive
void ConstraintTable::insert(size_t loc, int t_min, int t_max)
{
	assert(loc >= 0);
	ct[loc].emplace_back(t_min, t_max);


	if (t_max < MAX_TIMESTEP && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
	else if (t_max == MAX_TIMESTEP && t_min > latest_timestep)
	{
		latest_timestep = t_min;
	}
}

void ConstraintTable::insertLandmark(size_t loc, int t)
{
	auto it = landmarks.find(t);
	if (it == landmarks.end())
	{
		landmarks[t] = loc;
		if (t > latest_timestep)
			latest_timestep = t;
	}
	else
		assert(it->second == loc);
}

// return the location-time pairs on the barrier in an increasing order of their timesteps
list<pair<int, int>> ConstraintTable::decodeBarrier(int x, int y, int t)
{
	list<pair<int, int>> rst;
	int x1 = x / num_col, y1 = x % num_col;
	int x2 = y / num_col, y2 = y % num_col;
	if (x1 == x2)
	{
		if (y1 < y2)
			for (int i = min(y2 - y1, t); i >= 0; i--)
			{
				rst.emplace_back(x1 * num_col + y2 - i, t - i);
			}
		else
			for (int i = min(y1 - y2, t); i >= 0; i--)
			{
				rst.emplace_back(x1 * num_col + y2 + i, t - i);
			}
	}
	else // y1== y2
	{
		if (x1 < x2)
			for (int i = min(x2 - x1, t); i >= 0; i--)
			{
				rst.emplace_back((x2 - i) * num_col + y1, t - i);
			}
		else
			for (int i = min(x1 - x2, t); i >= 0; i--)
			{
				rst.emplace_back((x2 + i) * num_col + y1, t - i);
			}
	}
	return rst;
}

bool ConstraintTable::constrained_by_range(size_t loc, int t) const
{
	const auto& it = ct_ranges.find(loc);
	if (it == ct_ranges.end())
	{
		return false;
	}
	for (const auto& constraint: it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}
	return false;
}

bool ConstraintTable::constrained_by_time(size_t curr_loc, size_t next_loc, int next_t) const
{
	auto [found, state] = ct.get(next_loc, next_t);
	if (found)
	{
		if (state->vertex)
			return true;
		// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
		for (int i = 0; i < Instance::WAIT_MOVE; i++) // the wait action cannot lead to edge conflicts
		{
			if (next_loc - curr_loc == moves_offset[i])
			{
				if (state->edge[i])
					return true;
				break;
			}
		}
	}
	return false;
}

bool ConstraintTable::constrained(size_t loc, int t) const
{
	assert(loc >= 0);
	if (loc < map_size)
	{
		const auto& it = landmarks.find(t);
		if (it != landmarks.end() && it->second != loc)
			return true;  // violate the positive vertex constraint
	}

	bool is_constrained = constrained_by_time(loc, t);
	if (is_constrained)
		return true;

	return constrained_by_range(loc, t);
}

bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc, int next_t) const
{
	bool is_constrained = constrained_by_time(curr_loc, next_loc, next_t);
	if (is_constrained)
		return true;

	return constrained_by_range(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
	length_min = other.length_min;
	length_max = other.length_max;
	goal_location = other.goal_location;
	latest_timestep = other.latest_timestep;
	num_col = other.num_col;
	map_size = other.map_size;
	ct_ranges = other.ct_ranges;
	landmarks = other.landmarks;
	// Copy ct:
	for (size_t i = 0 ; i < other.ct.data.size() ; ++i)
	{
		if (other.ct.data[i] == nullptr)
			continue;
		for (auto& tuple_of_int_and_unique_ptr : *other.ct.data[i])
		{
			auto& [t, constraint_state_ptr] = tuple_of_int_and_unique_ptr;
			ct.set(i, t, new (constraint_state_pool.malloc()) ConstraintState(*constraint_state_ptr));
		}
	}
	// we do not copy cat
}


// build the constraint table for the given agent at the given node
void ConstraintTable::build(const CBSNode& node, int agent)
{
	auto curr = &node;
	while (curr->parent != nullptr)
	{
		auto[a, x, y, t, type] = curr->constraints.front();
		switch (type)
		{
		case constraint_type::LEQLENGTH:
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to reach its goal at or before timestep t.
				length_max = min(length_max, t);
			else // other agents cannot stay at x at or after timestep t
				insert(x, t, MAX_TIMESTEP);
			break;
		case constraint_type::GLENGTH:
			assert(curr->constraints.size() == 1);
			if (a == agent) // path of agent_id should be of length at least t + 1
				length_min = max(length_min, t + 1);
			break;
		case constraint_type::POSITIVE_VERTEX:
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t
			{
				insertLandmark(x, t);
			}
			else // other agents cannot stay at x at timestep t
			{
				insert(x, t, t + 1);
			}
			break;
		case constraint_type::POSITIVE_EDGE:
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
			{
				insertLandmark(x, t - 1);
				insertLandmark(y, t);
			}
			else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
			{
				insert(x, t - 1, t);
				insert(y, t, t + 1);
				insert(y, x, t, t + 1);
			}
			break;
		case constraint_type::VERTEX:
			if (a == agent)
			{
				for (const auto& constraint : curr->constraints) // we might have multiple vertex constraints generated by mutex propagation
				{
					tie(a, x, y, t, type) = constraint;
					insert(x, t, t + 1);
				}
			}
			break;
		case constraint_type::EDGE:
			assert(curr->constraints.size() == 1);
			if (a == agent)
				insert(x, y, t, t + 1);
			break;
		case constraint_type::BARRIER:
			if (a == agent)
			{
				for (auto constraint : curr->constraints)
				{
					tie(a, x, y, t, type) = constraint;
					assert(a == agent);
					auto states = decodeBarrier(x, y, t); // state = (location, timestep)
					for (const auto& state : states)
					{
						insert(state.first, state.second, state.second + 1);
					}
				}
			}
			break;
		case constraint_type::RANGE_VERTEX:
			assert(curr->constraints.size() == 1);
			if (a == agent)
			{
				int t1 = y;
				int t2 = t;
				insert(x, t1, t2 + 1); // the agent cannot stay at x from timestep y to timestep t.
			}
			break;
		case constraint_type::RANGE_EDGE:
			assert(curr->constraints.size() == 1);
			if (a == agent)
			{
				int t2 = t;
				insert(x, y, 0, t2 + 1); // the agent cannot use x,y from timestep 0 to timestep t.
			}
			break;
		}
		curr = curr->parent;
	}
	if (latest_timestep < length_min)
		latest_timestep = length_min;
	if (length_max < MAX_TIMESTEP && latest_timestep < length_max)
		latest_timestep = length_max;
}


// build the conflict avoidance table
//void ConstraintTable::buildCAT(int agent, const vector<Path*>& paths, size_t _cat_size)
//{
//	if (length_min >= MAX_TIMESTEP || length_min > length_max) // the agent cannot reach its goal location
//		return; // don't have to build CAT
//	cat_size = std::max(_cat_size, (size_t) latest_timestep);
//	if (map_size < map_size_threshold)
//	{
//		// cat_small.resize(cat_size * map_size, false);
//		cat_small.resize(cat_size, vector<bool>(map_size, false));
//		for (size_t ag = 0; ag < paths.size(); ag++)
//		{
//			if (ag == agent || paths[ag] == nullptr)
//				continue;
//			for (size_t timestep = 0; timestep < paths[ag]->size(); timestep++)
//			{
//				//cat_small[timestep * map_size + paths[ag]->at(timestep).location] = true;
//				cat_small[timestep][paths[ag]->at(timestep).location] = true;
//			}
//			int goal = paths[ag]->back().location;
//			for (size_t timestep = paths[ag]->size(); timestep < cat_size; timestep++)
//				// cat_small[timestep * map_size + goal] = true;
//				cat_small[timestep][goal] = true;
//		}
//	}
//	else
//	{
//		cat_large.resize(cat_size);
//		for (size_t ag = 0; ag < paths.size(); ag++)
//		{
//			if (ag == agent || paths[ag] == nullptr)
//				continue;
//			int prev = paths[ag]->front().location;
//			int curr;
//			for (size_t timestep = 1; timestep < paths[ag]->size(); timestep++)
//			{
//				curr = paths[ag]->at(timestep).location;
//				cat_large[timestep].push_back(curr);
//				cat_large[timestep].push_back(getEdgeIndex(curr, prev));
//				prev = curr;
//			}
//			int goal = paths[ag]->back().location;
//			for (size_t timestep = paths[ag]->size(); timestep < cat_size; timestep++)
//				cat_large[timestep].push_back(goal);
//		}
//	}
//}

//int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const
//{
//	if (map_size < map_size_threshold)
//	{
//		if (next_timestep >= (int) cat_small.size())
//		{
//			if (cat_small.back()[next_id])
//				return 1;
//			else
//				return 0;
//		}
//		if (cat_small[next_timestep][next_id] ||
//			(curr_id != next_id && cat_small[next_timestep - 1][next_id] && cat_small[next_timestep][curr_id]))
//			return 1;
//		else
//			return 0;
//	}
//	else
//	{
//		if (next_timestep >= (int) cat_large.size())
//		{
//			for (const auto& loc : cat_large.back())
//			{
//				if (loc == next_id)
//					return 1;
//			}
//			return 0;
//		}
//		for (const auto& loc : cat_large[next_timestep])
//		{
//			if (loc == next_id)
//			{
//				return 1;
//			}
//			if (loc == getEdgeIndex(curr_id, next_id))
//			{
//				return 1;
//			}
//		}
//		return 0;
//		/*auto& it = cat.back().find(next_id);
//		if (it != cat.back().end())
//			return 1;
//		else
//			return 0;*/
//	}
//	/*int rst = 0;
//	auto& it = cat[next_timestep].find(next_id);
//	if (it != cat[next_timestep].end())
//	{
//		rst++;
//	}
//	if (curr_id != next_id)
//	{
//		it = cat[next_timestep].find(getEdgeIndex(curr_id, next_id));
//		if (it != cat[next_timestep].end())
//		{
//			rst++;
//		}
//	}
//	return rst;*/
//
//
//	/*if (next_timestep >= (int)cat.size())
//	{
//		auto& it = cat.back().find(next_id);
//		if (it != cat.back().end())
//			return 1;
//		else
//			return 0;
//	}
//	int rst = 0;
//	auto& it = cat[next_timestep].find(next_id);
//	if (it != cat[next_timestep].end())
//	{
//		rst++;
//	}
//	if (curr_id != next_id)
//	{
//		it = cat[next_timestep].find(getEdgeIndex(curr_id, next_id));
//		if (it != cat[next_timestep].end())
//		{
//			rst++;
//		}
//	}
//	return rst;
//
//	*/
//}


// return the earliest timestep that the agent can hold its goal location
int ConstraintTable::getHoldingTime()
{
	int rst = length_min;
	auto it = ct.find(goal_location);
	if (it != ct.end())
	{
		for (auto time_range : it->second)
			rst = max(rst, time_range.second);
	}
	for (auto landmark : landmarks)
	{
		if (landmark.second != goal_location)
			rst = max(rst, (int) landmark.first + 1);
	}
	return rst;
}