#include "ConstraintTable.h"

void ConstraintTable::insert2CT(int from, int to, int t_min, int t_max)
{
	insert2CT(getEdgeIndex(from, to), t_min, t_max);
}

void ConstraintTable::insert2CT(int loc, int t_min, int t_max)
{
	ct[loc].emplace_back(t_min, t_max);
	if (t_max < MAX_TIMESTEP && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
}


void ConstraintTable::insertLandmark(int loc, int t)
{
	auto it = landmarks.find(t);
	if (it == landmarks.end())
		landmarks[t] = loc;
	else
		assert(it->second == loc);		
}


bool ConstraintTable::constrained(int loc, int t) const
{
	if (loc < map_size)
	{
		const auto& it = landmarks.find(t);
		if (it != landmarks.end() && it->second != loc)
			return true;  // violate the positive vertex constraint
	}	

	const auto& it = ct.find(loc);
	if (it == ct.end())
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

bool ConstraintTable::constrained(int curr_loc, int next_loc, int next_t) const
{
    return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
	length_min = other.length_min;
	length_max = other.length_max;
	goal_location = other.goal_location;
	latest_timestep = other.latest_timestep;
	num_col = other.num_col;
	map_size = other.map_size;
	ct = other.ct;
	landmarks = other.landmarks;
	// we do not copy cat
}


// build the constraint table for the given agent at the give node 
void ConstraintTable::build(const ICBSNode& node, int agent)
{
	auto curr = &node;
	while (curr->parent != nullptr)
	{
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = curr->constraints.front();
		if (type == constraint_type::LEQLENGTH)
		{
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to reach its goal at or before timestep t.
				length_max = min(length_max, t);
			else // other agents cannot stay at x at or after timestep t
				insert2CT(x, t, MAX_TIMESTEP);
		}
		else if (type == constraint_type::POSITIVE_VERTEX)
		{
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t 
			{
				insertLandmark(x, t);
			}
			else // other agents cannot stay at x at timestep t
			{
				insert2CT(x, t, t + 1);
			}
		}
		else if (type == constraint_type::POSITIVE_EDGE)
		{
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
			{
				insertLandmark(x, t - 1);
				insertLandmark(y, t);
			}
			else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
			{
				insert2CT(x, t - 1, t);
				insert2CT(y, t, t + 1);
				insert2CT(y, x, t, t + 1);
			}
		}
		else if (a == agent) // the rest types of constraints only affect agent a
		{
			for (auto constraint : curr->constraints)
			{
				tie(a, x, y, t, type) = constraint;
				if (type == constraint_type::RANGE) // time range constraint
				{
					insert2CT(x, y, t + 1); // the agent cannot stay at x from timestep y to timestep t.
				}
				else if (type == constraint_type::BARRIER) // barrier constraint
				{
					int x1 = x / num_col, y1 = x % num_col;
					int x2 = y / num_col, y2 = y % num_col;
					if (x1 == x2)
					{
						if (y1 < y2)
							for (int i = 0; i <= std::min(y2 - y1, t); i++)
							{
								insert2CT(x1 * num_col + y2 - i, t - i, t - i + 1);
							}
						else
							for (int i = 0; i <= std::min(y1 - y2, t); i++)
							{
								insert2CT(x1 * num_col + y2 + i, t - i, t - i + 1);
							}
					}
					else // y1== y2
					{
						if (x1 < x2)
							for (int i = 0; i <= std::min(x2 - x1, t); i++)
							{
								insert2CT((x2 - i) * num_col + y1, t - i, t - i + 1);
							}
						else
							for (int i = 0; i <= std::min(x1 - x2, t); i++)
							{
								insert2CT((x2 + i) * num_col + y1, t - i, t - i + 1);
							}
					}
				}
				else if (type == constraint_type::GLENGTH)
				{
					// path of agent_id should be of length at least t + 1
					length_min = max(length_min, t + 1);
				}
				else if (type == constraint_type::VERTEX)
				{
					insert2CT(x, t, t + 1);
				}
				else // edge
				{
					assert(type == constraint_type::EDGE);
					insert2CT(x, y, t, t + 1);
				}
			}
		}
		curr = curr->parent;
	}
}


// build the conflict avoidance table
void ConstraintTable::buildCAT(int agent, const vector<Path*>& paths, size_t cat_size)
{
	if (map_size < map_size_threshold)
	{
		// cat_small.resize(cat_size * map_size, false);
		cat_small.resize(cat_size, vector<size_t>(map_size, false));
		for (size_t ag = 0; ag < paths.size(); ag++)
		{
			if (ag == agent || paths[ag] == nullptr)
				continue;
			for (size_t timestep = 0; timestep < paths[ag]->size(); timestep++)
			{
				//cat_small[timestep * map_size + paths[ag]->at(timestep).location] = true;
				cat_small[timestep][paths[ag]->at(timestep).location] = true;
			}
			int goal = paths[ag]->back().location;
			for (size_t timestep = paths[ag]->size(); timestep < cat_size; timestep++)
				// cat_small[timestep * map_size + goal] = true;
				cat_small[timestep][goal] = true;
		}
	}
	else
	{
		cat_large.resize(cat_size);
		for (size_t ag = 0; ag < paths.size(); ag++)
		{
			if (ag == agent || paths[ag] == nullptr)
				continue;
			int prev = paths[ag]->front().location;
			int curr;
			for (size_t timestep = 1; timestep < paths[ag]->size(); timestep++)
			{
				curr = paths[ag]->at(timestep).location;
				cat_large[timestep].push_back(curr);
				cat_large[timestep].push_back(getEdgeIndex(curr, prev));
				prev = curr;
			}
			int goal = paths[ag]->back().location;
			for (size_t timestep = paths[ag]->size(); timestep < cat_size; timestep++)
				cat_large[timestep].push_back(goal);
		}
	}
}

int ConstraintTable::getNumOfConflictsForStep(int curr_id, int next_id, int next_timestep) const
{
	if (map_size < map_size_threshold)
	{
		if (next_timestep >= (int)cat_small.size())
		{
			if (cat_small.back()[next_id])
				return 1;
			else
				return 0;
		}
		if (cat_small[next_timestep][next_id] ||
			(curr_id != next_id && cat_small[next_timestep - 1][next_id] && cat_small[next_timestep][curr_id]))
			return 1;
		else
			return 0;
	}
	else
	{
		if (next_timestep >= (int)cat_large.size())
		{
			for (const auto& loc : cat_large.back())
			{
				if (loc == next_id)
					return 1;
			}
			return 0;
		}
		for (const auto& loc : cat_large[next_timestep])
		{
			if (loc == next_id)
			{
				return 1;
			}
			if (loc == getEdgeIndex(curr_id, next_id))
			{
				return 1;
			}
		}
		return 0;
		/*auto& it = cat.back().find(next_id);
		if (it != cat.back().end())
			return 1;
		else
			return 0;*/
	}
	/*int rst = 0;
	auto& it = cat[next_timestep].find(next_id);
	if (it != cat[next_timestep].end())
	{
		rst++;
	}
	if (curr_id != next_id)
	{
		it = cat[next_timestep].find(getEdgeIndex(curr_id, next_id));
		if (it != cat[next_timestep].end())
		{
			rst++;
		}
	}
	return rst;*/
	

	/*if (next_timestep >= (int)cat.size())
	{
		auto& it = cat.back().find(next_id);
		if (it != cat.back().end())
			return 1;
		else
			return 0;
	}
	int rst = 0;
	auto& it = cat[next_timestep].find(next_id);
	if (it != cat[next_timestep].end())
	{
		rst++;
	}
	if (curr_id != next_id)
	{
		it = cat[next_timestep].find(getEdgeIndex(curr_id, next_id));
		if (it != cat[next_timestep].end())
		{
			rst++;
		}
	}
	return rst;

	*/
}




// return the earliest timestep that the agent can hold its goal location
int ConstraintTable::getHoldingTime()
{
	int rst = 0;
	auto it = ct.find(goal_location);
	if (it != ct.end())
	{
		for (auto time_range : it->second)
			rst = max(rst, time_range.second);
	}
	for (auto landmark : landmarks)
	{
		if (landmark.second != goal_location)
			rst = max(rst, (int)landmark.first + 1);
	}
	return rst;
}