#include "ConstraintTable.h"

void ConstraintTable::insert(int from, int to, int t_min, int t_max, int map_size)
{
    insert((from + 1) * map_size + to, t_min, t_max);
}

void ConstraintTable::insert(int loc, int t_min, int t_max)
{
	CT[loc].emplace_back(t_min, t_max);
	if (loc == goal_location && t_max > length_min)
	{
		length_min = t_max;
	}
	if (t_max < INT_MAX && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
}

bool ConstraintTable::is_constrained(int loc, int t) const
{
	const auto& it = CT.find(loc);
	if (it == CT.end())
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

bool ConstraintTable::is_constrained(int curr_loc, int next_loc, int next_t, int map_size) const
{
    return is_constrained((1 + curr_loc) * map_size + next_loc, next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
    length_min = other.length_min;
    length_max = other.length_max;
    goal_location = other.goal_location;
    latest_timestep = other.latest_timestep;
    CT = other.CT;
}