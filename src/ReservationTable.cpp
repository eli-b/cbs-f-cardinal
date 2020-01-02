#include "ReservationTable.h"


/*int ResevationTable::get_holding_time(int location)
{ 
	auto it = constraints.find(location);
	if (it != constraints.end())
	{
		for (auto constraint : it->second)
			insert_constraint(location, constraint.first, constraint.second);
	}
	
	if (RT.find(location) == RT.end()) 
	{
		return 0;
	}
	int t = std::get<1>(RT[location].back());
	if (t < INTERVAL_MAX)
		return INTERVAL_MAX;
	for (auto p =  RT[location].rbegin(); p != RT[location].rend(); ++p)
	{
		if (t == std::get<1>(*p))
			t = std::get<0>(*p);
		else
			break;
	}
	return t;
}*/


// build the constraint table and the conflict avoidance table
void ReservationTable::buildCAT(int agent, const vector<Path*>& paths)
{
	for (size_t ag = 0; ag < paths.size(); ag++)
	{
		if (ag == agent || paths[ag] == nullptr)
			continue;
		if (paths[ag]->size() == 1) // its start location is its goal location
		{
			cat[paths[ag]->front().location].emplace_back(0, MAX_TIMESTEP);
			continue;
		}
		int prev_location = paths[ag]->front().location;
		int prev_timestep = 0;
		for (size_t timestep = 0; timestep < paths[ag]->size(); timestep++)
		{
			int curr_location = paths[ag]->at(timestep).location;
			if (prev_location != curr_location)
			{
				cat[prev_location].emplace_back(prev_timestep, timestep); // add vertex conflict
				cat[getEdgeIndex(curr_location, prev_location)].emplace_back(timestep, timestep + 1); // add edge conflict
				prev_location = curr_location;
				prev_timestep = timestep;
			}
		}
		cat[paths[ag]->back().location].emplace_back(paths[ag]->size() - 1, MAX_TIMESTEP);
	}
}

int ReservationTable::getNumOfConflictsForStep(int curr_id, int next_id, int next_timestep) const
{
	int rst = 0;
	const auto& it = cat.find(next_id);
	if (it != cat.end())
	{
		for (const auto& constraint : it->second)
		{
			if (constraint.first <= next_timestep && next_timestep < constraint.second)
				rst++;
		}
	}
	const auto& it2 = cat.find(getEdgeIndex(curr_id, next_id));
	if (it2 != cat.end())
	{
		for (const auto& constraint : it2->second)
		{
			if (constraint.first <= next_timestep && next_timestep < constraint.second)
				rst++;
		}
	}
	return rst;
}

void ReservationTable::insert2RT(int location, size_t t_min, size_t t_max)
{
    if (rt.find(location) == rt.end())
    {
        if (t_min > 0)
        {
            rt[location].emplace_back(0, t_min, 0);
        }
        rt[location].emplace_back(t_max, MAX_TIMESTEP, 0);
        return;
    }
    for (auto it = rt[location].begin(); it != rt[location].end();)
    {
        if (t_min >= get<1>(*it))
			++it; 
        else if (t_max <= get<0>(*it))
            break;
       else  if (get<0>(*it) < t_min && get<1>(*it) <= t_max)
        {
            (*it) = make_tuple(get<0>(*it), t_min, 0);
			++it;
        }
        else if (t_min <= get<0>(*it) && t_max < get<1>(*it))
        {
            (*it) = make_tuple(t_max, get<1>(*it), 0);
            break;
        }
        else if (get<0>(*it) < t_min && t_max < get<1>(*it))
        {
            rt[location].insert(it, make_tuple(get<0>(*it), t_min, 0));
            (*it) = make_tuple(t_max, get<1>(*it), 0);
            break;
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            it = rt[location].erase(it);
        }
    }
}


void ReservationTable::insertSoftConstraint2RT(int location, size_t t_min, size_t t_max)
{
    if (rt.find(location) == rt.end())
    {
        if (t_min > 0)
        {
            rt[location].emplace_back(0, t_min, 0);
        }
        rt[location].emplace_back(t_min, t_max, 1);
        rt[location].emplace_back(t_max, MAX_TIMESTEP, 0);
        return;
    }
    for (auto it = rt[location].begin(); it != rt[location].end(); it++)
    {
        if (t_min >= get<1>(*it))
            continue;
        else if (t_max <= get<0>(*it))
            break;

        int conflicts = get<2>(*it);

        if (get<0>(*it) < t_min && get<1>(*it) <= t_max)
        {
            rt[location].insert(it, make_tuple(get<0>(*it), t_min, conflicts));
            (*it) = make_tuple(t_min, get<1>(*it), conflicts + 1);
        }
        else if (t_min <= get<0>(*it) && t_max < get<1>(*it))
        {
            rt[location].insert(it, make_tuple(get<0>(*it), t_max, conflicts + 1));
            (*it) = make_tuple(t_max, get<1>(*it), conflicts);
        }
        else if (get<0>(*it) < t_min && t_max < get<1>(*it))
        {
            rt[location].insert(it, make_tuple(get<0>(*it), t_min, conflicts));
            rt[location].insert(it, make_tuple(t_min, t_max, conflicts + 1));
            (*it) = make_tuple(t_max, get<1>(*it), conflicts);
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            (*it) = make_tuple(get<0>(*it), get<1>(*it), conflicts + 1);
        }
    }
}


//merge successive safe intervals with the same number of conflicts.
void ReservationTable::mergeIntervals(list<Interval >& intervals)
{
	if (intervals.empty())
		return;
	auto prev = intervals.begin();
	auto curr = prev;
	++curr;
	while (curr != intervals.end())
	{
		if (get<1>(*prev) == get<0>(*curr) && get<2>(*prev) == get<2>(*curr))
		{
			*prev = make_tuple(get<0>(*prev), get<1>(*curr), get<2>(*prev));
			curr = intervals.erase(curr);
		}
		else
		{
			prev = curr;
			++curr;
		}
	}
}


// update RT at the gvien location
void ReservationTable::updateRT(int location)
{
	if (rt.find(location) == rt.end())
	{
		const auto& it = ct.find(location);
		if (it != ct.end())
		{
			for (auto time_range : it->second)
				insert2RT(location, time_range.first, time_range.second);
			ct.erase(it);
		}

		if (location < map_size)
		{
			for (auto landmark : landmarks)
			{
				if (landmark.second != location)
				{
					insert2RT(location, landmark.first, landmark.first + 1);
				}
			}
		}

		const auto& it2 = cat.find(location);
		if (it2 != cat.end())
		{
			for (auto time_range : it2->second)
				insertSoftConstraint2RT(location, time_range.first, time_range.second);
			cat.erase(it2);
			mergeIntervals(rt[location]);
		}
	}
}

// [lower_bound, upper_bound)
list<Interval> ReservationTable::get_safe_intervals(int location, size_t lower_bound, size_t upper_bound)
{
    list<Interval> safe_intervals;
    if (lower_bound >= upper_bound)
        return safe_intervals;

	updateRT(location);
	
	const auto& it = rt.find(location);

    if (it == rt.end()) 
    {
		safe_intervals.emplace_back(0, MAX_TIMESTEP, 0);
		return safe_intervals;
    }

    for(auto interval : rt[location])
    {
        if (lower_bound >= get<1>(interval))
            continue;
        else if (upper_bound <= get<0>(interval))
            break;
        else
        {
            safe_intervals.emplace_back(interval);
        }

    }
    return safe_intervals;
}

// [lower_bound, upper_bound)
list<Interval> ReservationTable::get_safe_intervals(int from, int to, size_t lower_bound, size_t upper_bound)
{
	list<Interval> safe_vertex_intervals = get_safe_intervals(to, lower_bound, upper_bound);
	list<Interval> safe_edge_intervals = get_safe_intervals(getEdgeIndex(from, to), lower_bound, upper_bound);

	list<Interval> safe_intervals;
	auto it1 = safe_vertex_intervals.begin();
	auto it2 = safe_edge_intervals.begin();
	while (it1 != safe_vertex_intervals.end() && it2 != safe_edge_intervals.end())
	{
		int t_min = max(get<0>(*it1), get<0>(*it2));
		int t_max = min(get<1>(*it1), get<1>(*it2));
		if (t_min < t_max)
			safe_intervals.emplace_back(t_min, t_max, get<2>(*it1) + get<2>(*it2));
		if (t_max == get<1>(*it1))
			++it1;
		if (t_max == get<1>(*it2))
			++it2;
	}
	mergeIntervals(safe_intervals);
	return safe_intervals;
}

Interval ReservationTable::get_first_safe_interval(int location)
{
	updateRT(location);
    const auto& it = rt.find(location);
    if (it == rt.end())
		return Interval(0, MAX_TIMESTEP, 0);
    else
		return it->second.front();
}

// find a safe interval with t_min as given
bool ReservationTable::find_safe_interval(Interval& interval, int location, int t_min)
{
	if (t_min >= MAX_TIMESTEP)
		return false;
	updateRT(location);
	const auto& it = rt.find(location);
    if (it == rt.end())
    {
		if (t_min == 0)
		{
			interval = Interval(0, MAX_TIMESTEP, 0);
			return true;
		}
		else
			return  false;
    }
    for( auto i : it->second)
    {
        if (t_min == (int)get<0>(i))
        {
            interval = i;
            return true;
        }
        else if (t_min < (int)get<0>(i))
            break;
    }
    return false;
}


void ReservationTable::print() const
{
    for (const auto& entry : rt)
    {
        cout << "loc=" << entry.first << ":";
        for (const auto& interval : entry.second)
        {
            cout << "[" << get<0>(interval) << "," << get<1>(interval) << "],";
        }
    }
    cout << endl;
}