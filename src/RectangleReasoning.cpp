#include "RectangleReasoning.h"


std::shared_ptr<Conflict> RectangleReasoning::findRectangleConflict(const vector<Path*>& paths, int timestep,
        int a1, int a2, int loc1, const MDD* mdd1, const MDD* mdd2)
{
    std::shared_ptr<Conflict> rectangle = nullptr;
    //Rectangle reasoning for semi and non cardinal vertex conflicts
    std::list<int>	s1s = getStartCandidates(*paths[a1], timestep);
    std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep);
    std::list<int>	s2s = getStartCandidates(*paths[a2], timestep);
    std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep);
	pair<int, int> location = instance.getCoordinate(loc1);

    // Try all possible combinations
    // std::shared_ptr<Conflict> conflict;
    int type = -1;
    int area = 0;
    for (int t1_start : s1s)
    {
        for (int t1_end : g1s)
        {
            int s1 = paths[a1]->at(t1_start).location;
            int g1 = paths[a1]->at(t1_end).location;
            if (instance.getManhattanDistance(s1, g1) !=  t1_end - t1_start)
                continue;
            for (int t2_start : s2s)
            {
                for (int t2_end : g2s)
                {
                    int s2 = paths[a2]->at(t2_start).location;
                    int g2 = paths[a2]->at(t2_end).location;
                    if (instance.getManhattanDistance(s2, g2) != t2_end - t2_start)
                        continue;
                    if (!isRectangleConflict(s1, s2, g1, g2))
                        continue;
                    std::pair<int, int> Rg = getRg(instance.getCoordinate(s1), instance.getCoordinate(g1), instance.getCoordinate(g2));
                    std::pair<int, int> Rs = getRs(instance.getCoordinate(s1), instance.getCoordinate(s2), instance.getCoordinate(g1));
                    int new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
                    int new_type = classifyRectangleConflict(s1, s2, g1, g2, Rg);
                    if (new_type > type || (new_type == type && new_area > area))
                    {
                        int Rg_t = timestep + abs(Rg.first - location.first) + abs(Rg.second - location.second);
						list<Constraint> constraint1;
						list<Constraint> constraint2;
						bool succ = addModifiedBarrierConstraints(a1, a2, Rs, Rg, 
							instance.getCoordinate(s1), instance.getCoordinate(s2), 
							Rg_t, mdd1, mdd2, constraint1, constraint2);
                        if (succ && blocked(*paths[a1], constraint1) && blocked(*paths[a2], constraint2))
                        {
							std::shared_ptr<Conflict> new_rectangle = shared_ptr<Conflict>(new Conflict());
							new_rectangle->rectangleConflict(a1, a2, Rs, Rg, Rg_t, constraint1, constraint2);
                            if (new_type == 2)
								new_rectangle->p = conflict_priority::CARDINAL;
                            else if (new_type == 1) // && !findRectangleConflict(parent.parent, *conflict))
								new_rectangle->p = conflict_priority::SEMI;
                            else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
								new_rectangle->p = conflict_priority::NON;
                            type = new_type;
                            area = new_area;
							rectangle = new_rectangle;
                        }
                    }
                }
            }
        }
    }

    return rectangle;
}

//Identify rectangle conflicts for CR/R
bool RectangleReasoning::isRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t)
{
	return g1_t == abs(s1.first - g1.first) + abs(s1.second - g1.second) &&  // Manhattan-optimal
		        g2_t == abs(s2.first - g2.first) + abs(s2.second - g2.second) && // Manhattan-optimal
			(s1.first - g1.first) * (s2.first - g2.first) >= 0 &&  //Move in the same direction
			(s1.second - g1.second) * (s2.second - g2.second) >= 0; //Move in the same direction
}

//Identify rectangle conflicts for RM
bool RectangleReasoning::isRectangleConflict(int s1, int s2, int g1, int g2)
{
	if (s1 == s2) // A standard cardinal conflict
		return false;
	else if (s1 == g1 || s2 == g2) // s1 = g1 or  s2 = g2
		return false;
	pair<int, int> S1 = instance.getCoordinate(s1);
	pair<int, int> S2 = instance.getCoordinate(s2);
	pair<int, int> G1 = instance.getCoordinate(g1);
	pair<int, int> G2 = instance.getCoordinate(g2);

	if ((S1.first - G1.first) * (S2.first - G2.first) < 0 || (S1.second - G1.second) * (S2.second - G2.second) < 0) // Not move in the same direction
		return false;
	else if ((S2.first - S1.first) * (S1.first - G1.first) < 0 && (S2.second - S1.second) * (S1.second - G1.second) < 0) // s1 always in the middle
		return false;
	else if ((S1.first - S2.first) * (S2.first - G2.first) < 0 && (S1.second - S2.second) * (S2.second - G2.second) < 0) // s2 always in the middle
		return false;
	else return !((S1.first == G1.first && S2.second == G2.second) || (S1.second == G1.second && S2.first == G2.first));
}

//Classify rectangle conflicts for CR/R
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int RectangleReasoning::classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2)
{
		int cardinal1 = 0, cardinal2 = 0;
		if ((s1.first - s2.first) * (g1.first - g2.first) <= 0)
			cardinal1++;
		if ((s1.second - s2.second) * (g1.second - g2.second) <= 0)
			cardinal2++;
		return cardinal1 + cardinal2;
}

//Classify rectangle conflicts for RM
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int RectangleReasoning::classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg)
{
	int cardinal1 = 0, cardinal2 = 0;
	pair<int, int> S1 = instance.getCoordinate(s1);
	pair<int, int> S2 = instance.getCoordinate(s2);
	pair<int, int> G1 = instance.getCoordinate(g1);
	pair<int, int> G2 = instance.getCoordinate(g2);

	if ((S1.first == S2.first && (S1.second - S2.second) * (S2.second - Rg.second) >= 0) ||
		(S1.first != S2.first && (S1.first - S2.first)*(S2.first - Rg.first) < 0))
	{
		if (Rg.first == G1.first)
			cardinal1 = 1;
		if (Rg.second == G2.second)
			cardinal2 = 1;
	}
	else
	{
		if (Rg.second == G1.second)
			cardinal1 = 1;
		if (Rg.first == G2.first)
			cardinal2 = 1;
	}

	return cardinal1 + cardinal2;
}

//Compute rectangle corner Rs
std::pair<int, int> RectangleReasoning::getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1)
{
	int x, y;
	if (s1.first == g1.first)
		x = s1.first;
	else if (s1.first < g1.first)
		x = std::max(s1.first, s2.first);
	else
		x = std::min(s1.first, s2.first);
	if (s1.second == g1.second)
		y = s1.second;
	else if (s1.second < g1.second)
		y = std::max(s1.second, s2.second);
	else
		y = std::min(s1.second, s2.second);
	return std::make_pair(x, y);
}

//Compute rectangle corner Rg
std::pair<int, int> RectangleReasoning::getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2)
{
	int x, y;
	if (s1.first == g1.first)
		x = g1.first;
	else if (s1.first < g1.first)
		x = std::min(g1.first, g2.first);
	else
		x = std::max(g1.first, g2.first);
	if (s1.second == g1.second)
		y = g1.second;
	else if (s1.second < g1.second)
		y = std::min(g1.second, g2.second);
	else
		y = std::max(g1.second, g2.second);
	return std::make_pair(x, y);
}

//Compute start candidates for RM
std::list<int> RectangleReasoning::getStartCandidates(const std::vector<PathEntry>& path, int timestep)
{
	std::list<int> starts;
	for (int t = 0; t <= timestep; t++) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && instance.getManhattanDistance(path[t].location, path[timestep].location) == timestep - t)
			starts.push_back(t);
	}
	return starts;
}

//Compute goal candidates for RM
std::list<int> RectangleReasoning::getGoalCandidates(const std::vector<PathEntry>& path, int timestep)
{
	std::list<int> goals;
	for (int t = path.size() - 1; t >= timestep; t--) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && instance.getManhattanDistance(path[t].location, path[timestep].location) == t - timestep)
			goals.push_back(t);
	}
	return goals;
}


/*int getRectangleTime(const Conflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col)
{
	int s1 = paths[conflict.a1]->at(std::get<3>(conflict)).location;
	int s2 = paths[conflict.a2]->at(std::get<4>(conflict)).location;
	std::pair<int, int>  S1 = std::make_pair(s1 / num_col, s1 % num_col);
	std::pair<int, int> S2 = std::make_pair(s2 / num_col, s2 % num_col);
	std::pair<int, int> Rg = std::make_pair(std::get<2>(conflict) / num_col, std::get<2>(conflict) % num_col);
	std::pair<int, int> Rs = getRs(S1, S2, Rg);
	return std::get<3>(conflict) - abs(S1.first - Rs.first) - abs(S1.second - Rs.second);
}*/

bool RectangleReasoning::addModifiedBarrierConstraints(int a1, int a2, 
	const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
	const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
	const MDD* mdd1, const MDD* mdd2,
	list<Constraint>& constraint1, list<Constraint>& constraint2)
{
	if (s1.first == s2.first)
	{
		if ((s1.second - s2.second) * (s2.second - Rg.second) >= 0)
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2))
			{
				return false;
			}
		}
		else
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2))
			{
				return false;
			}
		}
	}
	else if ((s1.first - s2.first)*(s2.first - Rg.first) >= 0)
	{
		// first agent moves vertically and second agent moves horizontally
		if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, constraint1))
		{
			return false;
		}
		if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, constraint2))
		{
			return false;
		}
	}
	else
	{
		// first agent moves horizontally and second agent moves vertically
		if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, constraint1))
		{
			return false;
		}
		if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, constraint2))
		{
			return false;
		}
	}
	return true;
}


// add a horizontal modified barrier constraint
bool RectangleReasoning::addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
	int Ri_y, int Rg_y, int Rg_t, std::list<Constraint>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)mdd->levels.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = instance.linearize_coordinate(x,  (Ri_y + (t2 - Ri_t) * sign));
		MDDNode* it = nullptr;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				it = n;
				break;
			}
		}
		if (it == nullptr && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = instance.linearize_coordinate(x, (Ri_y + (t1 - Ri_t) * sign));
			int loc2 = instance.linearize_coordinate(x, (Ri_y + (t2 - 1 - Ri_t) * sign));
			constraints.emplace_back(agent, loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != nullptr && t1 < 0)
		{
			t1 = t2;
		}
		if (it != nullptr && t2 == t_max)
		{
			int loc1 = instance.linearize_coordinate(x, (Ri_y + (t1 - Ri_t) * sign));
			constraints.emplace_back(agent, loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a vertival modified barrier constraint
bool RectangleReasoning::addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
	int Ri_x, int Rg_x, int Rg_t, list<Constraint>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)mdd->levels.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = instance.linearize_coordinate((Ri_x + (t2 - Ri_t) * sign), y);
		MDDNode* it = nullptr;
		for (MDDNode* n : mdd->levels[t2])
		{
			if (n->location == loc)
			{
				it = n;
				break;
			}
		}
		if (it == nullptr && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = instance.linearize_coordinate((Ri_x + (t1 - Ri_t) * sign), y);
			int loc2 = instance.linearize_coordinate((Ri_x + (t2 - 1 - Ri_t) * sign), y);
			constraints.emplace_back(agent, loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != nullptr && t1 < 0)
		{
			t1 = t2;
		}
		if (it != nullptr && t2 == t_max)
		{
			int loc1 = instance.linearize_coordinate((Ri_x + (t1 - Ri_t) * sign), y);
			constraints.emplace_back(agent, loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

bool RectangleReasoning::blocked(const Path& path, const std::list<Constraint>& constraints)
{
	for (auto constraint : constraints)
	{
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = constraint;
		assert(type == constraint_type::BARRIER);
		int x1 = instance.row_coordinate(x), y1 = instance.col_coordinate(x);
		int x2 = instance.row_coordinate(y), y2 = instance.col_coordinate(y);
		if (x1 == x2)
		{
			if (y1 < y2)
			{
				for (int i = 0; i <= std::min(y2 - y1, t); i++)
				{
					if (traverse(path, instance.linearize_coordinate(x1, y2 - i), t - i))
						return true;
				}
			}
			else
			{
				for (int i = 0; i <= std::min(y1 - y2, t); i++)
				{
					if (traverse(path, instance.linearize_coordinate(x1, y2 + i), t - i))
						return true;
				}
			}
		}
		else // y1== y2
		{
			if (x1 < x2)
			{
				for (int i = 0; i <= std::min(x2 - x1, t); i++)
				{
					if (traverse(path, instance.linearize_coordinate(x2 - i, y1), t - i))
						return true;
				}
			}
			else
			{
				for (int i = 0; i <= std::min(x1 - x2, t); i++)
				{
					if (traverse(path, instance.linearize_coordinate(x2 + i, y1), t - i))
						return true;
				}
			}
		}
	}
	return false;
}

bool RectangleReasoning::traverse(const Path& path, int loc, int t)
{
	if (t >= (int)path.size())
		return loc == path.back().location;
	else return t >= 0 && path[t].location == loc;
}