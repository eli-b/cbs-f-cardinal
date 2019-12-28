#include "Conflict.h"
#include "RectangleReasoning.h"

bool traverse(const Path& path, int loc, int t)
{
    if (t >= (int)path.size())
        return loc == path.back().location;
    else return t >= 0 && path[t].location == loc;
}

bool blocked(const Path& path, const std::list<Constraint>& constraints, int num_col)
{
    for (auto constraint : constraints)
    {
        int a, x, y, t;
        constraint_type type;
        tie(a, x, y, t, type) = constraint;
        if (type == constraint_type::RANGE) // time range constraint
        {
            for (int i = y; i < t; i++)
            {
                if (traverse(path, x, i))
                    return true;
            }
        }
        else if (type == constraint_type::BARRIER) // barrier constraint
        {
            int x1 = x / num_col, y1 = x % num_col;
            int x2 = y / num_col, y2 = y % num_col;
            if (x1 == x2)
            {
                if (y1 < y2)
                {
                    for (int i = 0; i <= std::min(y2 - y1, t); i++)
                    {
                        if (traverse(path, x1 * num_col + y2 - i, t - i))
                            return true;
                    }
                }
                else
                {
                    for (int i = 0; i <= std::min(y1 - y2, t); i++)
                    {
                        if (traverse(path, x1 * num_col + y2 + i, t - i))
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
                        if (traverse(path, (x2 - i) * num_col + y1, t - i))
                            return true;
                    }
                }
                else
                {
                    for (int i = 0; i <= std::min(x1 - x2, t); i++)
                    {
                        if (traverse(path, (x2 + i) * num_col + y1, t - i))
                            return true;
                    }
                }
            }
        }
    }
    return false;
}

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
		std::get<2>(constraint) << "," << std::get<3>(constraint) << "," << std::get<4>(constraint) << ">";
	return os;
}

// add a horizontal modified barrier constraint
bool Conflict::addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)mdd->levels.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
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
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			int loc2 = (Ri_y + (t2 - 1 - Ri_t) * sign) + x * num_col;
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
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
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
bool Conflict::addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)mdd->levels.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
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
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			int loc2 = (Ri_x + (t2 - 1 - Ri_t) * sign) * num_col + y;
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
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
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


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	switch (conflict.p)
	{
		case conflict_priority::CARDINAL:
			os << "cardinal ";
			break;
		case conflict_priority::SEMI:
			os << "semi-cardinal ";
			break;
		case conflict_priority::NON:
			os << "non-cardinal ";
			break;
        case conflict_priority::UNKNOWN:
            break;
        case conflict_priority::PRIORITY_COUNT:
            break;
    }
	switch (conflict.type)
	{
		case conflict_type::STANDARD:
			os << "standard";
			break;
		case conflict_type::RECTANGLE:
			os << "rectangle";
			break;
		case conflict_type::CORRIDOR:
			os << "corrdior";
			break;
		case conflict_type::TARGET:
			os << "target";
	    case conflict_type::TYPE_COUNT:
            break;
    }
	os << " conflict:  " << conflict.a1 << " with ";
	for (auto con : conflict.constraint1)
		os << con << ",";		
	os << " and " << conflict.a2 << " with ";
	for (auto con : conflict.constraint2)
		os << con << ",";		
	return os;
}

bool operator < (const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{
	/*if (conflict1.type == conflict_type::TARGET && conflict2.type == conflict_type::TARGET)
	{
		if (conflict1.p > conflict2.p)
		    return true;
		else if (conflict1.p < conflict2.p)
		    return false;
        else
            return (conflict2.t < conflict1.t);
	}
	else if (conflict1.type == conflict_type::TARGET)
		return false;
	else if (conflict2.type == conflict_type::TARGET)
		return true;*/
	
	if (conflict1.p < conflict2.p)
		return false;
	else if (conflict1.p > conflict2.p)
		return true;
	else if (conflict1.type == conflict_type::TARGET && conflict2.type != conflict_type::TARGET)
	    return false;
    else if (conflict1.type != conflict_type::TARGET && conflict2.type == conflict_type::TARGET)
        return true;
	else if (conflict1.type == conflict_type::CORRIDOR && conflict2.type != conflict_type::CORRIDOR)
        return false;
    else if (conflict1.type != conflict_type::CORRIDOR && conflict2.type == conflict_type::CORRIDOR)
        return true;
	else return (conflict2.t < conflict1.t);
}


bool Conflict::rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
                       const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
                       const MDD* mdd1, const MDD* mdd2, int num_col) // For RM
{
    constraint1.clear();
    constraint2.clear();
    this->a1 = a1;
    this->a2 = a2;
    this->t = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);

    if (s1.first == s2.first)
    {
        if ((s1.second - s2.second) * (s2.second - Rg.second) >= 0)
        {
            // first agent moves horizontally and second agent moves vertically
            if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
            {
                return false;
            }
            if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
            {
                return false;
            }
        }
        else
        {
            // first agent moves vertically and second agent moves horizontally
            if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
            {
                return false;
            }
            if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
            {
                return false;
            }
        }
    }
    else if ((s1.first - s2.first)*(s2.first - Rg.first) >= 0)
    {
        // first agent moves vertically and second agent moves horizontally
        if (!addModifiedHorizontalBarrierConstraint(a1, mdd1, Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
        {
            return false;
        }
        if (!addModifiedVerticalBarrierConstraint(a2, mdd2, Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
        {
            return false;
        }
    }
    else
    {
        // first agent moves horizontally and second agent moves vertically
        if (!addModifiedVerticalBarrierConstraint(a1, mdd1, Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
        {
            return false;
        }
        if (!addModifiedHorizontalBarrierConstraint(a2, mdd2, Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
        {
            return false;
        }
    }
    type = conflict_type::RECTANGLE;
    return true;
}
