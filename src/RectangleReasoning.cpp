#include "RectangleReasoning.h"
#include <algorithm>    // std::find


std::shared_ptr<Conflict> findRectangleConflict(const vector<Path*>& paths, int timestep, int num_col, int map_size,
        int a1, int a2, int loc1, const MDD* mdd1, const MDD* mdd2)
{
    std::shared_ptr<Conflict> rectangle = nullptr;
    //Rectangle reasoning for semi and non cardinal vertex conflicts
    std::list<int>	s1s = getStartCandidates(*paths[a1], timestep, num_col);
    std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep, num_col);
    std::list<int>	s2s = getStartCandidates(*paths[a2], timestep, num_col);
    std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep, num_col);

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
            if (!isManhattanOptimal(s1, g1, t1_end - t1_start, num_col))
                continue;
            for (int t2_start : s2s)
            {
                for (int t2_end : g2s)
                {
                    int s2 = paths[a2]->at(t2_start).location;
                    int g2 = paths[a2]->at(t2_end).location;
                    if (!isManhattanOptimal(s2, g2, t2_end - t2_start, num_col))
                        continue;
                    if (!isRectangleConflict(s1, s2, g1, g2, num_col))
                        continue;
                    std::pair<int, int> Rg = getRg(std::make_pair(s1 / num_col, s1 %  num_col), std::make_pair(g1 / num_col, g1 %  num_col),
                                                   std::make_pair(g2 / num_col, g2 %  num_col));
                    std::pair<int, int> Rs = getRs(std::make_pair(s1 / num_col, s1 %  num_col), std::make_pair(s2 / num_col, s2 %  num_col),
                                                   std::make_pair(g1 / num_col, g1 %  num_col));
                    int new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
                    int new_type = classifyRectangleConflict(s1, s2, g1, g2, Rg, num_col);
                    if (new_type > type || (new_type == type && new_area > area))
                    {
                        int Rg_t = timestep + abs(Rg.first - loc1 / num_col) + abs(Rg.second - loc1 % num_col);
						std::shared_ptr<Conflict> new_rectangle = shared_ptr<Conflict>(new Conflict());
						new_rectangle->rectangleConflict(a1, a2, Rs, Rg,
                                make_pair(s1 / num_col, s1 % num_col),
                                make_pair(s2 / num_col, s2 % num_col), Rg_t, mdd1, mdd2, num_col);
                        if (blocked(*paths[new_rectangle->a1], new_rectangle->constraint1, num_col) && blocked(*paths[new_rectangle->a2], new_rectangle->constraint2, num_col))
                        {
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
bool isRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
	const std::pair<int, int>& g1, const std::pair<int, int>& g2, int g1_t, int g2_t)
{
	return g1_t == abs(s1.first - g1.first) + abs(s1.second - g1.second) &&  // Manhattan-optimal
		        g2_t == abs(s2.first - g2.first) + abs(s2.second - g2.second) && // Manhattan-optimal
			(s1.first - g1.first) * (s2.first - g2.first) >= 0 &&  //Move in the same direction
			(s1.second - g1.second) * (s2.second - g2.second) >= 0; //Move in the same direction
}

//Identify rectangle conflicts for RM
bool isRectangleConflict(int s1, int s2, int g1, int g2, int num_col)
{
	if (s1 == s2) // A standard cardinal conflict
		return false;
	else if (s1 == g1 || s2 == g2) // s1 = g1 or  s2 = g2
		return false;
	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;

	if ((s1_x - g1_x) * (s2_x - g2_x) < 0 || (s1_y - g1_y) * (s2_y - g2_y) < 0) // Not move in the same direction
		return false;
	else if ((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) // s1 always in the middle
		return false;
	else if ((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) // s2 always in the middle
		return false;
	else return !((s1_x == g1_x && s2_y == g2_y) || (s1_y == g1_y && s2_x == g2_x));
}

//Classify rectangle conflicts for CR/R
// Return 2 if it is a cardinal rectangle conflict
// Return 1 if it is a semi-cardinal rectangle conflict
// Return 0 if it is a non-cardinal rectangle conflict
int classifyRectangleConflict(const std::pair<int, int>& s1, const std::pair<int, int>& s2,
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
int classifyRectangleConflict(int s1, int s2, int g1, int g2, const std::pair<int, int>& Rg, int num_col)
{
	int cardinal1 = 0, cardinal2 = 0;

	int s1_x = s1 / num_col, s1_y = s1 % num_col;
	int s2_x = s2 / num_col, s2_y = s2 % num_col;
	int g1_x = g1 / num_col, g1_y = g1 % num_col;
	int g2_x = g2 / num_col, g2_y = g2 % num_col;

	if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg.second) >= 0) ||
		(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg.first) < 0))
	{
		if (Rg.first == g1_x)
			cardinal1 = 1;
		if (Rg.second == g2_y)
			cardinal2 = 1;
	}
	else
	{
		if (Rg.second == g1_y)
			cardinal1 = 1;
		if (Rg.first == g2_x)
			cardinal2 = 1;
	}

	return cardinal1 + cardinal2;
}

//Compute rectangle corner Rs
std::pair<int, int> getRs(const std::pair<int, int>& s1, const std::pair<int, int>& s2, const std::pair<int, int>& g1)
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
std::pair<int, int> getRg(const std::pair<int, int>& s1, const std::pair<int, int>& g1, const std::pair<int, int>& g2)
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
std::list<int>  getStartCandidates(const std::vector<PathEntry>& path, int timestep, int num_col)
{
	std::list<int> starts;
	for (int t = 0; t <= timestep; t++) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && isManhattanOptimal(path[t].location, path[timestep].location, timestep - t, num_col))
			starts.push_back(t);
	}
	return starts;
}

//Compute goal candidates for RM
std::list<int>  getGoalCandidates(const std::vector<PathEntry>& path, int timestep, int num_col)
{
	std::list<int> goals;
	for (int t = path.size() - 1; t >= timestep; t--) //Find start that is single and Manhattan-optimal to conflicting location
	{
		if (path[t].single && isManhattanOptimal(path[t].location, path[timestep].location, t - timestep, num_col))
			goals.push_back(t);
	}
	return goals;
}

// whether the path between loc1 and loc2 is Manhattan-optimal
bool isManhattanOptimal(int loc1, int loc2, int dist, int num_col)
{
	return abs(loc1 / num_col - loc2 / num_col) + abs(loc1 % num_col - loc2 % num_col) == dist;
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
