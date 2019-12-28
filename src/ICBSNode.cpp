#include "ICBSNode.h"
#include <iostream>

void ICBSNode::clear()
{
	conflicts.clear();
	unknownConf.clear();
	conflictGraph.clear();
}

void ICBSNode::printConflictGraph(int num_of_agents) const
{
	std::cout << "Conflict graph in Node " << time_generated << " with f=" << g_val << "+" << h_val << std::endl;
	for (auto e : conflictGraph)
	{
		int i = e.first / num_of_agents;
		int j = e.first % num_of_agents;
		std::cout << "(" << i << "," << j << ")=" << e.second << std::endl;
	}
}

void ICBSNode::getConstraintTable(ConstraintTable& constraint_table, int agent,
        int num_col, int map_size) const
{
    auto curr = this;
    while (curr->parent != nullptr)
    {
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = curr->constraints.front();
		if (type == constraint_type::LEQLENGTH)
		{
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to reach its goal at or before timestep t.
				constraint_table.length_max = min(constraint_table.length_max, t);
			else // other agents cannot stay at x at or after timestep t
				constraint_table.insert(x, t, INT_MAX);
		}
		else if (type == constraint_type::POSITIVE_VERTEX)
		{
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t 
			{
				for (int i = 0; i < map_size; i++)
				{
					if (i != x)
					{
						constraint_table.insert(i, t, t + 1);
					}
				}
			}
			else // other agents cannot stay at x at timestep t
			{
				constraint_table.insert(x, t, t + 1);
			}	
		}
		else if (type == constraint_type::POSITIVE_EDGE)
		{
			assert(curr->constraints.size() == 1);
			if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
			{
				for (int i = 0; i < map_size; i++)
				{
					if (i != x)
					{
						constraint_table.insert(i, t - 1, t);
					}
					if (i != y)
					{
						constraint_table.insert(i, t, t + 1);
					}
				}
			}
			else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
			{
				constraint_table.insert(x, t - 1, t);
				constraint_table.insert(y, t, t + 1);
				constraint_table.insert(y, x, t, t + 1, map_size);
			}
		}
		else if (a == agent) // the rest types of constraints only affect agent a
		{
			for (auto constraint : curr->constraints)
			{
				tie(a, x, y, t, type) = constraint;
				if (type == constraint_type::RANGE) // time range constraint
				{
					constraint_table.insert(x, y, t + 1); // the agent cannot stay at x from timestep y to timestep t.
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
								constraint_table.insert(x1 * num_col + y2 - i, t - i, t - i + 1);
							}
						else
							for (int i = 0; i <= std::min(y1 - y2, t); i++)
							{
								constraint_table.insert(x1 * num_col + y2 + i, t - i, t - i + 1);
							}
					}
					else // y1== y2
					{
						if (x1 < x2)
							for (int i = 0; i <= std::min(x2 - x1, t); i++)
							{
								constraint_table.insert((x2 - i) * num_col + y1, t - i, t - i + 1);
							}
						else
							for (int i = 0; i <= std::min(x1 - x2, t); i++)
							{
								constraint_table.insert((x2 + i) * num_col + y1, t - i, t - i + 1);
							}
					}
				}
				else if (type == constraint_type::GLENGTH)
				{
					// path of agent_id should be of length at least t + 1
					constraint_table.length_min = max(constraint_table.length_min, t + 1);
				}
				else if (type == constraint_type::VERTEX)
				{
					constraint_table.insert(x, t, t + 1);
					if (x == constraint_table.goal_location && t >= constraint_table.length_min)
						constraint_table.length_min = t + 1;
				}
				else // edge
				{
					assert(type == constraint_type::EDGE);
					constraint_table.insert(x, y, t, t + 1, map_size);
				}
			}
		}
        curr = curr->parent;
    }

}