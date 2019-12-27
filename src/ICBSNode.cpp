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
        if (curr->agent_id == agent)
        {
            for (auto constraint : curr->constraints)
            {
                int x, y, z;
                constraint_type type;
                tie(x, y, z, type) = constraint;
                if (type == constraint_type::RANGE) // time range constraint
                {
                    constraint_table.insert(x, y, z + 1);
                }
                else if (type == constraint_type::BARRIER) // barrier constraint
                {
                    int x1 = x / num_col, y1 = x % num_col;
                    int x2 = y / num_col, y2 = y % num_col;
                    if (x1 == x2)
                    {
                        if (y1 < y2)
                            for (int i = 0; i <= std::min(y2 - y1, z); i++)
                            {
                                constraint_table.insert(x1 * num_col + y2 - i, z - i, z - i + 1);
                            }
                        else
                            for (int i = 0; i <= std::min(y1 - y2, z); i++)
                            {
                                constraint_table.insert(x1 * num_col + y2 + i, z - i, z - i + 1);
                            }
                    }
                    else // y1== y2
                    {
                        if (x1 < x2)
                            for (int i = 0; i <= std::min(x2 - x1, z); i++)
                            {
                                constraint_table.insert((x2 - i) * num_col + y1, z - i, z - i + 1);
                            }
                        else
                            for (int i = 0; i <= std::min(x1 - x2, z); i++)
                            {
                                constraint_table.insert((x2 + i) * num_col + y1, z - i, z - i + 1);
                            }
                    }
                }
                else if (type == constraint_type::LENGTH)
                {
                    if (x < 0 && y == agent)
                    { // <-1, agent_id, t>: path of agent_id should be of length at least t + 1
                        constraint_table.length_min = max(constraint_table.length_min, z + 1);
                    }
                    else if (x >= 0 && y == agent)
                    { // <loc, agent_id, t, TARGET>: path of agent_id should be of length at most t
                        constraint_table.length_max = min(constraint_table.length_max, z);
                    }
                    else if (x >= 0 && y != agent)
                    { // <loc, agent_id, t, TARGET>: any other agent cannot be at loc at or after timestep t
                        constraint_table.insert(x, z, INT_MAX);
                    }
                }
                else if (type == constraint_type::VERTEX)
                {
                    constraint_table.insert(x, z, z +1);
					if (x == constraint_table.goal_location && z >= constraint_table.length_min)
						constraint_table.length_min = z + 1;
                }
                else // edge
                {
                    constraint_table.insert(x, y, z, z + 1, map_size);
                }
            }
        }
        else if (!curr->constraints.empty())
        {
            int x, y, z;
            constraint_type type;
            tie(x, y, z, type) = curr->constraints.front();
            if (type == constraint_type::LENGTH && x >=0 && y != agent)
            {
                constraint_table.insert(x, z, INT_MAX);
            }
        }
        curr = curr->parent;
    }

}