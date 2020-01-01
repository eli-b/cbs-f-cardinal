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
	cout << "	Build conflict graph in " << *this << ": ";
	for (auto e : conflictGraph)
	{
		if (e.second == 0)
			continue;
		int i = e.first / num_of_agents;
		int j = e.first % num_of_agents;
		std::cout << "(" << i << "," << j << ")=" << e.second << ",";
	}
	cout << endl;
}

std::ostream& operator<<(std::ostream& os, const ICBSNode& node)
{
	os << "Node " << node.time_generated << " (" << node.f_val << " = " << node.g_val << " + " <<
		node.h_val << " ) with " << node.num_of_collisions << " conflicts and " <<
		node.paths.size() << " new paths ";
	return os;
}