#include "MDD.h"
#include <iostream>
#include "common.h"

/*bool MDD::isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons)  const
{
	if (cons.empty())
		return false;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons.size()))
	{
		for (const auto & it : cons[next_timestep])
		{
			if ((std::get<0>(it) == next_id && std::get<1>(it) < 0)//vertex constraint
				|| (std::get<0>(it) == curr_id && std::get<1>(it) == next_id)) // edge constraint
				return true;
		}
	}
	return false;
}*/

bool MDD::buildMDD(const ConstraintTable& ct,
        int num_of_levels, const SingleAgentSolver* solver)
{
    auto root = new MDDNode(solver->start_location, nullptr); // Root
	std::queue<MDDNode*> open;
	list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(num_of_levels);
	while (!open.empty())
	{
		auto curr = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (curr->level == num_of_levels - 1)
		{
			levels.back().push_back(curr);
			assert(open.empty());
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g - 2. -1 because it's the bound of the children.
		int heuristicBound = num_of_levels - curr->level - 2;
		list<int> next_locations = solver->getNextLocations(curr->location);
		for (int next_location : next_locations) // Try every possible move. We only add backward edges in this step.
		{
			if (solver->my_heuristic[next_location] <= heuristicBound &&
				!ct.constrained(next_location, curr->level + 1) &&
				!ct.constrained(curr->location, next_location, curr->level + 1)) // valid move
			{
				auto child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == curr->level + 1); ++child)
				{
					if ((*child)->location == next_location) // If the child node exists
					{
						(*child)->parents.push_back(curr); // then add corresponding parent link and child link
						find = true;
						break;
					}
				}
				if (!find) // Else generate a new mdd node
				{
					auto childNode = new MDDNode(next_location, curr);
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	assert(levels.back().size() == 1);

	// Backward
	auto goal_node = levels.back().back();
	for (auto parent : goal_node->parents)
	{
		if (parent->location == goal_node->location) // the parent of the goal node should not be at the goal location
			continue;
		levels[num_of_levels - 2].push_back(parent);
		parent->children.push_back(goal_node); // add forward edge	
	}
	for (int t = num_of_levels - 2; t > 0; t--)
	{
		for (auto node : levels[t])
		{
			for (auto parent : node->parents)
			{
				if (parent->children.empty()) // a new node
				{
					levels[t - 1].push_back(parent);
				}
				parent->children.push_back(node); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (auto it : closed)
		if (it->children.empty() && it->level < num_of_levels - 1)
			delete it;
	closed.clear();
	return true;
}

/*bool MDD::buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col)
{
	auto root = new MDDNode(start_location, nullptr); // Root
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
			levels[numOfLevels - 1].push_back(node);
			if (!open.empty())
			{
				std::cerr << "Failed to build MDD!" << std::endl;
				exit(1);
			}
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
		double heuristicBound = numOfLevels - node->level - 2 + 0.001;
		for (int i = 0; i < 5; i++) // Try every possible move. We only add backward edges in this step.
		{
			int newLoc = node->location + moves_offset[i];
			if (validMove(node->location, newLoc, map_size, num_col) &&
				my_heuristic[newLoc] < heuristicBound &&
				!isConstrained(node->location, newLoc, node->level + 1, constraints)) // valid move
			{
				auto child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->location == newLoc) // If the child node exists
					{
						(*child)->parents.push_back(node); // then add corresponding parent link and child link
						find = true;
						break;
					}
				if (!find) // Else generate a new mdd node
				{
					auto childNode = new MDDNode(newLoc, node);
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (auto it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (auto parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (auto & it : closed)
		if (it->children.empty() && it->level < numOfLevels - 1)
			delete it;
	closed.clear();
	return true;
}*/


void MDD::deleteNode(MDDNode* node)
{
	levels[node->level].remove(node);
	for (auto child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if((*child)->parents.empty())
			deleteNode(*child);
	}
	for (auto parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent);
	}
}

void MDD::clear()
{
	if(levels.empty())
		return;
	for (auto & level : levels)
	{
		for (auto & it : level)
			delete it;
	}
	levels.clear();
}

MDDNode* MDD::find(int location, int level) const
{
	if(level < (int)levels.size())
		for (auto it : levels[level])
			if(it->location == location)
				return it;
	return nullptr;
}

MDD::MDD(const MDD & cpy) // deep copy
{
	levels.resize(cpy.levels.size());
	auto root = new MDDNode(cpy.levels[0].front()->location, nullptr);
	levels[0].push_back(root);
	for(size_t t = 0; t < levels.size() - 1; t++)
	{
		for (auto node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->location, (*node)->level);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = find((*cpyChild)->location, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new MDDNode((*cpyChild)->location, (*node));
					levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}
		
	}
}

MDD::~MDD()
{
	clear();
}


SyncMDD::SyncMDD(const MDD & cpy) // deep copy of a MDD
{
	levels.resize(cpy.levels.size());
	auto root = new SyncMDDNode(cpy.levels[0].front()->location, nullptr);
	levels[0].push_back(root);
	for (int t = 0; t < (int)levels.size() - 1; t++)
	{
		for (auto node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->location, t);
			for (list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				SyncMDDNode* child = find((*cpyChild)->location, (*cpyChild)->level);
				if (child == nullptr)
				{
					child = new SyncMDDNode((*cpyChild)->location, (*node));
					levels[t + 1].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}

	}
}

SyncMDDNode* SyncMDD::find(int location, int level) const
{
	if (level < (int)levels.size())
		for (auto it : levels[level])
			if (it->location == location)
				return it;
	return nullptr;
}

void SyncMDD::deleteNode(SyncMDDNode* node, int level)
{
	levels[level].remove(node);
	for (auto child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if ((*child)->parents.empty())
			deleteNode(*child, level + 1);
	}
	for (auto parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent, level - 1);
	}
}


void SyncMDD::clear()
{
	if (levels.empty())
		return;
	for (auto & level : levels)
	{
		for (auto & it : level)
			delete it;
	}
	levels.clear();
}


SyncMDD::~SyncMDD()
{
	clear();
}




MDD * MDDTable::getMDD(CBSNode& node, int id, size_t mdd_levels)
{
	if (!lookupTable.empty())
	{
		ConstraintsHasher c(id, &node);
		auto got = lookupTable[c.a].find(c);
		if (got != lookupTable[c.a].end())
		{
			assert(got->second->levels.size() == mdd_levels);
			return got->second;
		}
		releaseMDDMemory(id);
	}

	clock_t t = clock();
	MDD * mdd = new MDD();
	ConstraintTable ct(initial_constraints[id]);
	ct.build(node, id);
	mdd->buildMDD(ct, mdd_levels, search_engines[id]);
	if (!lookupTable.empty())
	{
		ConstraintsHasher c(id, &node);
		lookupTable[c.a][c] = mdd;
	}
	runtime_build_MDDs += (double)(clock() - t) / CLOCKS_PER_SEC;
	return mdd;
}


void MDDTable::findSingletons(CBSNode& node, int agent, Path& path)
{
	auto mdd = getMDD(node, agent, path.size());
	for (size_t i = 0; i < mdd->levels.size(); i++)
		path[i].single = mdd->levels[i].size() == 1;
	if (lookupTable.empty())
		delete mdd;
}


void MDDTable::releaseMDDMemory(int id)
{
	if (id < 0 || lookupTable.empty() || (int)lookupTable[id].size() < max_num_of_mdds)
		return;
	int minLength = INT_MAX;
	for (auto mdd : lookupTable[id])
	{
		if ((int)mdd.second->levels.size() < minLength)
			minLength = mdd.second->levels.size();
	}
	for (auto mdd = lookupTable[id].begin(); mdd != lookupTable[id].end();)
	{
		if ((int)mdd->second->levels.size() == minLength)
		{
			delete mdd->second;
			mdd = lookupTable[id].erase(mdd);
			num_released_mdds++;
		}
		else
		{
			mdd++;
		}
	}
}


void MDDTable::clear()
{
	for (auto& mdds : lookupTable)
	{
		for (auto mdd : mdds)
		{
			delete mdd.second;
		}
	}
	lookupTable.clear();
	runtime_build_MDDs = 0;
	num_released_mdds = 0;
}