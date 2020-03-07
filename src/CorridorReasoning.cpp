#include "CorridorReasoning.h"
#include "Conflict.h"
#include <memory>
#include "SpaceTimeAStar.h"
#include "SIPP.h"

shared_ptr<Conflict> CorridorReasoning::run(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths,
	bool cardinal, const CBSNode& node)
{
	clock_t t = clock();
	auto corridor = findCorridorConflict(conflict, paths, cardinal, node);
	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
	return corridor;
}


shared_ptr<Conflict> CorridorReasoning::findCorridorConflict(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths,
	bool cardinal, const CBSNode& node)
{
    int a[2] = { conflict->a1, conflict->a2 };
    int  agent, loc1, loc2, timestep;
    constraint_type type;
    tie(agent, loc1, loc2, timestep, type) = conflict->constraint1.back();
    int curr = -1;
    if (instance.getDegree(loc1) == 2)
    {
        curr = loc1;
        if (loc2 >= 0)
            timestep--;
    }
    else if (loc2 >= 0 && instance.getDegree(loc2) == 2)
        curr = loc2;
    if (curr <= 0)
        return nullptr;

    int t[2];
    for (int i = 0; i < 2; i++)
        t[i] = getEnteringTime(*paths[a[i]], *paths[a[1-i]], timestep);
    if (t[0] > t[1])
    {
        int temp = t[0]; t[0] = t[1]; t[1] = temp;
        temp = a[0]; a[0] = a[1]; a[1] = temp;
    }
    int u[2];
    for (int i = 0; i < 2; i++)
        u[i] = paths[a[i]]->at(t[i]).location;
    if (u[0] == u[1])
        return nullptr;
    for (int i = 0; i < 2; i++)
    {
        bool found = false;
        for (int time = t[i]; time < (int)paths[a[i]]->size() && !found; time++)
        {
            if (paths[a[i]]->at(time).location == u[1 - i])
                found = true;
        }
        if (!found)
            return nullptr;
    }
    pair<int, int> edge; // one edge in the corridor
    int k = getCorridorLength(*paths[a[0]], t[0], u[1], edge);
	int t3, t3_, t4, t4_;
	if (usingSIPP)
	{
		pair<int, int> edge_empty = make_pair(-1, -1);
		ReservationTable rt1(initial_constraints[a[0]]);
		rt1.build(node, a[0]);
		t3 = getBypassLengthBySIPP(paths[a[0]]->front().location, u[1], edge_empty, rt1, INT_MAX);
		t3_ = getBypassLengthBySIPP(paths[a[0]]->front().location, u[1], edge, rt1, t3 + 2 * k + 1);
		ReservationTable rt2(initial_constraints[a[1]]);
		rt2.build(node, a[1]);
		t4 = getBypassLengthBySIPP(paths[a[1]]->front().location, u[0], edge_empty, rt2, INT_MAX);
		t4_ = getBypassLengthBySIPP(paths[a[1]]->front().location, u[0], edge, rt2, t3 + k + 1);
	}
	else
	{
		pair<int, int> edge_empty = make_pair(-1, -1);
		ConstraintTable ct1(initial_constraints[a[0]]);
		ct1.build(node, a[0]);
		t3 = getBypassLengthByAStar(paths[a[0]]->front().location, u[1], edge_empty, ct1, INT_MAX);
		t3_ = getBypassLengthByAStar(paths[a[0]]->front().location, u[1], edge, ct1, t3 + 2 * k + 1);
		ConstraintTable ct2(initial_constraints[a[1]]);
		ct2.build(node, a[1]);
		t4 = getBypassLengthByAStar(paths[a[1]]->front().location, u[0], edge_empty, ct2, INT_MAX);
		t4_ = getBypassLengthByAStar(paths[a[1]]->front().location, u[0], edge, ct2, t3 + k + 1);
	}
   
    if (abs(t3 - t4) <= k && t3_ > t3 && t4_ > t4)
    {
        shared_ptr<Conflict> corridor = make_shared<Conflict>();
        corridor->corridorConflict(a[0], a[1], u[1], u[0], t3, t4, t3_, t4_, k);
        if (blocked(*paths[corridor->a1], corridor->constraint1.front()) && 
			blocked(*paths[corridor->a2], corridor->constraint2.front()))
            return corridor;
    }

    return nullptr;
}


int CorridorReasoning::getEnteringTime(const vector<PathEntry>& path, const vector<PathEntry>& path2, int t)
{
	if (t >= (int)path.size())
		t = (int)path.size() - 1;
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.back().location &&
		instance.getDegree(loc) == 2)
	{
		t--;
		loc = path[t].location;
	}
	return t;
}

int CorridorReasoning::getCorridorLength(const vector<PathEntry>& path, int t_start, int loc_end, pair<int, int>& edge)
{
	int curr = path[t_start].location;
	int next;
	int prev = -1;
	int length = 0; // distance to the start location
	int t = t_start;
	bool moveForward = true;
	bool updateEdge = false;
	while (curr != loc_end)
	{
		t++;
		next = path[t].location;
		if (next == curr) // wait
			continue;
		else if (next == prev) // turn aournd
			moveForward = !moveForward;
		if (moveForward)
		{
			if (!updateEdge)
			{
				edge = make_pair(curr, next);
				updateEdge = true;
			}
			length++;
		}
		else
			length--;
		prev = curr;
		curr = next;
	}
	return length;
}

/*int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::pairing_heap< AStarNode*, boost::heap::compare<AStarNode::compare_node> > heap;
	// boost::heap::pairing_heap< AStarNode*, boost::heap::compare<AStarNode::compare_node> >::handle_type open_handle;
	unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> nodes;

	auto root = new AStarNode(start, 0, getMahattanDistance(start, end, num_col), nullptr, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	int moves_offset[4] = { 1, -1, num_col, -num_col };
	AStarNode* curr = nullptr;
	while (!heap.empty())
	{
		curr = heap.top();
		heap.pop();
		if (curr->loc == end)
		{
			length = curr->g_val;
			break;
		}
		for (int direction = 0; direction < 4; direction++)
		{
			int next_loc = curr->loc + moves_offset[direction];
			if (validMove(curr->loc, next_loc, num_col, map_size) && !my_map[next_loc])
			{  // if that grid is not blocked
				if ((curr->loc == blocked.first && next_loc == blocked.second) ||
					(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
				{
					continue;
				}
				int next_g_val = curr->g_val + 1;
				auto next = new LLNode(next_loc, next_g_val, getMahattanDistance(next_loc, end, num_col), nullptr, 0);
				auto it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					open_handle = (*it)->open_handle;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	for (auto node : nodes)
	{
		delete node;
	}
	return length;
}*/

// run space-time A* to find the length of the shortest path between start and goal without using the blocked edge from either direction.
// if the length is longer than the upper bound, then give up.
int CorridorReasoning::getBypassLengthByAStar(int start, int end, pair<int, int> blocked,
	const ConstraintTable& constraint_table, int upper_bound)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	pairing_heap< AStarNode*, compare<AStarNode::compare_node> > open_list;
	// boost::heap::pairing_heap< AStarNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> nodes;
	auto root = new AStarNode(start, 0, instance.getManhattanDistance(start, end), nullptr, 0);
	root->open_handle = open_list.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	AStarNode* curr = nullptr;
	while (!open_list.empty())
	{
		curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (curr->location == next_location)
				{
					continue;
				}
				next_timestep--;
			}
			if ((curr->location == blocked.first && next_location == blocked.second) ||
				(curr->location == blocked.second && next_location == blocked.first)) // use the prohibited edge
			{
				continue;
			}
			if (!constraint_table.constrained(next_location, next_timestep) &&
				!constraint_table.constrained(curr->location, next_location, next_timestep))
			{  // if that grid is not blocked
				int next_h_val = instance.getManhattanDistance(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new AStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep);
				auto it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.increase(existing_next->open_handle);
					}
				}
			}
		}
	}
	open_list.clear();
	for (auto node: nodes)
	{
		delete node;
	}
	nodes.clear();
	return length;
}


int CorridorReasoning::getBypassLengthBySIPP(int start, int end, pair<int, int> blocked,
	ReservationTable& reservation_table, int upper_bound)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	pairing_heap< SIPPNode*, compare<SIPPNode::compare_node> > open_list;
	// boost::heap::pairing_heap< AStarNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	unordered_set<SIPPNode*, SIPPNode::NodeHasher, SIPPNode::eqnode> nodes;
	
	Interval interval = reservation_table.get_first_safe_interval(start);
	assert(get<0>(interval) == 0);
	auto root = new SIPPNode(start, 0, instance.getManhattanDistance(start, end), nullptr, 0, interval);
	root->open_handle = open_list.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)

	while (!open_list.empty())
	{
		auto curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		for (int next_location : instance.getNeighbors(curr->location))
		{
			if ((curr->location == blocked.first && next_location == blocked.second) ||
				(curr->location == blocked.second && next_location == blocked.first)) // use the prohibited edge
			{
				continue;
			}

			for (auto interval : reservation_table.get_safe_intervals(
				curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
			{
				int next_timestep = max(curr->timestep + 1, (int)get<0>(interval));
				int next_g_val = next_timestep;
				int next_h_val = instance.getManhattanDistance(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new SIPPNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, interval);
				auto it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.update(existing_next->open_handle);
					}
				}
			}
		}
	}
	open_list.clear();
	for (auto node : nodes)
	{
		delete node;
	}
	nodes.clear();
	return length;
}

bool CorridorReasoning::blocked(const Path& path, const Constraint& constraint)
{
	int a, loc, t1, t2;
	constraint_type type;
	tie(a, loc, t1, t2, type) = constraint;
	assert(type == constraint_type::RANGE);
	for (int t = t1; t < t2; t++)
	{
		if (t >= (int)path.size() && loc == path.back().location)
			return true;
		else if (t >= 0 && path[t].location == loc)
			return true;
	}
	return false;
}