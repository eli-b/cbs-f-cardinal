#include "CorridorReasoning.h"
#include "Conflict.h"
#include <memory>
#include "LLNode.h"

std::shared_ptr<Conflict> findCorridorConflict(const std::shared_ptr<Conflict>& con,
        const vector<Path*>& paths,
        const vector<ConstraintTable>& initial_constraints,
        bool cardinal, ICBSNode* node,
        const bool* my_map, int num_col, int map_size)
{
    int a[2] = {con->a1, con->a2};
    int  loc1, loc2, timestep;
    constraint_type type;
    std::tie(loc1, loc2, timestep, type) = con->constraint1.back();
    int curr = -1;
    if (getDegree(loc1, my_map, num_col, map_size) == 2)
    {
        curr = loc1;
        if (loc2 >= 0)
            timestep--;
    }
    else if (getDegree(loc2, my_map, num_col, map_size) == 2)
        curr = loc2;
    if (curr <= 0)
        return nullptr;

    int t[2];
    for (int i = 0; i < 2; i++)
        t[i] = getEnteringTime(*paths[a[i]], *paths[a[1-i]], timestep, my_map, num_col, map_size);
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
    std::pair<int, int> edge; // one edge in the corridor
    int k = getCorridorLength(*paths[a[0]], t[0], u[1], edge);

    std::pair<int, int> edge_empty = make_pair(-1, -1);
    ConstraintTable ct1(initial_constraints[a[0]]);
    node->getConstraintTable(ct1, a[0], num_col, map_size);
    int t3 = getBypassLength(paths[a[0]]->front().location, u[1], edge_empty, my_map, num_col, map_size, ct1, INT_MAX);
    int t3_ = getBypassLength(paths[a[0]]->front().location, u[1], edge, my_map, num_col, map_size, ct1, t3 + 2 * k  + 1);
    ConstraintTable ct2(initial_constraints[a[1]]);
    node->getConstraintTable(ct2, a[1], num_col, map_size);
    int t4 = getBypassLength(paths[a[1]]->front().location, u[0], edge_empty, my_map, num_col, map_size, ct2, INT_MAX);
    int t4_ = getBypassLength(paths[a[1]]->front().location, u[0], edge, my_map, num_col, map_size, ct2, t3 + k + 1);
    if (abs(t3 - t4) <= k && t3_ > t3 && t4_ > t4)
    {
        std::shared_ptr<Conflict> corridor = std::make_shared<Conflict>();
        corridor->corridorConflict(a[0], a[1], u[1], u[0], t3, t4, t3_, t4_, k);
        if (blocked(*paths[corridor->a1], corridor->constraint1, num_col) && blocked(*paths[corridor->a2], corridor->constraint2, num_col))
            return corridor;
    }

    return nullptr;
}

/*bool validMove(int curr, int next, int map_cols, int map_size)
{
	if (next < 0 || next >= map_size)
		return false;
	return getMahattanDistance(curr, next, map_cols) < 2;
}*/

int getMahattanDistance(int loc1, int loc2, int map_cols)
{
	int loc1_x = loc1 / map_cols;
	int loc1_y = loc1 % map_cols;
	int loc2_x = loc2 / map_cols;
	int loc2_y = loc2 % map_cols;
	return std::abs(loc1_x - loc2_x) + std::abs(loc1_y - loc2_y);
}


int getDegree(int loc, const bool*map, int num_col, int map_size)
{
	if (loc < 0 || loc >= map_size || map[loc])
		return -1;
	int degree = 0;
	if (0 < loc - num_col && !map[loc - num_col])
		degree++;
	if (loc + num_col < map_size && !map[loc + num_col])
		degree++;
	if (loc % num_col > 0 && !map[loc - 1])
		degree++;
	if (loc % num_col < num_col - 1 && !map[loc + 1])
		degree++;
	return degree;
}

int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	const bool*map, int num_col, int map_size)
{
	if (t >= (int)path.size())
		t = (int)path.size() - 1;
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.back().location &&
		getDegree(loc, map, num_col, map_size) == 2)
	{
		t--;
		loc = path[t].location;
	}
	return t;
}

int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge)
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
				edge = std::make_pair(curr, next);
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

int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> nodes;

	auto root = new LLNode(start, 0, getMahattanDistance(start, end, num_col), nullptr, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	int moves_offset[4] = { 1, -1, num_col, -num_col };
	LLNode* curr = nullptr;
	while (!heap.empty())
	{
		curr = heap.top();
		heap.pop();
		if (curr->loc == end)
		{
			length = curr->g_val;
			break;
		}
		for (int direction = 0; direction < 5; direction++)
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
				auto next = new LLNode(next_loc, next_g_val, getMahattanDistance(next_loc, end, num_col), NULL, 0);
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
}


int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size, ConstraintTable& constraint_table, int upper_bound)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> nodes;
	auto root = new LLNode(start, 0, getMahattanDistance(start, end, num_col), nullptr, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	int moves_offset[5] = { 1, -1, num_col, -num_col, 0};
	LLNode* curr = nullptr;
	while (!heap.empty())
	{
		curr = heap.top();
		heap.pop();
		if (curr->loc == end)
		{
			length = curr->g_val;
			break;
		}
		for (int direction = 0; direction < 5; direction++)
		{
			int next_loc = curr->loc + moves_offset[direction];
			int next_timestep = curr->timestep + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (direction == 4)
				{
					continue;
				}
				next_timestep--;
			}
			
			if (validMove(curr->loc, next_loc, num_col, map_size) && !my_map[next_loc] && 
				!constraint_table.is_constrained(next_loc, next_timestep) &&
				!constraint_table.is_constrained(curr->loc * map_size + next_loc, next_timestep))
			{  // if that grid is not blocked
				if ((curr->loc == blocked.first && next_loc == blocked.second) ||
					(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
				{
					continue;
				}
				int next_g_val = curr->g_val + 1;
				int next_h_val = getMahattanDistance(next_loc, end, num_col);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto* next = new LLNode(next_loc, next_g_val, next_h_val, nullptr, next_timestep);
				auto it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = *it;
					open_handle = (*it)->open_handle;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	for (auto node: nodes)
	{
		delete node;
	}
	return length;
}


bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)
{
	if (cons == nullptr)
		return false;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons->size()))
	{
		for (const auto & it : cons->at(next_timestep))
		{
			if ((std::get<0>(it) == next_id && std::get<1>(it) < 0)//vertex constraint
				|| (std::get<0>(it) == curr_id && std::get<1>(it) == next_id)) // edge constraint
				return true;
		}
	}
	return false;
}