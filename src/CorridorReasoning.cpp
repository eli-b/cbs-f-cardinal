#include "CorridorReasoning.h"
#include "Conflict.h"
#include <memory>
#include "SpaceTimeAStar.h"
#include "SIPP.h"

// Either returns nullptr if the given conflict isn't a corridor conflict, else returns the conflict as a corridor conflict
shared_ptr<Conflict> CorridorReasoning::run(const shared_ptr<Conflict>& conflict,
											const vector<Path*>& paths,
											bool cardinal, const CBSNode& node)
{
	clock_t t = clock();
	auto corridor_conflict = findCorridorConflict(conflict, paths, cardinal, node);
	accumulated_runtime += (double) (clock() - t) / CLOCKS_PER_SEC;
	return corridor_conflict;
}

// Returns the length of the corridor
int CorridorReasoning::findCorridor(const shared_ptr<Conflict>& conflict,
									const vector<Path*>& paths, int endpoints[], int endpoints_time[])
{
	if (paths[conflict->a1]->size() <= 1 || paths[conflict->a2]->size() <= 1)
		return 0;
	assert(conflict->constraint1.size() == 1);
	auto [agent, loc2, loc1, t, type] = conflict->constraint1.back();
	if (t < 1)
		return 0;
	if (loc1 < 0) // vertex conflict
	{
		if (search_engines[0]->instance.getDegree(loc2) != 2)
			return 0; // not a corridor 
		loc1 = loc2;
	}
	else // edge conflict
	{
		if (search_engines[0]->instance.getDegree(loc1) != 2 && search_engines[0]->instance.getDegree(loc2) != 2)
			return 0; // not a corridor 	
	}

	endpoints_time[0] = getExitingTime(*paths[conflict->a1], t); // the first timestep when agent 1 exits the corridor
	endpoints_time[1] = getExitingTime(*paths[conflict->a2], t); // the first timestep when agent 2 exits the corridor
	endpoints[0] = paths[conflict->a1]->at(endpoints_time[0]).location; // the exit location for agent 1
	endpoints[1] = paths[conflict->a2]->at(endpoints_time[1]).location; // the exit location for agent 2
	if (endpoints[0] == endpoints[1]) // agents exit the corridor in the same direction
		return 0;
	// count the distance between the two endpoints, and
	// check whether the corridor between the two exit locations traverse the conflict location, 
	// which indicates whether the two agents come in different directions
	int prev = endpoints[0];
	int curr = paths[conflict->a1]->at(endpoints_time[0] - 1).location;
	bool traverseTheConflictingLocation = false;
	int corridor_length = 1;
	while (curr != endpoints[1])
	{
		if (curr == loc2)
			traverseTheConflictingLocation = true;
		auto neighbors = search_engines[0]->instance.getNeighbors(curr);
		if (neighbors.size() == 2) // inside the corridor
		{
			if (neighbors.front() == prev)
			{
				prev = curr;
				curr = neighbors.back();
			}
			else
			{
				assert(neighbors.back() == prev);
				prev = curr;
				curr = neighbors.front();
			}
		}
		else // endpoint of the corridor
		{
			assert(std::find(neighbors.begin(), neighbors.end(), endpoints[1]) != neighbors.end()); // endpoint2 must be in neighbors.
			prev = curr;
			curr = endpoints[1];
		}
		corridor_length++;
	}

	// When k=2, it might just be a corner cell, which we do not want to recognize as a corridor
	if (corridor_length == 2 &&
		search_engines[0]->instance.getColCoordinate(endpoints[0]) != search_engines[0]->instance.getColCoordinate(endpoints[1]) &&
		search_engines[0]->instance.getRowCoordinate(endpoints[0]) != search_engines[0]->instance.getRowCoordinate(endpoints[1]))
	{
		return 0;
	}
	return corridor_length;
}


// Returns the conflict as a (generalized cardinal) corridor conflict, or nullptr if it isn't
shared_ptr<Conflict> CorridorReasoning::findCorridorConflict(const shared_ptr<Conflict>& conflict,
															 const vector<Path*>& paths,
															 bool cardinal, const CBSNode& node)
{
	int a[2] = { conflict->a1, conflict->a2 };
	auto [agent, loc1, loc2, timestep, type] = conflict->constraint1.back();

	if (search_engines[0]->instance.getDegree(loc1) == 2)  // The conflict is in a corridor
	{
		if (loc2 >= 0)
			timestep--;  // The timestep of an edge conflict is the one after it occurs
	}
	else if (loc2 >= 0 && search_engines[0]->instance.getDegree(loc2) == 2)  // It's an edge conflict between an entrance to a corridor and the start of the corridor
	{}
	else  // Not a corridor conflict
		return nullptr;

	int t[2];  // The time each agent reaches an entrance of the corridor
	for (int i = 0; i < 2; i++)
		t[i] = getEnteringTime(*paths[a[i]], *paths[a[1 - i]], timestep);
	if (t[0] > t[1])  // then swap the agents
	{
		int temp = t[0]; t[0] = t[1]; t[1] = temp;
		temp = a[0]; a[0] = a[1]; a[1] = temp;
	}
	int entrance_vertex[2];  // The respective entrances of the corridor
	for (int i = 0; i < 2; i++)
		entrance_vertex[i] = (*paths[a[i]])[t[i]].location;
	if (entrance_vertex[0] == entrance_vertex[1])
		return nullptr;
	// Make sure each agent passes all the way through the corridor:
	for (int i = 0; i < 2; i++)
	{
		bool found = false;
		for (int time = t[i]; time < (int) paths[a[i]]->size() && !found; time++)
		{
			if ((*paths[a[i]])[time].location == entrance_vertex[1 - i])
				found = true;
		}
		if (!found)
			return nullptr;
	}
	pair<int, int> entry_edge1;
	int corridor_length = getCorridorLength(*paths[a[0]], t[0], entrance_vertex[1], entry_edge1);
	int t3, t4;  // The time each agent reaches its exit of the corridor
	ConstraintTable ct1(initial_constraints[a[0]]);
	t3 = search_engines[a[0]]->getTravelTime(paths[a[0]]->front().location, entrance_vertex[1], ct1, MAX_TIMESTEP);  // Can't just use t[0] + corridor_length. The agent might wait in the corridor or change directions
																														// Why not take it from the agent's path though?
	ConstraintTable ct2(initial_constraints[a[1]]);
	t4 = search_engines[a[1]]->getTravelTime(paths[a[1]]->front().location, entrance_vertex[0], ct2, MAX_TIMESTEP);

	if (abs(t3 - t4) > corridor_length)  // One of the agents could have fully traversed the corridor before the other.
											// FIXME: Not necessarily. What's the explanation then?
		return nullptr;

	int t3_, t4_;  // The time each agent reaches its exit of the corridor without passing through the corridor
	int cost_lookahead1=-1, cost_lookahead2=-1;  // The time each agent reaches its *target* under the range constraint that would force it to let other agent pass first
	ct1.build(node, a[0]);
	// Block the entering edge of the corridor in both directions
	ct1.insert(entry_edge1.first, entry_edge1.second, 0, MAX_TIMESTEP);
	ct1.insert(entry_edge1.second, entry_edge1.first, 0, MAX_TIMESTEP);
	t3_ = search_engines[a[0]]->getTravelTime(paths[a[0]]->front().location, entrance_vertex[1], ct1,
								  t4 + corridor_length + 1);
	if (t3_ <= t3)  // The agent could have avoided the corridor
		return nullptr;

	ct2.build(node, a[1]);
	// Block the entering edge of the corridor in both directions
	ct1.insert(entry_edge1.first, entry_edge1.second, 0, MAX_TIMESTEP);
	ct1.insert(entry_edge1.second, entry_edge1.first, 0, MAX_TIMESTEP);
	t4_ = search_engines[a[1]]->getTravelTime(paths[a[1]]->front().location, entrance_vertex[0], ct2, t3 + corridor_length + 1);
	if (t4_ <= t4)  // The agent could have avoided the corridor
		return nullptr;

	int range_end1 = std::min(t3_ - 1, t4 + corridor_length);
	int range_end2 = std::min(t4_ - 1, t3 + corridor_length);

	if (calc_alt_cost == calc_alt_cost_variant::YES)
	{
		ConstraintTable ct1_c_(initial_constraints[a[0]]);
		ct1_c_.build(node, a[0]);
		ct1.insert(entrance_vertex[1], 0, range_end1); // Simulate the range constraint
		// until the other finishes crossing it
		cost_lookahead1 = search_engines[a[0]]->getTravelTime(paths[a[0]]->front().location, paths[a[0]]->back().location,
															  ct1_c_, MAX_TIMESTEP);

		ConstraintTable ct2_c_(initial_constraints[a[1]]);
		ct2_c_.build(node, a[1]);
		ct2.insert(entrance_vertex[0], 0, range_end2); // Simulate the range constraint
		cost_lookahead2 = search_engines[a[1]]->getTravelTime(paths[a[1]]->front().location, paths[a[1]]->back().location,
															  ct2_c_, MAX_TIMESTEP);
	}

	shared_ptr<Conflict> corridor_conflict = make_shared<Conflict>();
	corridor_conflict->corridorConflict(a[0], a[1], entrance_vertex[1], entrance_vertex[0],
										range_end1, range_end2, t3, t4,
										corridor_length, cost_lookahead1, cost_lookahead2, paths[a[0]]->size() - 1, paths[a[1]]->size() - 1);
	if (blocked(*paths[a[0]], corridor_conflict->constraint1.front()) &&
			blocked(*paths[a[1]], corridor_conflict->constraint2.front()))  // Make sure the constraints force a path change.
	{
		return corridor_conflict;
	}

	return nullptr;
}


int CorridorReasoning::getExitingTime(const std::vector<PathEntry>& path, int t)
{
	if (t >= (int) path.size())
		t = (int) path.size() - 1;
	int loc = path[t].location;
	while (loc != path.back().location &&
		   search_engines[0]->instance.getDegree(loc) == 2)
	{
		t++;
		loc = path[t].location;
	}
	return t;
}


// Returns time step in <path> that the agent steps into the entrance to the same corridor it is in in time step <t>.
// If the other agent's target is inside the corridor and the agent passes through it on the way to the conflict,
// the time step that this happens in is returned.
// TODO: Consider only passing the other agent's target location
int CorridorReasoning::getEnteringTime(const vector<PathEntry>& path, const vector<PathEntry>& path2, int t)
{
	if (t >= (int) path.size())
		t = (int) path.size() - 1;
	int loc = path[t].location;
	while (loc != path.front().location &&
		   loc != path2.back().location &&  // The other agent's target isn't inside the corridor
		   search_engines[0]->instance.getDegree(loc) == 2)
	{
		t--;
		loc = path[t].location;
	}
	return t;
}


// Also populates edge with the first edge in the path in the corridor where the agent is planned to move forward
int CorridorReasoning::getCorridorLength(const vector<PathEntry>& path, int t_start, int loc_end, pair<int, int>& edge)
{
	int curr = path[t_start].location;
	int next;
	int prev = -1;
	int length = 0; // distance to the start location
	int t = t_start;
	bool movingForward = true;
	bool updatedEdge = false;
	while (curr != loc_end)
	{
		t++;
		next = path[t].location;
		if (next == curr) // wait
			continue;
		else if (next == prev) // The agent turned around
			movingForward = !movingForward;
		if (movingForward)
		{
			if (!updatedEdge)
			{
				edge = make_pair(curr, next);
				updatedEdge = true;
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

bool CorridorReasoning::blocked(const Path& path, const Constraint& constraint)
{
	auto [a, loc, t1, t2, type] = constraint;
	assert(type == constraint_type::RANGE_EDGE || type == constraint_type::RANGE_VERTEX);
	for (int t = t1; t < t2; t++)
	{
		if (t >= (int) path.size() && loc == path.back().location)
			return true;
		else if (t >= 0 && path[t].location == loc)
			return true;
	}
	return false;
}