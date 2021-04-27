#include "ConflictAvoidanceTable.h"
#include "Instance.h"  // For WAIT_MOVE

#include <new>  // For placement new

//std::uses_allocator<DeleterFromPool, DeleterFromPool::allocator_type> instantiation;

boost::pool<> avoidance_state_pool(sizeof(AvoidanceState));
DeleterFromPool avoidanceStatePoolDeleter(avoidance_state_pool);

//AvoidanceState state;
//std::unique_ptr<AvoidanceState, DeleterFromPool> try1((AvoidanceState*) avoidance_state_pool.malloc(), avoidanceStatePoolDeleter);
//std::tuple<int, std::unique_ptr<AvoidanceState, DeleterFromPool>> try2(
//		std::allocator_arg, std::allocator<std::tuple<int, std::unique_ptr<AvoidanceState, DeleterFromPool>>>(),
//		1, (AvoidanceState*) avoidance_state_pool.malloc(), avoidanceStatePoolDeleter);
//std::pmr::list<std::tuple<int, std::unique_ptr<AvoidanceState, DeleterFromPool>>> try3;

// Return the number of conflicts with the known_paths (by looking at the conflict avoidance table)
// for the move [curr_id,next_id], entering next_id at next_timestep.
int ConflictAvoidanceTable::num_conflicts_for_step(int curr_id, int next_id, int next_timestep) const
{
    int retVal = 0;

    // Check if next_id collides with an agent that reached its goal
    auto goal_it = at_goal.find(next_id);
    if (goal_it != at_goal.end())
    {
        int other_agent_first_timestep = goal_it->second;

        if (other_agent_first_timestep <= next_timestep)  // next_id was found in the map for the last timestep in the plans of another agent
            retVal += 1;
    }

    // Check for other collisions
    auto [found, state] = toward_goal.get(next_id, next_timestep);
    if (found)
    {
        retVal += state->vertex;
        // check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
        for (int i = 0; i < Instance::WAIT_MOVE; i++) // the wait action cannot lead to edge conflicts
        {
            if (next_id - curr_id == moves_offset[i])
            {
                retVal += state->edge[i];
                break;
            }
        }
    }

    return retVal;
}

int ConflictAvoidanceTable::latest_vertex_entry(int location) const
{
    // Check if location collides with an agent that reached its goal
    auto goal_it = at_goal.find(location);
    if (goal_it != at_goal.end())
    {
        return std::numeric_limits<int>::max();  // This vertex will always be occupied after a certain time step
    }

    auto [found, t] = toward_goal.latest_entry(location);
    return t;
}


int ConflictAvoidanceTable::latest_entry()
{
    int latest = -1;
    for (auto [location, time]: at_goal) {
        if (time > latest)
            latest = time;
    }
    return latest;
    //FIXME: This assumes entries are sane - no toward_goal entry is later than all at_goal entries
}


void ConflictAvoidanceTable::add_action(int timestep, int from, int to)
{
    auto [found_from, from_entry] = toward_goal.get(from, timestep);  // Yes, not at timestep - 1!
    if (!found_from)
    {
        from_entry = new (avoidance_state_pool.malloc()) AvoidanceState();
        toward_goal.set(from, timestep, from_entry);
    }
    auto [found_to, to_entry] = toward_goal.get(to, timestep);
    if (!found_to)
    {
        to_entry = new (avoidance_state_pool.malloc()) AvoidanceState();
        toward_goal.set(to, timestep, to_entry);
    }
    if (to_entry->vertex < 255)
        to_entry->vertex++;
//    for (int i = 0; i <= MapLoader::valid_moves_t::WAIT_MOVE; i++)
//    {
//        if (from - to == moves_offset[i]) {
//            if (from_entry->edge[i] < 255)
//                from_entry->edge[i]++;
//            break;
//        }
//    }
    // TODO: Have a small table mapping from move offsets to their index and use it instead of iterating
}

void ConflictAvoidanceTable::add_wait_at_goal(int timestep, int location) {
    at_goal[location] = timestep;
}

void ConflictAvoidanceTable::remove_wait_at_goal(int timestep, int location) {
    at_goal.erase(location);
}

void ConflictAvoidanceTable::remove_action(int timestep, int from, int to)
{
    auto [found_from, from_entry] = toward_goal.get(from, timestep);  // Yes, not at timestep - 1!
    if (!found_from)
    {
        std::cerr << "Vertex " << from << " at timestep " << timestep << " is not in the CAT" << std::endl;
        std::abort();
    }
    auto [found_to, to_entry] = toward_goal.get(to, timestep);
    if (!found_to)
    {
        std::cerr << "Vertex " << to << " at timestep " << timestep << " is not in the CAT" << std::endl;
        std::abort();
    }
    if (to_entry->vertex > 0)
        to_entry->vertex--;
    else {
        std::cerr << "Path already removed?? Vertex " << to << " at timestep " << timestep << " is not in the CAT" << std::endl;
        std::abort();
    }
//    for (int i = 0; i < MapLoader::valid_moves_t::WAIT_MOVE; i++)
//    {
//        if (from - to == moves_offset[i]) {
//            if (from_entry->edge[i] > 0)
//                from_entry->edge[i]--;
//            else {
//                std::cerr << "Path already removed?? Edge from " << from << " to " << to << " at timestep " << timestep << " is not in the CAT" << std::endl;
//                std::abort();
//            }
//            break;
//        }
//    }
    // TODO: Have a small table mapping from move offsets to their index and use it instead of iterating
}

void ConflictAvoidanceTable::addPath(Path &path) {
	int first_wait_at_goal_timestep = path.size();
	for (size_t j = path.size() - 2; j >= 0 ; --j) {
		if (path[j].location == path[j + 1].location)
			--first_wait_at_goal_timestep;
		else
			break;
	}
	this->add_wait_at_goal(first_wait_at_goal_timestep, path.back().location);
	for (size_t timestep = 1; timestep < path.size(); timestep++)
	{
		int to = path[timestep].location;
		int from = path[timestep - 1].location;
		this->add_action(timestep, from, to);
	}
}


void ConflictAvoidanceTable::removePath(Path &path) {
	int first_wait_at_goal_timestep = path.size();
	this->remove_wait_at_goal(first_wait_at_goal_timestep, path.back().location);
	for (size_t timestep = 1; timestep < path.size(); timestep++)
	{
		int to = path[timestep].location;
		int from = path[timestep - 1].location;
		this->remove_action(timestep, from, to);
	}
}
