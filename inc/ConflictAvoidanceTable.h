#pragma once

#include <unordered_map>
#include <memory_resource>  // For pmr
#include <boost/pool/pool.hpp>
#include "XytHolder.h"
#include "common.h"


struct AvoidanceState
{
    uint8_t vertex = 0;
    uint8_t edge[5] = { 0, 0, 0, 0, 0 };
};

extern boost::pool<> avoidance_state_pool;
extern DeleterFromPool avoidanceStatePoolDeleter;

//extern std::unique_ptr<AvoidanceState, DeleterFromPool> try1;
//extern std::tuple<int, std::unique_ptr<AvoidanceState, DeleterFromPool>> try2;
//extern std::pmr::list<std::tuple<int, std::unique_ptr<AvoidanceState, DeleterFromPool>>> try3;


class ConflictAvoidanceTable {
public:
    const int* moves_offset;

    explicit ConflictAvoidanceTable(const int* moves_offset, int map_size)
           : moves_offset(moves_offset), toward_goal(map_size, avoidanceStatePoolDeleter)
			//: moves_offset(moves_offset), toward_goal(map_size, avoidance_state_pool_deleter)
		   {}

    ConflictAvoidanceTable(ConflictAvoidanceTable&& other)
    	  : toward_goal(std::move(other.toward_goal)),
			at_goal(std::move(other.at_goal)),
			moves_offset(other.moves_offset)
		{}

    XytHolder<AvoidanceState, DeleterFromPool> toward_goal;  // maps location+time pairs to their avoidance state.
	//XytHolder<AvoidanceState, std::function<void(AvoidanceState*)>> toward_goal;  // maps location+time pairs to their avoidance state.
															 // For every non-WAIT action, we'll have to access both the from location
															 // and the to location, but that's ok because a) they're relatively close
															 // so locality would be on our side, and b) it's very likely that we accessed
															 // the from location earlier, as we're dealing with paths.
    std::unordered_map<int, int> at_goal;  // Maps locations to when agents reach them and never leave (because the location is their goal)

    virtual int num_conflicts_for_step(int curr_id, int next_id, int next_timestep) const;
    virtual int latest_vertex_entry(int location) const;
    virtual void add_action(int timestep, int from, int to);
    virtual void remove_action(int timestep, int from, int to);
    virtual void add_wait_at_goal(int timestep, int location);
    virtual void remove_wait_at_goal(int timestep, int location);
    virtual void addPath(Path &path);
    virtual void removePath(Path &path);

    int latest_entry();    
};

class EmptyConflictAvoidanceTable : public ConflictAvoidanceTable {
public:
    explicit EmptyConflictAvoidanceTable() : ConflictAvoidanceTable(nullptr, 0) {}
    int num_conflicts_for_step(int curr_id, int next_id, int next_timestep) const { return 0; }
    void add_action(int timestep, int from, int to) {}
    void remove_action(int timestep, int from, int to) {}
    void add_wait_at_goal(int timestep, int location) {}
    void remove_wait_at_goal(int timestep, int location) {}
};
