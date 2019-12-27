#pragma once
#include "common.h"
#include "ConstraintTable.h"
#include "Conflict.h"
#include "ICBSNode.h"

// typedef std::tuple<int, int, int> CorridorTable_Key; // endpoint1, endpoint2, length
// typedef unordered_map<CorridorTable_Key, int, three_tuple_hash> CorridorTable; // value = length of the bypass

std::shared_ptr<Conflict> findCorridorConflict( const std::shared_ptr<Conflict>& con,
                                                const vector<Path*>& paths,
                                                const vector<ConstraintTable>& initial_constraints,
                                                bool cardinal, ICBSNode* node,
                                                const bool* my_map, int num_col, int map_size);

bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons);

int getDegree(int loc, const bool*map, int num_col, int map_size);

int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	const bool*map, int num_col, int map_size);
int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);


int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size);
int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size,
        ConstraintTable& constraint_table, int upper_bound);

int getMahattanDistance(int loc1, int loc2, int map_cols);