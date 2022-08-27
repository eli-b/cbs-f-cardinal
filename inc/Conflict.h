#pragma once

#include "common.h"


enum conflict_type { MUTEX, TARGET, CORRIDOR, RECTANGLE, STANDARD, TYPE_COUNT };

enum conflict_priority { F_CARDINAL_G_CARDINAL, F_CARDINAL_OTHERWISE,
						 SEMI_F_CARDINAL_G_CARDINAL, SEMI_F_CARDINAL_OTHERWISE, G_CARDINAL, PSEUDO_G_CARDINAL_SEMI_G_CARDINAL,
						 SEMI_G_CARDINAL, PSEUDO_G_CARDINAL_NON_G_CARDINAL, NON_G_CARDINAL, UNKNOWN, PRIORITY_COUNT };
// Pseudo-cardinal conflicts are semi-/non-cardinal conflicts between dependent agents.
// We prioritize them over normal semi-/non-cardinal conflicts

enum constraint_type
{
	LEQLENGTH, GLENGTH, RANGE_VERTEX, RANGE_EDGE, BARRIER, VERTEX, EDGE,
	POSITIVE_VERTEX, POSITIVE_EDGE, CONSTRAINT_COUNT
};

enum conflict_selection {RANDOM, EARLIEST, CONFLICTS, MCONSTRAINTS, FCONSTRAINTS, WIDTH, SINGLETONS};

typedef std::tuple<int, int, int, int, constraint_type> Constraint;
// <agent, loc, -1, t, VERTEX>
// <agent, loc, -1, t, POSITIVE_VERTEX>
// <agent, from, to, t, EDGE>
// <agent, from, to, t, POSITIVE_EDGE>
// <agent, B1, B2, t, BARRIER>
// <agent, loc, t1, t2, RANGE_VERTEX>
// <agent, from, to, t2, RANGE_EDGE>  // TODO: Add an int to Constraint to enable t1!=0
// <agent, loc, -1, t, LEQLENGTH>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <agent, loc, -1, t, GLENGTH>: path of agent_id should be of length at least t + 1

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


class Conflict
{
public:
	int a1;
	int a2;
	list<Constraint> constraint1;
	list<Constraint> constraint2;
	conflict_type type;
	conflict_priority priority = conflict_priority::UNKNOWN;
	double secondary_priority = 0; // used as the tie-breaking criterion for conflict selection

  // For mutex propagation
  int final_len_1;
  int final_len_2;

  // For mutex propagation in Non-cardinal
  // The actual MDD node/edge which was used to generated the biclique;
  int t1;
  int loc1;
  int loc1_to=-1;
  int t2;
  int loc2;
  int loc2_to=-1;

	// For NVW heuristics
	int a1_path_cost;
	int c1_lookahead;
	int c2_lookahead;
	int corridor_length;
	int c1;
	int c2;

	void vertexConflict(int a1, int a2, int v, int t, int a1_path_cost)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->a1_path_cost = a1_path_cost;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::VERTEX);
		this->constraint2.emplace_back(a2, v, -1, t, constraint_type::VERTEX);
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t, int a1_path_cost)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->a1_path_cost = a1_path_cost;
		this->constraint1.emplace_back(a1, v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(a2, v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void corridorConflict(int a1, int a2, int v1, int v2, int range_end1, int range_end2,
						  int t1, int t2, int corridor_length,
						  int c1_lookahead, int c2_lookahead,
						  int c1, int c2)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->c1_lookahead = c1_lookahead;
		this->c2_lookahead = c2_lookahead;
		this->t1 = t1;  // For debugging
		this->t2 = t2;  // For debugging
		this->c1 = c1;
		this->c2 = c2;
		this->corridor_length = corridor_length;  // For debugging
		this->constraint1.emplace_back(a1, v1, 0, range_end1, constraint_type::RANGE_VERTEX);
		this->constraint2.emplace_back(a2, v2, 0, range_end2, constraint_type::RANGE_VERTEX);
		type = conflict_type::CORRIDOR;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
						   int Rg_t, const list<Constraint>& constraint1, const list<Constraint>& constraint2) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1 = constraint1;
		this->constraint2 = constraint2;
		type = conflict_type::RECTANGLE;
		return true;
	}


	void targetConflict(int a1, int a2, int v, int t, int a1_path_cost)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->a1_path_cost = a1_path_cost;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::LEQLENGTH);  // This constraint is first to enable bypassing
																				   // to skip generating the second child
		this->constraint2.emplace_back(a1, v, -1, t, constraint_type::GLENGTH);
		type = conflict_type::TARGET;
	}


	void mutexConflict(int a1, int a2)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		type = conflict_type::MUTEX;
		priority = conflict_priority::G_CARDINAL;
		// TODO add constraints from mutex reasoning
	}
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator<(const Conflict& conflict1, const Conflict& conflict2);
