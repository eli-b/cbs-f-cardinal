#pragma once
#include "common.h"


enum conflict_type { TARGET, CORRIDOR, RECTANGLE, STANDARD, TYPE_COUNT };
enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };
enum constraint_type { LEQLENGTH, GLENGTH, RANGE, BARRIER, VERTEX, EDGE, 
											POSITIVE_VERTEX, POSITIVE_EDGE, CONSTRAINT_COUNT };

typedef std::tuple<int, int, int, int, constraint_type> Constraint;
// <agent, loc, -1, t, VERTEX>
// <agent, loc, -1, t, POSITIVE_VERTEX>
// <agent, from, to, t, EDGE> 
// <agent, B1, B2, t, RECTANGLE>
// <agent, loc, t1, t2, CORRIDOR> 
// <agent, loc, -1, t, LEQLENGTH>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <agent, loc, -1, t, GLENGTH>: path of agent_id should be of length at least t + 1

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


class Conflict
{
public:
	int a1;
	int a2;
	int t;
	list<Constraint> constraint1;
	list<Constraint> constraint2;
	conflict_type type;
	conflict_priority p = conflict_priority::UNKNOWN;

	void vertexConflict(int a1, int a2, int v, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::VERTEX);
		this->constraint2.emplace_back(a2, v, -1, t, constraint_type::VERTEX);
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(a1, v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(a2, v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void corridorConflict(int a1, int a2, int v1, int v2, int t3, int t4, int t3_, int t4_, int k)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = std::min(t3, t4);
		this->constraint1.emplace_back(a1, v1, t3, std::min(t3_ - 1, t4 + k), constraint_type::RANGE);
		this->constraint2.emplace_back(a2, v2, t4, std::min(t4_ - 1, t3 + k), constraint_type::RANGE);
		type = conflict_type::CORRIDOR;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
	                         int Rg_t, const list<Constraint>& constraint1, const list<Constraint>& constraint2) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->constraint1 = constraint1;
		this->constraint2 = constraint2;
		type = conflict_type::RECTANGLE;
		return true;
	}


	void targetConflict(int a1, int a2, int v, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::LEQLENGTH);
		this->constraint2.emplace_back(a1, v, -1, t, constraint_type::GLENGTH);
		type = conflict_type::TARGET;
	}



};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);

