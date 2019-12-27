#pragma once
#include "common.h"
#include "MDD.h"

enum conflict_type { TARGET, CORRIDOR, RECTANGLE, STANDARD, TYPE_COUNT };
enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };

enum constraint_type { LENGTH, RANGE, BARRIER, VERTEX, EDGE, CONSTRAINT_COUNT };

typedef std::tuple<int, int, int, constraint_type> Constraint;
// <loc, -1, t, VERTEX>
// <from, to, t, EDGE> 
// <B1, B2, t, RECTANGLE>
// <loc, t1, t2, CORRIDOR> 
// <loc, agent_id, t, TARGET>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <-1, agent_id, t>: path of agent_id should be of length at least t + 1

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

/*
// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const MDD* mdd, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints);

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const MDD* mdd, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints);
 */

bool traverse(const Path& path, int loc, int t);

bool blocked(const Path& path, const std::list<Constraint>& constraints, int num_col);

class Conflict
{
public:
	int a1;
	int a2;
	int t;
	std::list<Constraint> constraint1;
	std::list<Constraint> constraint2;
	conflict_type type;
	conflict_priority p = conflict_priority::UNKNOWN;

	void vertexConflict(int a1, int a2, int v, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(v, -1, t, constraint_type::VERTEX);
		this->constraint2.emplace_back(v, -1, t, constraint_type::VERTEX);
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void corridorConflict(int a1, int a2, int v1, int v2, int t3, int t4, int t3_, int t4_, int k)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = std::min(t3, t4);
		this->constraint1.emplace_back(v1, t3, std::min(t3_ - 1, t4 + k), constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t4, std::min(t4_ - 1, t3 + k), constraint_type::RANGE);
		type = conflict_type::CORRIDOR;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
	                        const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
	                        const MDD* mdd1, const MDD* mdd2, int num_col); // For RM

	void targetConflict(int a1, int a2, int v, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->constraint1.emplace_back(-1, a1, t, constraint_type::LENGTH);
		this->constraint2.emplace_back(v, a1, t, constraint_type::LENGTH);
		type = conflict_type::TARGET;
	}

    // add a horizontal modified barrier constraint
    static bool addModifiedHorizontalBarrierConstraint(const MDD* mdd, int x,
                                                int Ri_y, int Rg_y, int Rg_t, int num_col,
                                                std::list<Constraint>& constraints);

    // add a vertival modified barrier constraint
    static bool addModifiedVerticalBarrierConstraint(const MDD* mdd, int y,
                                              int Ri_x, int Rg_x, int Rg_t, int num_col,
                                              std::list<Constraint>& constraints);

};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);

