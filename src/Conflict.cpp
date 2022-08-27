#include "Conflict.h"
#include "RectangleReasoning.h"
#include "MDD.h"


std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
	   std::get<2>(constraint) << "," << std::get<3>(constraint) << ",";
	switch (get<4>(constraint))
	{
	case constraint_type::VERTEX:
		os << "V";
		break;
	case constraint_type::POSITIVE_VERTEX:
		os << "V+";
		break;
	case constraint_type::EDGE:
		os << "E";
		break;
	case constraint_type::POSITIVE_EDGE:
		os << "E+";
		break;
	case constraint_type::BARRIER:
		os << "B";
		break;
	case constraint_type::RANGE_VERTEX:
		os << "RV";
		break;
	case constraint_type::RANGE_EDGE:
		os << "RE";
		break;
	case constraint_type::GLENGTH:
		os << "G";
		break;
	case constraint_type::LEQLENGTH:
		os << "L";
		break;
	case constraint_type::CONSTRAINT_TYPE_COUNT:
		break;
	}
	os << ">";
	return os;
}


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	switch (conflict.priority)
	{
	case conflict_priority::F_CARDINAL_G_CARDINAL:
		os << "f-cardinal-g-cardinal ";
		break;
	case conflict_priority::SEMI_F_CARDINAL_G_CARDINAL:
		os << "semi-f-cardinal-g-cardinal ";
		break;
	case conflict_priority::G_CARDINAL:
		os << "g-cardinal ";
		break;
	case conflict_priority::PSEUDO_G_CARDINAL_SEMI_G_CARDINAL:
		os << "pseudo-g-cardinal-semi-g-cardinal ";
		break;
	case conflict_priority::SEMI_G_CARDINAL:
		os << "semi-g-cardinal ";
		break;
	case conflict_priority::PSEUDO_G_CARDINAL_NON_G_CARDINAL:
		os << "pseudo-g-cardinal-non-g-cardinal ";
		break;
	case conflict_priority::NON_G_CARDINAL:
		os << "non-g-cardinal ";
		break;
	case conflict_priority::UNKNOWN:
		break;
	case conflict_priority::PRIORITY_COUNT:
		break;
	}
	switch (conflict.type)
	{
	case conflict_type::STANDARD:
		os << "standard";
		break;
	case conflict_type::RECTANGLE:
		os << "rectangle";
		break;
	case conflict_type::CORRIDOR:
		os << "corridor";
		break;
	case conflict_type::TARGET:
		os << "target";
		break;
	case conflict_type::MUTEX:
		os << "mutex";
		break;
	case conflict_type::TYPE_COUNT:
		break;
	}
	os << " conflict:  " << conflict.a1 << " with ";
	for (auto con : conflict.constraint1)
		os << con << ",";
	os << " and " << conflict.a2 << " with ";
	for (auto con : conflict.constraint2)
		os << con << ",";
	return os;
}


// conflict selection (larger is *better*)
// First compare the cardinality: f-cardinal > semi-f-cardinal > g-cardinal > semi-g-cardinal > non-cardinal (This step can be skipped by the user)
// (Note the better/more preferable values are *lower* in the enum)
// Second compare the type: mutex > target > corridor > rectangle > vertex/edge
// Third compare the user-specified tie-breaking rule: RANDOM, EARLIEST, CONFLICTS, MCONSTRAINTS, FCONSTRAINTS, WIDTH, SINGLETONS
// Last break ties randomly
// For all the values below, smaller is better
bool operator<(const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{
	if (conflict1.priority == conflict2.priority ||
		(conflict1.priority == conflict_priority::PSEUDO_G_CARDINAL_SEMI_G_CARDINAL && conflict2.priority == conflict_priority::SEMI_G_CARDINAL) ||
		(conflict2.priority == conflict_priority::PSEUDO_G_CARDINAL_SEMI_G_CARDINAL && conflict1.priority == conflict_priority::SEMI_G_CARDINAL) ||
		(conflict1.priority == conflict_priority::PSEUDO_G_CARDINAL_NON_G_CARDINAL && conflict2.priority == conflict_priority::NON_G_CARDINAL) ||
		(conflict2.priority == conflict_priority::PSEUDO_G_CARDINAL_NON_G_CARDINAL && conflict1.priority == conflict_priority::NON_G_CARDINAL)
	)  // TODO: Enable distinguishing between pseudo-g-cardinal conflicts and their origin priority based on the
	   //       conflict_prioritization value that was chosen.
	{
		if (conflict1.type == conflict2.type)
		{
			if (conflict1.secondary_priority == conflict2.secondary_priority)
			{
				return 3 * std::hash<int>()(conflict1.a1) + 5 * std::hash<int>()(conflict1.a2) <=
					   3 * std::hash<int>()(conflict2.a1) + 5 * std::hash<int>()(conflict2.a2);  // Return an arbitrary consistent answer
			}
			return conflict1.secondary_priority > conflict2.secondary_priority;
		}
		return conflict1.type > conflict2.type;
	}
	return conflict1.priority > conflict2.priority;
}

