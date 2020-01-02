#pragma once
#include "CBSNode.h"


struct HTableEntry // look-up table entry 
{
	int a1{};
	int a2{};
	CBSNode* n{};

	HTableEntry()= default;
	HTableEntry(int a1, int a2, CBSNode* n): a1(a1), a2(a2), n(n) {};

	struct EqNode
	{
		bool operator() (const HTableEntry& h1, const HTableEntry& h2) const
		{
			std::set<Constraint> cons1[2], cons2[2];
			const CBSNode* curr = h1.n;
			while (curr->parent != nullptr)
			{
			    if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
					for (auto con : curr->constraints)
					{
						cons1[0].insert(con);
						cons2[0].insert(con);
					}
                } else {
                    if (get<0>(curr->constraints.front()) == h1.a1)
                        for (auto con : curr->constraints)
                            cons1[0].insert(con);
                    else if (get<0>(curr->constraints.front()) == h1.a2)
                        for (auto con : curr->constraints)
                            cons2[0].insert(con);
			    }

				curr = curr->parent;
			}
			curr = h2.n;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
					for (auto con : curr->constraints)
					{
						cons1[1].insert(con);
						cons2[1].insert(con);
					}
                } else {
                    if (get<0>(curr->constraints.front()) == h2.a1)
                        for (auto con : curr->constraints)
                            cons1[1].insert(con);
                    else if (get<0>(curr->constraints.front()) == h2.a2)
                        for (auto con : curr->constraints)
                            cons2[1].insert(con);
                }

                curr = curr->parent;
			}
			if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size())
				return false;

			if (!equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin()))
				return false;
            return equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin());
        }
	};
	

	struct Hasher
	{
		size_t operator()(const HTableEntry& entry) const
		{
			CBSNode* curr = entry.n;
			size_t cons1_hash = 0, cons2_hash = 0;
			while (curr->parent != nullptr)
			{
				if (get<0>(curr->constraints.front()) == entry.a1 ||
					get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons1_hash +=   3 * std::hash<int>()(std::get<0>(con)) +
						                5 * std::hash<int>()(std::get<1>(con)) +
						                7 * std::hash<int>()(std::get<2>(con)) +
                                        11 * std::hash<int>()(std::get<3>(con));
					}
				}
				else if (get<0>(curr->constraints.front()) == entry.a2 ||
					get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons2_hash +=   3 * std::hash<int>()(std::get<0>(con)) +
						                5 * std::hash<int>()(std::get<1>(con)) +
						                7 * std::hash<int>()(std::get<2>(con)) +
                                        11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons1_hash ^ (cons2_hash << 1);
		}
	};
};




typedef unordered_map<HTableEntry, int, HTableEntry::Hasher, HTableEntry::EqNode> HTable;