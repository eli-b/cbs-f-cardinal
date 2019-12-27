#pragma once

#include "SingleAgentICBS.h"


class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	int location;
	int level;

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}


	std::list<MDDNode*> children;
	std::list<MDDNode*> parents;
	//MDDNode* parent;
};



class MDD
{
public:
	std::vector<std::list<MDDNode*>> levels;

	bool buildMDD(const ConstraintTable& ct, int map_size,
		int numOfLevels, const SingleAgentICBS & solver);
	// bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
	// 	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col);

	MDDNode* find(int location, int level) const;
	void deleteNode(MDDNode* node);
	void clear();
	// bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

	MDD()= default;;
	MDD(const MDD & cpy);
	~MDD();
};


class SyncMDDNode
{
public:
	SyncMDDNode(int currloc, SyncMDDNode* parent)
	{
		location = currloc;
		if (parent != nullptr)
		{
			//level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	int location;
	//int level;

	bool operator == (const SyncMDDNode & node) const
	{
		return (this->location == node.location);
	}


	std::list<SyncMDDNode*> children;
	std::list<SyncMDDNode*> parents;
	std::list<const MDDNode*> coexistingNodesFromOtherMdds;

};


class SyncMDD
{
public:
	std::vector<std::list<SyncMDDNode*>> levels;

	SyncMDDNode* find(int location, int level) const;
	void deleteNode(SyncMDDNode* node, int level);
	void clear();

	explicit SyncMDD(const MDD & cpy);
	~SyncMDD();
};


// Match and prune MDD according to another MDD.
bool SyncMDDs(const MDD &mdd1, const MDD& mdd2);