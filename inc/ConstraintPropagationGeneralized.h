#pragma once
#include "ConstraintPropagation.h"

class ConstraintPropagationGeneralized: public ConstraintPropagation
{
protected:
  void add_generalized_mutex(node_pair node_a, node_pair node_b);

  boost::unordered_set<node_pair> counted_edges;


public:
  boost::unordered_set<edge_pair> general_mutexes;

	double max_bc_runtime = 0;
	double max_bc_flow_runtime = 0;

  void fwd_mutex_prop();
  void compute_generalized_mutex();

  bool has_generalized_mutex(node_pair a, node_pair b);

  ConstraintPropagationGeneralized(MDD* mdd0, MDD* mdd1):
    ConstraintPropagation(mdd0, mdd1)
  {};

  ~ConstraintPropagationGeneralized(){};

  // std::pair<std::vector<Constraint>, std::vector<Constraint>> generate_constraints(int level_0, int loc_0, int level_1, int loc_1);

  int bipartite_size(node_pair np0, node_pair np1);


  std::pair<std::vector<Constraint>, std::vector<Constraint>> generate_constraints_greedy(const vector<Path*>& paths, int a1, int a2);


  std::pair<std::vector<Constraint>, std::vector<Constraint>> generate_constraints(node_pair np0, node_pair np1, int con_lt=0);

};
