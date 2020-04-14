#include "ConstraintPropagation.h"

class ConstraintPropagationGeneralized: public ConstraintPropagation
{
protected:
  boost::unordered_set<edge_pair> general_mutexes;

  bool has_generalized_mutex(node_pair a, node_pair b);
  void add_generalized_mutex(node_pair node_a, node_pair node_b);

public:
  void fwd_mutex_prop();

  ConstraintPropagationGeneralized(MDD* mdd0, MDD* mdd1):
    ConstraintPropagation(mdd0, mdd1)
  {};
};
