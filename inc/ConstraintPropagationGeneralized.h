#include "ConstraintPropagation.h"

class ConstraintPropagationGeneralized: public ConstraintPropagation
{
protected:
  boost::unordered_set<node_pair> general_mutexes;

  bool has_generalized_node_mutex(MDDNode* a, MDDNode* b);
  void add_generalized_node_mutex(MDDNode* node_a, MDDNode* node_b);

public:
  void fwd_mutex_prop();

  ConstraintPropagationGeneralized(MDD* mdd0, MDD* mdd1):
    ConstraintPropagation(mdd0, mdd1)
  {};
};
