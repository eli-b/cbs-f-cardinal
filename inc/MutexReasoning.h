#pragma once
#include "IncrementalPairwiseMutexPropagation.hpp"
#include "ConstraintPropagation.h"
#include "ConstraintPropagationGeneralized.h"
#include "MDD.h"

enum mutex_strategy{N_MUTEX, MUTEX_C, MUTEX_NC_FIRST_K, MUTEX_NC_GREEDY };

class MutexReasoning{
public:
	double accumulated_runtime = 0;
	double max_bc_runtime = 0;
	double max_bc_flow_runtime = 0;

	MutexReasoning(const Instance& instance, const vector<ConstraintTable>& initial_constraints):
		instance(instance), initial_constraints(initial_constraints) {}
	shared_ptr<Conflict> run(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2);

	vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' paths and mdd

  mutex_strategy strategy;

private:

  

  shared_ptr<Conflict> iter_path_first_k(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2, ConstraintPropagationGeneralized* cp);
  shared_ptr<Conflict> iter_path_greedy(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2, ConstraintPropagationGeneralized* cp);

  void cache_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2, shared_ptr<Conflict> constraint);
  shared_ptr<Conflict> find_applicable_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2, const vector<Path*> & paths);
  bool has_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2);


  bool constraint_applicable(const vector<Path*> & paths, shared_ptr<Conflict> conflict);
  bool constraint_applicable(const vector<Path*> & paths, list<Constraint>& constraint);
  const Instance& instance;
  const vector<ConstraintTable>& initial_constraints;
  // TODO using MDDs from cache
  // A problem can be whether the modified MDD still being safe for other modules..

  // (cons_hasher_0, cons_hasher_1) -> Constraint
  // Invariant: cons_hasher_0.a < cons_hasher_1.a
  // unordered_map<ConstraintsHasher,
  //               unordered_map<ConstraintsHasher, shared_ptr<Conflict>, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>,
  //               ConstraintsHasher::Hasher, ConstraintsHasher::EqNode
  //               > lookupTable;
  unordered_map<ConstraintsHasher,
                unordered_map<ConstraintsHasher, std::list<shared_ptr<Conflict>>, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>,
                ConstraintsHasher::Hasher, ConstraintsHasher::EqNode
                > lookupTable;


  shared_ptr<Conflict> findMutexConflict(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2);
};

// other TODOs
// TODO duplicated cardinal test in classify conflicts
