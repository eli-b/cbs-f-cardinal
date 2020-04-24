#include "MutexReasoning.h"
#include "ConstraintPropagation.h"
#include "ConstraintPropagationGeneralized.h"


shared_ptr<Conflict> MutexReasoning::run(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2)
{
  if (a1 > a2){
    std::swap(a1, a2);
    std::swap(mdd_1, mdd_2);
  }

  if (strategy == mutex_strategy::N_MUTEX){
    return nullptr;
  }

  clock_t t = clock();

  // Seems that there is no need to do the swapped...

	auto conflict = findMutexConflict(paths, a1, a2, node, mdd_1, mdd_2);

	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;

	return conflict;
}

shared_ptr<Conflict> MutexReasoning::findMutexConflict(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2){
  assert(a1 < a2);
	ConstraintsHasher c_1(a1, &node);
	ConstraintsHasher c_2(a2, &node);
  bool use_general_mutex = strategy == mutex_strategy::MUTEX_NC;

  if (has_constraint(c_1, c_2)){
    if (!use_general_mutex){
      return find_applicable_constraint(c_1, c_2, paths);
    }
    else{
      shared_ptr<Conflict> mutex_conflict = find_applicable_constraint(c_1, c_2, paths);
      if (mutex_conflict != nullptr){
        // found a usable mutex;
        return mutex_conflict;
      }
    }
  }

  ConstraintPropagation* cp;
shared_ptr<Conflict> mutex_conflict = nullptr;

  if (use_general_mutex){
    cp = new ConstraintPropagationGeneralized(mdd_1, mdd_2);

  }else{
    cp = new ConstraintPropagation(mdd_1, mdd_2);
  }
  cp->init_mutex();
  cp->fwd_mutex_prop();

  if (cp->_feasible(mdd_1->levels.size() - 1, mdd_2->levels.size() - 1) >= 0){
    if (! use_general_mutex){
      delete cp;
      cache_constraint(c_1, c_2, nullptr);
      return nullptr;
    }
    ConstraintPropagationGeneralized* cp_gen = (ConstraintPropagationGeneralized*) cp;
    cp_gen->compute_generalized_mutex();

    mutex_conflict = iter_path(paths, a1, a2, node, mdd_1, mdd_2, cp_gen);

    delete cp;

    if (mutex_conflict != nullptr){
      cache_constraint(c_1, c_2, mutex_conflict);
    }
    return mutex_conflict;
    // use generalized mutex to resolve non-cardinal conflict;
  }

  delete cp;

  // generate constraint;
  mutex_conflict = make_shared<Conflict>();
 mutex_conflict->mutexConflict(a1, a2);

  MDD mdd_1_cpy(*mdd_1);
  MDD mdd_2_cpy(*mdd_2);

  ConstraintTable ct1(initial_constraints[a1]);
  ConstraintTable ct2(initial_constraints[a2]);

  ct1.build(node, a1);
  ct2.build(node, a2);
  auto ip = IPMutexPropagation(&mdd_1_cpy, &mdd_2_cpy, search_engines[a1], search_engines[a2],
                               ct1, ct2);
  con_vec a;
  con_vec b;
  std::tie(a, b) = ip.gen_constraints();

  for (auto con:a){
    get<0>(con) = a1;
    mutex_conflict->constraint1.push_back(con);
  }

  for (auto con:b){
    get<0>(con) = a2;
    mutex_conflict->constraint2.push_back(con);
  }

  mutex_conflict->final_len_1 = ip.final_len_0;
  mutex_conflict->final_len_2 = ip.final_len_1;

  // mutex_conflict->constraint1 = list<Constraint>(a.begin(), a.end());
  // mutex_conflict->constraint2 = list<Constraint>(b.begin(), b.end());

  cache_constraint(c_1, c_2, mutex_conflict);

  // prepare for return
  return mutex_conflict;
}

bool MutexReasoning::constraint_applicable(const vector<Path*> & paths, shared_ptr<Conflict> conflict){
  if (conflict->priority== conflict_priority::CARDINAL){
    return true;
  }else{
    return constraint_applicable(paths, conflict->constraint1) && constraint_applicable(paths, conflict->constraint2);
  }
}


bool MutexReasoning::constraint_applicable(const vector<Path*> & paths, list<Constraint>& constraint){
  for (Constraint& c: constraint){
    if (get<4>(c) == constraint_type::VERTEX){
      int ag = get<0>(c);
      int loc = get<1>(c);
      int t = get<3>(c);
      if ((*paths[ag])[t].location == loc) {
        return true;
      }
    }else if (get<4>(c) == constraint_type::EDGE){
      int ag = get<0>(c);
      int loc = get<1>(c);
      int loc_to = get<2>(c);
      int t = get<3>(c);
      if ((*paths[ag])[t - 1].location == loc &&
          (*paths[ag])[t].location == loc_to ) {
        return true;
      }
    }
  }
  return false;
}




shared_ptr<Conflict> MutexReasoning::iter_path(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2, ConstraintPropagationGeneralized* cp_gen){
  shared_ptr<Conflict> mutex_conflict;
  Path* p1 = paths[a1];
    Path* p2 = paths[a2];
    std::pair<std::vector<Constraint>, std::vector<Constraint>> constraint;
    edge_pair center;

    int cnt = 5;

    // for (int i = max(p1->size(), p2->size()) - 1 ; i >= 0 ; i--){
    for (int i = 0; i < max(p1->size(), p2->size()); i++){
      int i1 = i <= p1->size()? i: p1->size() - 1;
      int i2 = i <= p2->size()? i: p2->size() - 1;
      auto pe1 = (*p1)[i1];
      auto pe2 = (*p2)[i2];
      MDDNode* n1 = mdd_1->find(pe1.location, i1);
      MDDNode* n2 = mdd_2->find(pe2.location, i2);

      cout << "i" << endl;

      if (cnt > 0 && cp_gen->has_generalized_mutex({n1, nullptr}, {n2, nullptr})){
        cnt --;
        cout << "Agents: "  << a1 << ", " << a2 << ", vertexes: " << n1->location << ", " << n2->location << endl;
        clock_t start = clock();
        auto temp_con = cp_gen->generate_constraints({n1, nullptr}, {n2, nullptr}, constraint.first.size() + constraint.second.size());
        cout << "size: " << temp_con.first.size() + temp_con.second.size() << ", time: " << clock() - start << endl;
        max_bc_runtime += cp_gen->max_bc_runtime;
        max_bc_flow_runtime += cp_gen->max_bc_flow_runtime;
        if (temp_con.first.size() + temp_con.second.size() > constraint.first.size() + constraint.second.size()){
          constraint = temp_con;
          center = { {n1, nullptr}, {n2, nullptr} };
        }
      }
      if (i + 1 < min(p1->size(), p2->size())){
        auto pe1_to = (*p1)[i + 1];
        auto pe2_to = (*p2)[i + 1];
        MDDNode* n1_to = mdd_1->find(pe1_to.location, i + 1);
        MDDNode* n2_to = mdd_2->find(pe2_to.location, i + 1);
        if (cnt > 0 && cp_gen->has_generalized_mutex({n1, n1_to}, {n2, n2_to})){
          cnt -- ;
          cout << "Agents: "  << a1 << ", " << a2 << ", edges: (" << n1->location << ", " << n1_to->location << "), ("<< n2_to->location << ", " << n2->location << ")" << endl;
          clock_t start = clock();
          auto temp_con = cp_gen->generate_constraints({n1, n1_to}, {n2, n2_to}, constraint.first.size() + constraint.second.size());
          cout << "size: " << temp_con.first.size() + temp_con.second.size() << ", time: " << clock() - start << endl;
          if (temp_con.first.size() + temp_con.second.size() > constraint.first.size() + constraint.second.size()){
            constraint = temp_con;
            center = { {n1, n1_to}, {n2, n2_to} };
          }
        }
      }
    }

    if (constraint.first.size() == 0 ||constraint.second.size() == 0){
      // TODO fix old mutex implementation (conflict after agent stop...)
      // lookupTable[c_1][c_2] = nullptr;
      return nullptr;
    }

    mutex_conflict = make_shared<Conflict>();
    mutex_conflict->mutexConflict(a1, a2);
    mutex_conflict->priority = conflict_priority::MUTEX_NON;
    if (center.first.second == nullptr){
      mutex_conflict->t1 = center.first.first->level;
      mutex_conflict->t2 = center.second.first->level;
      mutex_conflict->loc1 = center.first.first->location;
      mutex_conflict->loc2 = center.second.first->location;
        }else{
      mutex_conflict->t1 = center.first.first->level;
      mutex_conflict->t2 = center.second.first->level;
      mutex_conflict->loc1 = center.first.first->location;
      mutex_conflict->loc2 = center.second.first->location;
      mutex_conflict->loc1_to = center.first.second->location;
      mutex_conflict->loc2_to = center.second.second->location;
    }
    for (auto con: constraint.first){
      get<0>(con) = a1;
      mutex_conflict->constraint1.push_back(con);
    }

    for (auto con: constraint.second){
      get<0>(con) = a2;
      mutex_conflict->constraint2.push_back(con);
    }
    return mutex_conflict;
}

void MutexReasoning::cache_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2, shared_ptr<Conflict> constraint){
  lookupTable[c1][c2].push_back(constraint);
}

shared_ptr<Conflict> MutexReasoning::find_applicable_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2, const vector<Path*> & paths){
  if (lookupTable.find(c1) != lookupTable.end()){
    if (lookupTable[c1].find(c2) != lookupTable[c1].end()){
      for (auto& constraint: lookupTable[c1][c2]){
        if (constraint_applicable(paths, constraint)){
          return make_shared<Conflict>(*constraint);
        }
      }
    }
  }
  return nullptr;
}

bool MutexReasoning::has_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2){
  return lookupTable.find(c1) != lookupTable.end() && lookupTable[c1].find(c2) != lookupTable[c1].end();
}
