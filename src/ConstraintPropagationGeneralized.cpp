#include "ConstraintPropagationGeneralized.h"
#include "MaxBC.h"

void ConstraintPropagationGeneralized::fwd_mutex_prop(){
  /*
    Generalize mutex to MDD node of different level.

    MDD nodes a and b (a.t > b.t) is is mutex if all of a's parent is mutex with b.
  */
  std::vector<boost::unordered_set<edge_pair> > to_check(2 * max(mdd0->levels.size(), mdd1->levels.size()));


  // add init cons to queue
  for (const auto & mutex: fwd_mutexes){
    int l = mutex.first.first->level;
    if ( is_edge_mutex(mutex) ){
      to_check[2 * l + 1].insert(mutex);
    }
    else{
      to_check[2 * l].insert(mutex);
    }
  }

  // propagation
  for (int i = 0; i < to_check.size(); i ++){
    for (auto & mutex: to_check[i]){
      // generalMutexes[mutex.first].push_back(mutex.second);
      // generalMutexes[mutex.second].push_back(mutex.first);
      if (is_edge_mutex(mutex)){

        auto node_to_1 = mutex.first.second;
        auto node_to_2 = mutex.second.second;

        if (has_fwd_mutex(node_to_1, node_to_2)){
          continue;
        }
        if (!should_be_fwd_mutexed(node_to_1, node_to_2)){
          continue;
        }

        auto new_mutex = std::make_pair(std::make_pair(node_to_1, nullptr),
                                        std::make_pair(node_to_2, nullptr));

        fwd_mutexes.insert(new_mutex);
        assert(i + 1 == 2 * node_to_1->level);
        to_check[i + 1].insert(new_mutex);

      }else{
        // Node mutex
        auto node_a = mutex.first.first;
        auto node_b = mutex.second.first;

        // Check their child

        for (auto node_a_ch: node_a->children){
          for (auto node_b_ch: node_b->children){
            add_fwd_edge_mutex(node_a, node_a_ch, node_b, node_b_ch);
            to_check[i + 1].insert({{node_a, node_a_ch}, { node_b, node_b_ch}});
          }
        }
      }
    }
  }

  // for (const auto & map_entry: generalMutexes){
  //   node_pair node = map_entry.first;
  //   for (node_pair mutex : map_entry.second){
  //     if (2 * node.first->level + (node.second == nullptr? 1:0) > 2 * mutex.first->level + (mutex.second == nullptr? 1:0)){
  //       add_generalized_mutex(node, mutex);
  //     }
  //   }
  // }

  cout  << "a";
  return;
}

void ConstraintPropagationGeneralized::compute_generalized_mutex(){
  unordered_map<node_pair, list<node_pair>> generalMutexes;

  for (const auto & mutex: fwd_mutexes){
    add_generalized_mutex(mutex.first, mutex.second);
    if (mutex.first.second != nullptr){
      counted_edges.insert(mutex.first);
      counted_edges.insert(mutex.second);
    }
    generalMutexes[mutex.first].push_back(mutex.second);
    generalMutexes[mutex.second].push_back(mutex.first);
  }

  // generalized mutex
  for (int l = 0; l < mdd0->levels.size(); l++){
    for (MDDNode* node: mdd0->levels[l]){
      int i = 0;
      unordered_set< node_pair> mutexes;
      for (auto parent: node->parents){
        node_pair e = {parent, node};
        if (i == 0){
          mutexes = unordered_set<node_pair>(generalMutexes[e].begin(), generalMutexes[e].end());
          i ++;
        }else{
          if ( mutexes.size() == 0 ){continue;}

          unordered_set<node_pair> tmp_mutexes;
          for (node_pair mutex: generalMutexes[e]){
            if (mutexes.find(mutex) != mutexes.end()){
              tmp_mutexes.insert(mutex);
            }
          }
          mutexes = tmp_mutexes;
        }
      }
      for (node_pair mutex_node: mutexes){
        generalMutexes[{node, nullptr}].push_back(mutex_node);
        add_generalized_mutex({node, nullptr}, mutex_node);
      }
      for (MDDNode* ch_node: node->children){
        node_pair e = {node, ch_node};
        for (node_pair m : generalMutexes[{node, nullptr}])
          {
            generalMutexes[e].push_back(m);
            add_generalized_mutex(e, m);
          }
      }
    }
  }
  
  for (int l = 0; l < mdd1->levels.size(); l++){
    for (MDDNode* node: mdd1->levels[l]){
      int i = 0;
      unordered_set< node_pair> mutexes;
      for (auto parent: node->parents){
        node_pair e = {parent, node};
        if (i == 0){
          mutexes = unordered_set<node_pair>(generalMutexes[e].begin(), generalMutexes[e].end());
          i ++;
        }else{
          if ( mutexes.size() == 0 ){continue;}

          unordered_set<node_pair> tmp_mutexes;
          for (node_pair mutex: generalMutexes[e]){
            if (mutexes.find(mutex) != mutexes.end()){
              tmp_mutexes.insert(mutex);
            }
          }
          mutexes = tmp_mutexes;
        }
      }
      for (node_pair mutex_node: mutexes){
        generalMutexes[{node, nullptr}].push_back(mutex_node);
        add_generalized_mutex(mutex_node, {node, nullptr});
      }
      for (MDDNode* ch_node: node->children){
        node_pair e = {node, ch_node};
        for (node_pair m : generalMutexes[{node, nullptr}])
          {
          generalMutexes[e].push_back(m);
          add_generalized_mutex(m, e);
          }
      }
    }
  }
}

bool ConstraintPropagationGeneralized::has_generalized_mutex(node_pair a, node_pair b){
  return general_mutexes.find({a, b}) != general_mutexes.end() || general_mutexes.find({b, a}) != general_mutexes.end();
}

void ConstraintPropagationGeneralized::add_generalized_mutex(node_pair node_a, node_pair node_b){
  if (has_generalized_mutex(node_a, node_b)){
    return;
  }
  general_mutexes.insert({node_a, node_b});
}


bool is_mutex_both_vertex(edge_pair e){
  return e.first.second == nullptr && e.second.second == nullptr;
}

// std::pair<std::vector<Constraint>, std::vector<Constraint>> ConstraintPropagationGeneralized::generate_constraints(int level_0, int loc_0, int level_1, int loc_1){
//   MDDNode * mdd_node_0 = mdd0->find(loc_0, level_0);
//   MDDNode * mdd_node_1 = mdd1->find(loc_1, level_1);

//   boost::unordered_set<MDDNode*> nodes_from_mdd_0;
//   boost::unordered_set<MDDNode*> nodes_from_mdd_1;

//   for (edge_pair ep: general_mutexes){
//     if (!(is_mutex_both_vertex(ep))){
//       continue;
//     }
//     if (ep.first.first == mdd_node_0){
//       nodes_from_mdd_1.insert(ep.second.first);
//     }
//     if (ep.second.first == mdd_node_1){
//       nodes_from_mdd_0.insert(ep.first.first);
//     }
//   }

//   boost::unordered_set<edge_pair> edges;
//   for (edge_pair ep: general_mutexes){
//     if (!(is_mutex_both_vertex(ep))){
//       continue;
//     }

//     if (nodes_from_mdd_0.find(ep.first.first) != nodes_from_mdd_0.end() &&
//         nodes_from_mdd_1.find(ep.second.first) != nodes_from_mdd_1.end() ){
//       edges.insert(ep);
//     }
//   }

//   vector<MDDNode*> id_to_node_0(nodes_from_mdd_0.begin(), nodes_from_mdd_0.end());
//   vector<MDDNode*> id_to_node_1(nodes_from_mdd_1.begin(), nodes_from_mdd_1.end());

//   boost::unordered_map<MDDNode*, int> node_to_id_0;
//   for (int i = 0; i < id_to_node_0.size(); i++){
//     node_to_id_0[id_to_node_0[i]] = i;
//   }

//   boost::unordered_map<MDDNode*, int> node_to_id_1;
//   for (int i = 0; i < id_to_node_1.size(); i++){
//     node_to_id_1[id_to_node_1[i]] = i;
//   }

//   vector<pair<int,int>> id_edges;
//   id_edges.reserve(edges.size());
//   for (auto e:edges){
//     id_edges.push_back({node_to_id_0[e.first.first], node_to_id_1[e.second.first]});
//   }

//   auto bc = MaxBC(id_to_node_0.size(), id_to_node_1.size(), id_edges);
//   auto sol = bc.solve();

//   vector<Constraint> con_0;
//   vector<Constraint> con_1;

//   for (int id : sol.first){
//     MDDNode* node = id_to_node_0[id];
//     con_0.push_back({0, node->location, -1, node->level, constraint_type::VERTEX});
//   }
//   for (int id : sol.second){
//     MDDNode* node = id_to_node_1[id];
//     con_1.push_back({0, node->location, -1, node->level, constraint_type::VERTEX});
//   }

//   return {con_0, con_1};
// }

std::pair<std::vector<Constraint>, std::vector<Constraint>> ConstraintPropagationGeneralized::generate_constraints(node_pair np1, node_pair np2, int con_size_larger_than){

  boost::unordered_set<node_pair> nodes_from_mdd_0({np1});
  boost::unordered_set<node_pair> nodes_from_mdd_1({np2});

  for (edge_pair ep: general_mutexes){
    if (ep.first == np1 && (ep.second.second == nullptr || counted_edges.find(ep.second) != counted_edges.end())){
      nodes_from_mdd_1.insert(ep.second);
    }
    if (ep.second == np2 &&
        (
         ep.first.second == nullptr || counted_edges.find(ep.first) != counted_edges.end()
         )
        ){
      nodes_from_mdd_0.insert(ep.first);
    }
  }

  boost::unordered_set<edge_pair> edges;
  for (edge_pair ep: general_mutexes){
    if (nodes_from_mdd_0.find(ep.first) != nodes_from_mdd_0.end() &&
        nodes_from_mdd_1.find(ep.second) != nodes_from_mdd_1.end()){
      edges.insert(ep);
    }
  }

  vector<node_pair> id_to_node_0(nodes_from_mdd_0.begin(), nodes_from_mdd_0.end());
  vector<node_pair> id_to_node_1(nodes_from_mdd_1.begin(), nodes_from_mdd_1.end());

  boost::unordered_map<node_pair, int> node_to_id_0;
  for (int i = 0; i < id_to_node_0.size(); i++){
    node_to_id_0[id_to_node_0[i]] = i;
  }

  boost::unordered_map<node_pair, int> node_to_id_1;
  for (int i = 0; i < id_to_node_1.size(); i++){
    node_to_id_1[id_to_node_1[i]] = i;
  }

  vector<pair<int,int>> id_edges;
  id_edges.reserve(edges.size());
  for (auto e:edges){
    id_edges.push_back({node_to_id_0[e.first], node_to_id_1[e.second]});
  }

  cout << "graph size: " <<  id_to_node_0.size() << ", " << id_to_node_1.size() << endl;

  auto bc = MaxBC(id_to_node_0.size(), id_to_node_1.size(), id_edges);
  auto sol = bc.solve(con_size_larger_than);

  max_bc_runtime = bc.max_vc_solver.accumulated_runtime;
  max_bc_flow_runtime = bc.max_vc_solver.maxflow_runtime;

  vector<Constraint> con_0;
  vector<Constraint> con_1;

  for (int id : sol.first){
    node_pair np = id_to_node_0[id];
    if (np.second == nullptr){
      auto node = np.first;
      con_0.push_back({0, node->location, -1, node->level, constraint_type::VERTEX});
    }else{
      con_0.push_back({0, np.first->location, np.second->location, np.second->level, constraint_type::EDGE});
    }

  }
  for (int id : sol.second){
    node_pair np = id_to_node_1[id];
    if (np.second == nullptr){
      auto node = np.first;
      con_1.push_back({1, node->location, -1, node->level, constraint_type::VERTEX});
    }else{
      con_1.push_back({1, np.first->location, np.second->location, np.second->level, constraint_type::EDGE});
    }
  }

  return {con_0, con_1};
}
