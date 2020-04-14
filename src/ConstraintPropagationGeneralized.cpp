#include "ConstraintPropagationGeneralized.h"

void ConstraintPropagationGeneralized::fwd_mutex_prop(){
  /*
    Generalize mutex to MDD node of different level.

    MDD nodes a and b (a.t > b.t) is is mutex if all of a's parent is mutex with b.
  */
  std::vector<boost::unordered_set<edge_pair> > to_check(2 * max(mdd0->levels.size(), mdd1->levels.size()));

  unordered_map<node_pair, list<node_pair>> generalMutexes;

  for (const auto & mutex: fwd_mutexes){
    int l = mutex.first.first->level;
    if ( is_edge_mutex(mutex) ){
      to_check[2 * l + 1].insert(mutex);
    }
    else{
      to_check[2 * l].insert(mutex);
    }
  }

  for (int i = 0; i < to_check.size(); i ++){
    for (auto & mutex: to_check[i]){
      generalMutexes[mutex.first].push_back(mutex.second);
      generalMutexes[mutex.second].push_back(mutex.first);
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

bool ConstraintPropagationGeneralized::has_generalized_mutex(node_pair a, node_pair b){
  return general_mutexes.find({a, b}) != general_mutexes.end() || general_mutexes.find({b, a}) != general_mutexes.end();
}

void ConstraintPropagationGeneralized::add_generalized_mutex(node_pair node_a, node_pair node_b){
  if (has_generalized_mutex(node_a, node_b)){
    return;
  }
  general_mutexes.insert({node_a, node_b});
}
