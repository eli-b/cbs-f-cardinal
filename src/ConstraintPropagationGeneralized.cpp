#include "ConstraintPropagationGeneralized.h"

void ConstraintPropagationGeneralized::fwd_mutex_prop(){
  /*
    Generalize mutex to MDD node of different level.

    MDD nodes a and b (a.t > b.t) is is mutex if all of a's parent is mutex with b.
  */
  std::vector<boost::unordered_set<edge_pair> > to_check(max(mdd0->levels.size(), mdd1->levels.size()));

  unordered_map<MDDNode*, list<MDDNode*>> generalMutexes;

  for (const auto & mutex: fwd_mutexes){
    int l = mutex.first.first->level;
    to_check[l].insert(mutex);
  }

  for (int i = 0; i < to_check.size(); i ++){
    for (auto & mutex: to_check[i]){
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
        generalMutexes[node_to_1].push_back(node_to_2);
        generalMutexes[node_to_2].push_back(node_to_1);
        assert(i + 1 == node_to_1->level);
        to_check[i + 1].insert(new_mutex);

      }else{
        // Node mutex
        auto node_a = mutex.first.first;
        auto node_b = mutex.second.first;

        generalMutexes[node_a].push_back(node_b);
        generalMutexes[node_b].push_back(node_a);
        // Check their child

        for (auto node_a_ch: node_a->children){
          for (auto node_b_ch: node_b->children){
            add_fwd_edge_mutex(node_a, node_a_ch, node_b, node_b_ch);
            if (has_fwd_mutex(node_a_ch, node_b_ch)){
              continue;
            }
            if (!should_be_fwd_mutexed(node_a_ch, node_b_ch)){
              continue;
            }

            auto new_mutex = std::make_pair(std::make_pair(node_a_ch, nullptr),
                                            std::make_pair(node_b_ch, nullptr));

            fwd_mutexes.insert(new_mutex);
            generalMutexes[node_a_ch].push_back(node_b_ch);
            generalMutexes[node_b_ch].push_back(node_a_ch);
            assert(i + 1 == node_a_ch->level);
            to_check[i + 1].insert(new_mutex);
          }
        }
      }
    }
  }

  for (int l = 1; l < mdd0->levels.size(); l++){
    for (MDDNode* node: mdd0->levels[l]){
      int i = 0;
      unordered_set<MDDNode*> mutexes;
      for (auto parent: node->parents){
        if (i == 0){
          mutexes = unordered_set<MDDNode*>(generalMutexes[parent].begin(), generalMutexes[parent].end());
          i ++;
        }else{
          if ( mutexes.size() == 0 ){continue;}

          unordered_set<MDDNode*> tmp_mutexes;
          for (MDDNode* mutex_node: generalMutexes[parent]){
            if (mutexes.find(mutex_node) != mutexes.end()){
              tmp_mutexes.insert(mutex_node);
            }
          }
          mutexes = tmp_mutexes;
        }
      }
      for (MDDNode* mutex_node: mutexes){
        generalMutexes[node].push_back(mutex_node);
      }
    }
  }
  for (int l = 1; l < mdd1->levels.size(); l++){
    for (MDDNode* node: mdd1->levels[l]){
      int i = 0;
      unordered_set<MDDNode*> mutexes;
      for (auto parent: node->parents){
        if (i == 0){
          mutexes = unordered_set<MDDNode*>(generalMutexes[parent].begin(), generalMutexes[parent].end());
          i ++;
        }else{
          if ( mutexes.size() == 0 ){continue;}

          unordered_set<MDDNode*> tmp_mutexes;
          for (MDDNode* mutex_node: generalMutexes[parent]){
            if (mutexes.find(mutex_node) != mutexes.end()){
              tmp_mutexes.insert(mutex_node);
            }
          }
          mutexes = tmp_mutexes;
        }
      }
      for (MDDNode* mutex_node: mutexes){
        generalMutexes[mutex_node].push_back(node);
      }
    }
  }
  for (const auto & map_entry: generalMutexes){
    MDDNode* node = map_entry.first;
    for (MDDNode* mutex : map_entry.second){
      if (node->level > mutex->level){
        add_generalized_node_mutex(node, mutex);
      }
    }
  }

  return;
}

bool ConstraintPropagationGeneralized::has_generalized_node_mutex(MDDNode* a, MDDNode* b){
  return general_mutexes.find({a, b}) != general_mutexes.end() || general_mutexes.find({b, a}) != general_mutexes.end();
}

void ConstraintPropagationGeneralized::add_generalized_node_mutex(MDDNode* node_a, MDDNode* node_b){
  if (has_generalized_node_mutex(node_a, node_b)){
    return;
  }
  general_mutexes.insert({node_a, node_b});
}
