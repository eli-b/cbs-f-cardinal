#include <stack>
#include <iostream>

#include "MaxBC.h"
#include "boost/graph/edmonds_karp_max_flow.hpp"
#include "boost/graph/boykov_kolmogorov_max_flow.hpp"
#include "boost/unordered_map.hpp"

bool traversable_edge(vertex_t from, vertex_t to, FlowGraph_t& g, boost::property_map<FlowGraph_t,
                      boost::edge_residual_capacity_t>::type& residualweightmap,
                      boost::property_map<FlowGraph_t, boost::edge_capacity_t>::type& weightmap){


  if (to == 1){
    // not going to the sink
    return false;
  }
  if (to > from){
    auto e = boost::edge(from, to, g).first;
    assert(boost::edge(from, to, g).second);
    return weightmap[e] - residualweightmap[e] == 0;
  }else{
    auto e = boost::edge(to, from, g).first;
    assert(boost::edge(to, from, g).second);
    return weightmap[e] - residualweightmap[e] != 0;
  }
}


std::vector<std::pair<int, int>> residue_edges(int l_num, int r_num, std::vector<std::pair<int, int>> edges){
  std::vector<std::pair<int, int>> res;
  res.reserve(l_num * r_num - edges.size());

  boost::unordered_set<std::pair<int, int>> edges_set(edges.begin(), edges.end());
  for (int i = 0; i < l_num; i++){
    for(int j = 0; j < r_num; j++){
      if (edges_set.find({i,j}) == edges_set.end()){
        res.push_back({i, j});
      }
    }
  }
  return res;
}
std::pair<std::vector<int>, std::vector<int>> MinVC::solve(int smaller_than){
  // Build maxflow graph and solve.
	clock_t start_t = clock();

  long flow =  boost::edmonds_karp_max_flow(flow_g, s, t);
  // long flow =  boost::boykov_kolmogorov_max_flow(flow_g, s, t);

  if (smaller_than >= 0 && flow > smaller_than){
    early_stop = true;
    return { {},{} };
  }

  maxflow_runtime = clock() - start_t;

  // std::cout << "flow:" <<flow << std::endl;

  // check which node could be reachable
  // TODO change to boost implementation of graph algorithm

  boost::property_map<FlowGraph_t, boost::edge_residual_capacity_t>::type residualweightmap = boost::get(boost::edge_residual_capacity, flow_g);
  boost::property_map<FlowGraph_t, boost::edge_capacity_t>::type weightmap = boost::get(boost::edge_capacity, flow_g);


  std::stack<vertex_t> open({s});
  boost::unordered_set<vertex_t> closed({s});

  // std::cout << "Flow:" << std::endl; 
  // for (int i = 0; i < l_side.size(); i++){

  //   auto ei = boost::edge(s, l_side[i], flow_g).first;
  //   if (residualweightmap[ei] == 0){
  //     for (int j = 0; j < r_side.size(); j++){
  //       edge_t e;
  //       bool edge_exists;
  //       boost::tie(e, edge_exists) = boost::edge(l_side[i], r_side[j], flow_g);
  //       if (edge_exists && residualweightmap[e] == 0)
  //         {
  //           // std::cout << i << " -> " << j << std::endl;
  //         }
  //     }
  //   }
  // }
  // // std::cout << "l:" << std::endl;
  // for (int i = 0; i < l_side.size(); i++){
  //   std::cout << i << ", " << l_side[i] << std::endl;
  // }
  // std::cout << "r:" << std::endl;
  // for (int i = 0; i < r_side.size(); i++){
  //   std::cout << i << ", " << r_side[i] << std::endl;
  // }


  while(!open.empty()){
    auto node = open.top();
    // std::cout << node << std::endl;
    open.pop();
    // check outgoing edge
    typename boost::graph_traits < FlowGraph_t >::out_edge_iterator ei, ei_end;

    for (boost::tie(ei, ei_end) = out_edges(node, flow_g); ei != ei_end; ++ei){
      auto source_node = boost::source( *ei, flow_g );
      auto ch_node = boost::target ( *ei, flow_g );

      if (closed.find(ch_node) != closed.end() ||
          !traversable_edge(source_node, ch_node, flow_g, residualweightmap, weightmap )){
        continue;
      }
      closed.insert(ch_node);
      open.push(ch_node);
    }
  }

  std::vector<int> l_vc;
  std::vector<int> r_vc;

  // std::cout << "left side" << std::endl;
  for (int i = 0; i < l_side.size(); i ++){
    auto e = boost::edge(s, l_side[i], flow_g).first;
    if (closed.find(l_side[i]) == closed.end() && residualweightmap[e] == 0){
      l_vc.push_back(i);
    }
  }

  // std::cout << "right side" << std::endl;
  for (int i = 0; i < r_side.size(); i ++){
    auto e = boost::edge(r_side[i], t, flow_g).first;
    if (closed.find(r_side[i]) != closed.end() && residualweightmap[e] == 0){
      r_vc.push_back(i);
    }
  }
  accumulated_runtime = clock() - start_t;
  return {l_vc, r_vc};
}

// TODO improve the efficieny in passing parameter
MinVC::MinVC(int l_num, int r_num, std::vector<std::pair<int, int>> edges)
{
  s = boost::add_vertex(flow_g);
  t = boost::add_vertex(flow_g);
  boost::property_map<FlowGraph_t, boost::edge_capacity_t>::type weightmap = boost::get(boost::edge_capacity, flow_g);
  boost::property_map<FlowGraph_t, boost::edge_reverse_t>::type reverse_edge = boost::get(boost::edge_reverse, flow_g);
  l_side.reserve(l_num);
  r_side.reserve(r_num);
  for (int i = 0; i < l_num; i ++){
    l_side.push_back(boost::add_vertex(flow_g));
    auto e = boost::add_edge(s, l_side[i], flow_g);
    auto e_r = boost::add_edge(l_side[i], s, flow_g);
    weightmap[e.first] = 1;
    weightmap[e_r.first] = 0;
    reverse_edge[e.first] = e_r.first;
    reverse_edge[e_r.first] = e.first;

  }
  for (int i = 0; i < r_num; i ++){
    r_side.push_back(boost::add_vertex(flow_g));
    auto e = boost::add_edge(r_side[i], t, flow_g);
    auto e_r = boost::add_edge(t, r_side[i], flow_g);
    weightmap[e.first] = 1;
    weightmap[e_r.first] = 0;
    reverse_edge[e.first] = e_r.first;
    reverse_edge[e_r.first] = e.first;
  }

  for (const auto edge:edges){
    int l, r;
    boost::tie(l, r) = edge;
    auto e = boost::add_edge(l_side[l], r_side[r], flow_g);
    auto e_r = boost::add_edge(r_side[r], l_side[l], flow_g);

    reverse_edge[e.first] = e_r.first;
    reverse_edge[e_r.first] = e.first;

    weightmap[e.first] = 1;
    weightmap[e_r.first] = 0;
  }
}

MaxBC::MaxBC(int l_num, int r_num, std::vector<std::pair<int, int>> edges):l_num(l_num), r_num(r_num), max_vc_solver(l_num, r_num, residue_edges(l_num, r_num, edges)){}

std::pair<std::vector<int>, std::vector<int>> MaxBC::solve(int larger_than){
  std::vector<int> vec_l;
  std::vector<int> vec_r;
  boost::tie(vec_l, vec_r) = max_vc_solver.solve(l_num + r_num - larger_than);

  if (max_vc_solver.early_stop){
    return {{}, {}};
  }

  std::vector<int> l_bc;
  std::vector<int> r_bc;

  // std:: cout<< "l:" << std::endl;
  for (int i=0, j = 0; i < l_num; i ++){
    if (vec_l.size() != 0 && vec_l[j] == i){
      if (j + 1 < vec_l.size()){
        j++;
      }
    }else{
      // std:: cout<< i << std::endl;
      l_bc.push_back(i);
    }
  }

  // std:: cout<< "r:" << std::endl;
  for (int i=0, j=0; i < r_num; i ++){
    if (vec_r.size() != 0 && vec_r[j] == i){
      if (j + 1 < vec_r.size()){
        j++;
      }
    }else{
      // std:: cout<< i << std::endl;
      r_bc.push_back(i);
    }
  }
  return {l_bc, r_bc};

}

