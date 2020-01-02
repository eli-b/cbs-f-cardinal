#pragma once
#include "common.h"

int minimumVertexCover(const vector<int>& CG, int old_mvc, int cols, int num_of_edges);

bool KVertexCover(const vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols);

int greedyMatching(const vector<int>& CG, int cols);

int weightedVertexCover(const vector<int>& CG, int N);
int weightedVertexCover(vector<int>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far);


