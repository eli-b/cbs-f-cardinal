#include "CBSHeuristic.h"
#include "CBS.h"
#include <ilcplex/ilocplex.h>


int WDGHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit)
{
  int h = -1;
	int num_of_CGedges;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
  if (!buildWeightedDependenceGraph(curr, HG))
    return false;
  h = minimumWeightedVertexCover(HG);
  return h;
}

int WDGHeuristic::minimumWeightedVertexCover(const vector<int>& HG)
{
	clock_t t = clock();
	int rst = weightedVertexCover(HG);
	runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

int WDGHeuristic::weightedVertexCover(const std::vector<int>& CG)
{
int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], CG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
				else if (CG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], CG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(CG[indices[0] * num_of_agents + indices[1]], CG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(CG[indices[j] * num_of_agents + indices[k]], CG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		if (num > ILP_node_threshold) // solve by ILP
		{
			rst += ILPForWMVC(G, range);
		}
		else // solve by dynamic programming
		{
			std::vector<int> x(num);
			int best_so_far = MAX_COST;
			rst += DPForWMVC(x, 0, 0, G, range, best_so_far);
		}
		double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}

	//test
	/*std::vector<int> x(N, 0);
	std::vector<int> range(N, 0);
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			range[i] = std::max(range[i], CG[i * N + j]);
			range[j] = std::max(range[j], CG[i * N + j]);
		}
	}
	int best_so_far = INT_MAX;
	int rst2 = DPForWMVC(x, 0, 0, CG, range, best_so_far);
	if ( rst != rst2)
		std::cout << "ERROR" <<std::endl;*/

	return rst;
}

bool WDGHeuristic::buildWeightedDependenceGraph(CBSNode& node, vector<int>& CG)
{
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (node.conflictGraph.find(idx) != node.conflictGraph.end())
			continue;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;
			node.conflictGraph[idx] = got->second;
		}
		else if (rectangle_reasoning || mutex_reasoning)
		{
			node.conflictGraph[idx] = solve2Agents(a1, a2, node, false);
			assert(node.conflictGraph[idx] >= 0);
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.conflictGraph[idx];
		}
		else
		{
			bool cardinal = conflict->priority == conflict_priority::CARDINAL;
			if (!cardinal) // using merging MDD methods before running 2-agent instance
			{
				cardinal = dependent(a1, a2, node);
			}
			if (cardinal) // run 2-agent solver only for dependent agents
			{
				node.conflictGraph[idx] = solve2Agents(a1, a2, node, cardinal);
				assert(node.conflictGraph[idx] >= 1);
			}
			else
			{
				node.conflictGraph[idx] = 0;
			}
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.conflictGraph[idx];
		}

		if (node.conflictGraph[idx] == MAX_COST) // no solution
		{
			return false;
		}

		if (conflict->priority != conflict_priority::CARDINAL && node.conflictGraph[idx] > 0)
		{
			conflict->priority = conflict_priority::PSEUDO_CARDINAL; // the two agents are dependent, although resolving this conflict might not increase the cost
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_dependency_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
	}

	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = node.conflictGraph.find(i * num_of_agents + j);
			if (got != node.conflictGraph.end() && got->second > 0)
			{
				CG[i * num_of_agents + j] = got->second;
				CG[j * num_of_agents + i] = got->second;
			}
		}
	}
	runtime_build_dependency_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


// recursive component of dynamic programming for weighted vertex cover
int
WDGHeuristic::DPForWMVC(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range,
						int& best_so_far)
{
	if (sum >= best_so_far)
		return MAX_COST;
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int) x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}

	int cols = x.size();

	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = DPForWMVC(x, i + 1, sum + x[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost;
	}

	return best_so_far;
}

/*
// integer linear programming for weighted vertex cover
// SCIP
int WDGHeuristic::ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value)
{
	int N = (int) node_max_value.size();

	// initialize SCIP environment 
	SCIP* scip = NULL;
	SCIP_CALL(SCIPcreate(&scip));

	// Version information
	// SCIPprintVersion(scip, NULL);
	// SCIPinfoMessage(scip, NULL, "\n");

	// include default plugins 
	SCIP_CALL(SCIPincludeDefaultPlugins(scip));

	// set verbosity parameter 
	SCIP_CALL(SCIPsetIntParam(scip, "display/verblevel", 5));
	// SCIP_CALL( SCIPsetBoolParam(scip, "display/lpinfo", TRUE) );

	// disable scip output to stdout
	SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(scip), TRUE);

	// set runtime limit
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	SCIP_CALL(SCIPsetRealParam(scip, "limits/time", time_limit - runtime));

	// create empty problem 
	SCIP_CALL(SCIPcreateProb(scip, "WDG", NULL, NULL, NULL, NULL, NULL, NULL, NULL));

	// set the objective senseif necessary, default is minimize
	// SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

	// add variables 
	vector<SCIP_VAR*> vars(N);
	for (int i = 0; i < N; ++i)
	{
		std::string s = std::to_string(i);
		char const* pchar = s.c_str();  //use char const* as target type
		SCIP_CALL(SCIPcreateVar(scip,
								&vars[i],                   // returns new index
								pchar,               // name
								0,                    // lower bound
								node_max_value[i] + 1,                    // upper bound
								1,             // objective
								SCIP_VARTYPE_INTEGER,   // variable type
								true,                   // initial
								false,                  // forget the rest ...
								NULL, NULL, NULL, NULL, NULL));
		SCIP_CALL(SCIPaddVar(scip, vars[i]));
	}

	// add constraints 
	list<SCIP_CONS*> cons;
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			if (CG[i * N + j] > 0)
			{
				SCIP_CONS* con;
				SCIP_VAR* var[2] = { vars[i], vars[j] };
				double coeff[2] = { 1, 1 };
				std::string s = std::to_string(i * N + j);
				char const* pchar = s.c_str();  //use char const* as target type
				SCIP_CALL(SCIPcreateConsLinear(scip, &con,
											   pchar,                    // name of the constraint
											   2,                            // number of variables to be added to the constraint
											   var,                        // array of SCIP_VAR pointers to variables
											   coeff,                    // array of values of the coeffcients
											   CG[i * N + j],    // lhs
											   SCIPinfinity(scip),                    // rhs
											   true,                   // initial: set this to TRUE if you want the constraint to occur in the root problem
											   true,                  // separate: set this to TRUE if you would like the handler to separate, e.g. generate cuts
											   true,                   // enforce: set this to TRUE if you would like the handler to enforce solutions. This means that when the handler declares an LP or pseudo solution as infeasible, it can resolve infeasibility by adding cuts, reducing the domain of a variable, performing a branching, etc.
											   true,                   // check: set this to TRUE if the constraint handler should check solutions
											   true,                   // propagate: set this to TRUE if you want to propagate solutions, this means tighten variables domains based on constraint information
											   false,                  // local: set this to TRUE if the constraint is only locally valid, e.g., generated in a branch and bound node
											   false,                   // modifiable: set this to TRUE if the constraint may be modified during solution process, e.g. new variables may be added (colum generation)
											   false,                  // dynamic: set this to TRUE if this constraint is subject to aging, this means it will be removed after being inactive for a while (you should also say TRUE to removable in that case) removable set this to TRUE to allow the deletion of the relaxation of the constraint from the LP
											   false,                  // removable
											   false));               // stickingatnode: set this to TRUE if you want the constraint to be kept at the node it was added

				// add the vars belonging to field in this row to the constraint
				// SCIP_CALL(SCIPaddCoefLinear(scip, con, vars[i], 1));
				// SCIP_CALL(SCIPaddCoefLinear(scip, con, vars[j], 1));
				// add the constraint to scip
				SCIP_CALL(SCIPaddCons(scip, con));
				cons.push_back(con);
			}
		}
	}

	// Solve 

	SCIP_CALL(SCIPsolve(scip));

	// Statistics
	//SCIP_CALL(SCIPprintStatistics(scip, NULL));
	//SCIP_CALL(SCIPprintBestSol(scip, NULL, FALSE));
	// get the best found solution from scip
	SCIP_SOL* sol = SCIPgetBestSol(scip);
	int rst = -1;
	if (sol != NULL) // solved successfully
	{
		rst = SCIPgetSolOrigObj(scip, sol);
	}

	// Deinitialization

	// release variables
	for (auto var: vars)
	{
		SCIP_CALL(SCIPreleaseVar(scip, &var));
	}
	for (auto con : cons)
	{
		SCIP_CALL(SCIPreleaseCons(scip, &con));
	}

	SCIP_CALL(SCIPfree(&scip));

	BMScheckEmptyMemory();
	return rst;
}
*/

// CPLEX
int WDGHeuristic::ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value)
{
	int N = (int)node_max_value.size();
	IloEnv env = IloEnv();
	IloModel model = IloModel(env);
	IloExpr sum_obj = IloExpr(env);
	IloNumVarArray var(env);
	IloRangeArray con(env);
	for (int i = 0; i < N; i++)
	{
		var.add(IloNumVar(env, 0, node_max_value[i] + 1, ILOINT));
		sum_obj += var[i];
	}
	model.add(IloMinimize(env, sum_obj));
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			if (CG[i * N + j] > 0)
			{
				con.add(var[i] + var[j] >= CG[i * N + j]);
			}
		}
	}
	model.add(con);
	IloCplex cplex(env);
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	cplex.setParam(IloCplex::TiLim, time_limit - runtime);
	int solution_cost = -1;
	cplex.extract(model);
	cplex.setOut(env.getNullStream());
	int rst = 0;
	if (cplex.solve())
		rst = (int)cplex.getObjValue();
	else
	{
		runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (time_limit > runtime + 1)
		{
			std::cout << "ERROR! remaining time = " << time_limit - runtime << " seconds" << endl;
			cplex.exportModel("error.lp");
			system("pause");
		}
	}
	env.end();
	return rst;
}

