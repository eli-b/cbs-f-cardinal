﻿/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2019
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "CBS.h"

/* Declare some static utility functions */
static void usage();

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("output,o", po::value<string>(), "output file for schedule")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("stats", po::value<bool>()->default_value(false), "write to files some statistics")
		// params for instance generators
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")
		("warehouseWidth", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instacnes")

		// params for CBS
		("heuristics", po::value<string>()->default_value("CG"), "heuristics for the high-level search (Zero, CG,DG, WDG)")
		("prioritizingConflicts", po::value<bool>()->default_value(true), "conflict prioirtization. If true, conflictSelection is used as a tie-breaking rule.")
		("conflictSelection", po::value<string>()->default_value("Earliest"), 
			"conflict selection (Random\n Earliest,\n Conflicts: most conflicts with others\n MConstraints: most constraints\n FConstraints: fewest constraints\n Width: thinest MDDs\n Singletons: most singletons in MDDs)")
		("nodeSelection", po::value<string>()->default_value("Conflicts"),
			"conflict selection (Random\n H: smallest h value\n Depth: depth-first manner\n Conflicts: most conflicts\n ConflictPairs: most conflictinng pairs of agents\n MVC: MVC on the conflict graph)")
		("bypass", po::value<bool>()->default_value(false), "Bypass1")
		("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
		("rectangleReasoning", po::value<string>()->default_value("None"), "rectangle reasoning strategy (None, R, RM, Disjoint)")
		("corridorReasoning", po::value<string>()->default_value("None"), " corridor reasoning strategy (None, C, Disjoint")
		("mutexReasoning", po::value<bool>()->default_value(false), "Using mutex reasoning")
		("targetReasoning", po::value<bool>()->default_value(false), "Using target reasoning")
		("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)")
		("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver")
		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		usage();
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);
	/////////////////////////////////////////////////////////////////////////
	// check the correctness and consistence of params
	if (vm["sipp"].as<bool>() && vm["targetReasoning"].as<bool>())
	{
		cerr << "SIPP cannot work together with target reasoning!" << endl;
		return -1;
	}

  heuristics_type h;
	if (vm["heuristics"].as<string>() == "Zero")
		h = heuristics_type::ZERO;
	else if (vm["heuristics"].as<string>() == "CG")
		h = heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = heuristics_type::WDG;
	else
	{
		cout << "WRONG heuristics strategy!" << endl;
		return -1;
	}

	rectangle_strategy r;
	if (vm["rectangleReasoning"].as<string>() == "None")
		r = rectangle_strategy::NR;
	else if (vm["rectangleReasoning"].as<string>() == "R")
		r = rectangle_strategy::R;
	else if (vm["rectangleReasoning"].as<string>() == "RM")
		r = rectangle_strategy::RM;
	else if (vm["rectangleReasoning"].as<string>() == "Disjoint")
		r = rectangle_strategy::DISJOINTR;
	else
	{
		cout << "WRONG rectangle reasoning strategy!" << endl;
		return -1;
	}

	corridor_strategy c;
	if (vm["corridorReasoning"].as<string>() == "None")
		c = corridor_strategy::NC;
	else if (vm["corridorReasoning"].as<string>() == "C")
		c = corridor_strategy::C;
	else if (vm["corridorReasoning"].as<string>() == "Disjoint")
		c = corridor_strategy::DISJOINTC;
	else
	{
		cout << "WRONG corridor reasoning strategy!" << endl;
		return -1;
	}

	conflict_selection conflict;
	if (vm["conflictSelection"].as<string>() == "Random")
		conflict = conflict_selection::RANDOM;
	else if (vm["conflictSelection"].as<string>() == "Earliest")
		conflict = conflict_selection::EARLIEST;
	else if (vm["conflictSelection"].as<string>() == "Conflicts")
		conflict = conflict_selection::CONFLICTS;
	else if (vm["conflictSelection"].as<string>() == "MConstraints")
		conflict = conflict_selection::MCONSTRAINTS;
	else if (vm["conflictSelection"].as<string>() == "FConstraints")
		conflict = conflict_selection::FCONSTRAINTS;
	else if (vm["conflictSelection"].as<string>() == "Width")
		conflict = conflict_selection::WIDTH;
	else if (vm["conflictSelection"].as<string>() == "Singletons")
		conflict = conflict_selection::SINGLETONS;
	else
	{
		cout << "WRONG conflict selection strategy!" << endl;
		return -1;
	}

	node_selection n;
	if (vm["nodeSelection"].as<string>() == "Random")
		n = node_selection::NODE_RANDOM;
	else if (vm["nodeSelection"].as<string>() == "H")
		n = node_selection::NODE_H;
	else if (vm["nodeSelection"].as<string>() == "Depth")
		n = node_selection::NODE_DEPTH;
	else if (vm["nodeSelection"].as<string>() == "Conflicts")
		n = node_selection::NODE_CONFLICTS;
	else if (vm["nodeSelection"].as<string>() == "ConflictPairs")
		n = node_selection::NODE_CONFLICTPAIRS;
	else if (vm["nodeSelection"].as<string>() == "MVC")
		n = node_selection::NODE_MVC;
	else
	{
		cout << "WRONG node selection strategy!" << endl;
		return -1;
	}
	srand((int)time(0));

	///////////////////////////////////////////////////////////////////////////
	// load the instance
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>(),
		vm["rows"].as<int>(), vm["cols"].as<int>(), vm["obs"].as<int>(), vm["warehouseWidth"].as<int>());

	srand(vm["seed"].as<int>());

	int runs = vm["restart"].as<int>();
	
	//////////////////////////////////////////////////////////////////////
	// initialize the solver
	CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
	cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
	cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
	cbs.setBypass(vm["bypass"].as<bool>());
	cbs.setRectangleReasoning(r);
	cbs.setCorridorReasoning(c);
	cbs.setHeuristicType(h);
	cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
	cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
	cbs.setConflictSelectionRule(conflict);
	cbs.setNodeSelectionRule(n);
	cbs.setSavingStats(vm["stats"].as<bool>());
	//////////////////////////////////////////////////////////////////////
	// run
	double runtime = 0;
	int min_f_val = 0;
	for (int i = 0; i < runs; i++)
	{
		cbs.clear();
		cbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
		runtime += cbs.runtime;
		if (cbs.solution_found)
			break;
		min_f_val = (int)cbs.min_f_val;
		cbs.randomRoot = true;
	}
	cbs.runtime = runtime;
	if (vm.count("output"))
		cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
	if (vm["stats"].as<bool>())
	{
		cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
	}
	cbs.clearSearchEngines();
	return 0;

}



/*
Prints out usage help.
*/
static void usage()
{
	//("help", "produce help message")
	//("screen", po::value<int>(), "screen output on or off (default)")
	//("instance", po::value<std::string>(), "input instance")
	//("exp", po::value<std::string>(), "experiment")
	//("strat", po::value<int>(), "branching strategy")
	//("sbnodes", po::value<int>(), "number of SB nodes")
	//("diag", po::value<int>(), "diagnostic trigger")
	//("root", po::value<int>(), "limit cuts and heuristics to root node")
	//("learningAlg", po::value<int>(), "learning algorithm")
	//("c", po::value<std::string>(), "SVM parameter, default 0.1")
	//("alpha", po::value<std::string>(), "label threshold, default 0.2")
	//("loss", po::value<int>(), "loss function for learning")
	//("varPerNode", po::value<int>(), "number of SB variables per node")
	//("varSorting", po::value<int>(), "variable sorting criterion")
	//("maxtime", po::value<double>(), "time cutoff")
	//("restart", po::value<int>(), "restart from root")
	//("kernel", po::value<int>(), "2nd degree polynomial kernel, default 1")
	//("seed", po::value<int>(), "random seed for CPLEX, default 1")
	//("cutoff", po::value<int>(), "supply cutoff value, default 0")
	//("collect", po::value<int>(), "collect data and abort or not, default 0")
	//("whichpc", po::value<int>(), "default 0 means PC features are set after phase 1, 1 means along the way")
	//("desc", po::value<std::string>(), "description to be added to filenames to distinguish them");

	fprintf(stderr, "Usage: optimize instance exp strat [options]\n");
	fprintf(stderr, "Arguments:\n");
	fprintf(stderr, "	help		-> this list\n");
	fprintf(stderr, "	screen		-> screen output on(=1) or off(=0) (default: 0)\n");
	fprintf(stderr, "	instance	-> MIP in MPS format\n");
	fprintf(stderr, "	exp		-> experiment name\n");
	fprintf(stderr, "	strat		-> branching strategy:\n");
	fprintf(stderr, "				-1: CPLEX Default\n");
	fprintf(stderr, "				-2: FSB\n");
	fprintf(stderr, "				-3: Most Infeasible\n");
	fprintf(stderr, "				-4: SB\n");
	fprintf(stderr, "				-5: PC\n");
	fprintf(stderr, "				 3: Hybrid SB/PC\n");
	fprintf(stderr, "				 6: ML\n");
	fprintf(stderr, "				 7: ML + Problem Features \n");
	fprintf(stderr, "	sbnodes		-> num. of SB nodes if strat:={3,6,7}\n");
	fprintf(stderr, "	varPerNode	-> num. of variables per SB node if strat:={3,6,7}\n");
	fprintf(stderr, "	varSorting	-> variable sorting criterion if strat:={-4,3,6,7}\n");
	fprintf(stderr, "	learningAlg	-> learning algorithm if strat={6,7}\n");
	fprintf(stderr, "				 1: SVM-Rank\n");
	fprintf(stderr, "				 2: NDCG\n");
	fprintf(stderr, "				 3: Regression\n");
	fprintf(stderr, "	loss		-> SVM loss function variant, (default: 2)\n");
	fprintf(stderr, "	c		-> SVM parameter (default: 0.1)\n");
	fprintf(stderr, "	alpha		-> Fraction of max. SB score to get label 1 (default: 0.2)\n");
	fprintf(stderr, "	diag		-> diagnostic mode (default: 0)\n");
	fprintf(stderr, "	root		-> root-only cuts and heuristics (default: 0)\n");
	fprintf(stderr, "	maxtime		-> time cutoff in sec. (default: 7200)\n");
	fprintf(stderr, "	restart		-> Restart after learning(=1) or not (=0), (default: 0)\n");
	fprintf(stderr, "	kernel		-> Add interaction features with Kernel(=1) or not(=0), (default: 1)\n");
	fprintf(stderr, "	seed		-> CPLEX random seed (default: 1)\n");
	fprintf(stderr, "	cutoff		-> Use instance's optimal value as cutoff(=1) or not(=0), (default: 0)\n");
	fprintf(stderr, "	whichpc		-> Use PC scores as search goes(=1) or after Phase 1(=0), (default: 0)\n");
	fprintf(stderr, "whichFeatures	-> which features to include, (default 0) \n");
	fprintf(stderr, "				0: All\n");
	fprintf(stderr, "				1: Static\n");
	fprintf(stderr, "				2: Active\n");
	fprintf(stderr, "				3: Compact\n");
	fprintf(stderr, "	desc		-> Optional string describing this experiment\n");
	fprintf(stderr, "Exiting...\n");
}