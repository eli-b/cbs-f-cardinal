/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2019
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <iostream>


#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>(), "output file for schedule")
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")		
		("warehouseWidth,b", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instacnes")
		("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("MaxMDDs", po::value<int>(), "maximum number of MDDs saved for each pair of agents")

		("PC,p", po::value<bool>()->default_value(true), "conflict prioirtization")
		("bypass", po::value<bool>()->default_value(true), "Bypass1")
		("heuristics,h", po::value<std::string>()->default_value("NONE"), "heuristics for the high-level search (NONE, CG,DG, WDG)")
		("disjointSplitting", po::value<bool>()->default_value(true), "disjoint splitting")
		("rectangleReasoning", po::value<bool>()->default_value(false), "Using rectangle reasoning")
		("corridorReasoning", po::value<bool>()->default_value(false), "Using corridor reasoning")
		("targetReasoning", po::value<bool>()->default_value(false), "Using target reasoning")
		("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)")

	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	// read the map file and construct its two-dim array
	MapLoader ml(vm["map"].as<string>(), vm["rows"].as<int>(), 
		vm["cols"].as<int>(), vm["obs"].as<int>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>(), vm["warehouseWidth"].as<int>());
 
	srand(vm["seed"].as<int>());

	heuristics_type h;
	if (vm["heuristics"].as<string>() == "NONE")
		h = heuristics_type::NONE;
	else if (vm["heuristics"].as<string>() == "CG")
		h = heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = heuristics_type::WDG;
	else
	{
		std::cout <<"WRONG HEURISTICS NAME!" << std::endl;
		return -1;
	}

	int runs = vm["restart"].as<int>();
	assert(runs > 0);
	ICBSSearch icbs(ml, al, 1.0, h, vm["PC"].as<bool>(), vm["screen"].as<int>());
	icbs.disjoint_splitting = vm["disjointSplitting"].as<bool>();
	icbs.bypass = vm["bypass"].as<bool>();
	icbs.rectangle_reasoning = vm["rectangleReasoning"].as<bool>();
    icbs.corridor_reasoning = vm["corridorReasoning"].as<bool>();
    icbs.target_reasoning = vm["targetReasoning"].as<bool>();

	if(vm.count("MaxMDDs"))
		icbs.max_num_of_mdds = vm["MaxMDDs"].as<int>();

	double runtime = 0;
	int initial_h = 0;
	for (int i = 0; i < runs; i++)
	{
		icbs.clear();
		icbs.runICBSSearch(vm["cutoffTime"].as<double>(), initial_h);
		runtime += icbs.runtime;
		if (icbs.solution_found)
			break;
		initial_h = (int)icbs.min_f_val - icbs.dummy_start->g_val;
	}
	icbs.runtime  = runtime;
	if (vm.count("output"))
		icbs.saveResults(vm["output"].as<std::string>(), vm["agents"].as<string>());
	icbs.clearSearchEngines();
	return 0;

}
