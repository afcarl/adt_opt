#include <iostream>
#include <Utils/utilities.hpp>

#include <adt/optimization_problems/adt_opt_jump_problem.hpp>

int main(int argc, char **argv){
	Optimization_Problem_Main* 	adt_jump_problem = new Jump_Opt();
	delete adt_jump_problem;

	return 0;
}