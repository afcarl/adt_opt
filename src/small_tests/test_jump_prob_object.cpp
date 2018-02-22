#include <iostream>
#include <Utils/utilities.hpp>

#include <adt/optimization_problems/adt_opt_jump_problem.hpp>

int main(int argc, char **argv){
	Optimization_Problem_Main* 	adt_jump_problem = new Jump_Opt();

	std::vector<double> F_eval;
	adt_jump_problem->compute_F_constraints(F_eval);

	delete adt_jump_problem;

	return 0;
}