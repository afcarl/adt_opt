#include <iostream>
#include <adt/optimization_problems/adt_opt_jump_problem.hpp>
#include <adt/adt_snopt_wrapper.hpp>

int main(int argc, char **argv)
{
	std::cout << "[Main] Running Actuator Dynamics Optimization Problem" << std::endl;
	Optimization_Problem_Main* 	adt_problem = new Jump_Opt();

	//snopt_wrapper::solve_problem_partial_gradients(wbc_problem);	
	snopt_wrapper::solve_problem_no_gradients(adt_problem);

	delete adt_problem;
	return 0;
}
