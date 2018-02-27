#include <iostream>
#include <Utils/utilities.hpp>

#include <adt/optimization_problems/adt_opt_jump_problem.hpp>

int main(int argc, char **argv){
	Optimization_Problem_Main* 	adt_jump_problem = new Jump_Opt();

	// Prepare Variable Containers
	std::vector<double> x_vars;
	std::vector<double> x_vars_low;
	std::vector<double> x_vars_upp;	

	std::vector<double> F_eval;
	std::vector<double> F_eval_low;
	std::vector<double> F_eval_upp;

	adt_jump_problem->get_init_opt_vars(x_vars);
	std::cout << std::endl;
	std::cout << std::endl;	
	std::cout << "[Main Test] Initialized Initial Value of Optimization Variables" << std::endl;
	std::cout << "[Main Test]                    Number of Optimization Variables: " << x_vars.size() << std::endl;	

	adt_jump_problem->get_opt_vars_bounds(x_vars_low, x_vars_upp);
	std::cout << "[Main Test] Initialized Bounds of Optimization Variables" << std::endl;
	std::cout << "[Main Test]  						  Num of Lower Bounds: " << x_vars_low.size() << std::endl;	
	std::cout << "[Main Test]  						  Num of Upper Bounds: " << x_vars_upp.size() << std::endl;		

	adt_jump_problem->get_F_bounds(F_eval_low, F_eval_upp);
	std::cout << "[Main Test] Initialized Bounds of Functions" << std::endl;
	std::cout << "[Main Test]  			  Num of Lower Bounds: " << F_eval_low.size() << std::endl;	
	std::cout << "[Main Test]  			  Num of Upper Bounds: " << F_eval_upp.size() << std::endl;		


	std::cout << std::endl;
	std::cout << std::endl;

	adt_jump_problem->compute_F(F_eval);
	std::cout << "[Main Test] Size of F_eval: " << F_eval.size() << std::endl;		

	delete adt_jump_problem;

	return 0;
}