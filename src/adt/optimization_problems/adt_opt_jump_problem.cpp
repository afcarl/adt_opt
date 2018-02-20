#include <Utils/utilities.hpp>
#include <adt/optimization_problems/adt_opt_jump_problem.hpp>
#include "DracoP1Rot_Definition.h"

Jump_Opt::Jump_Opt(){
	problem_name = "Draco Jump Optimization Problem";

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	robot_q_init.setZero(); 
	robot_qdot_init.setZero();
}

Jump_Opt::~Jump_Opt(){
	std::cout << "[Jump Opt] Destructor Called" << std::endl;
}


// Problem Specific Initialization
void Jump_Opt::Initialization(){}
void Jump_Opt::initialize_starting_configuration(){}
void Jump_Opt::initialize_contact_list(){}
void Jump_Opt::initialize_td_constraint_list(){}
void Jump_Opt::initialize_opt_vars(){}
void Jump_Opt::initialize_objective_func(){}


// SNOPT Interface
void Jump_Opt::get_init_opt_vars(std::vector<double> &x_vars){}
void Jump_Opt::get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp){}   	  	
void Jump_Opt::get_current_opt_vars(std::vector<double> &x_vars_out){}
void Jump_Opt::update_opt_vars(std::vector<double> &x_vars){} 	  		

void Jump_Opt::get_F_bounds(std::vector<double> &F_low, std::vector<double> &F_upp){}
void Jump_Opt::get_F_obj_Row(int &obj_row){}

void Jump_Opt::compute_F(std::vector<double> &F_eval){}
void Jump_Opt::compute_F_constraints(std::vector<double> &F_eval){}
void Jump_Opt::compute_F_objective_function(double &result_out){}

void Jump_Opt::compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG){}
void Jump_Opt::compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA){}


