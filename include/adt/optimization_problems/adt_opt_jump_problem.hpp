#ifndef ADT_JUMP_OPTIMIZATION_PROBLEM_H
#define ADT_JUMP_OPTIMIZATION_PROBLEM_H

#include <adt/optimization_problems/adt_opt_problem_main.hpp>
#include <adt/containers/adt_opt_variable_manager.hpp>

#include <adt/contacts/adt_draco_contact_toe.hpp>
#include <adt/contacts/adt_draco_contact_heel.hpp>

class Jump_Opt: public Optimization_Problem_Main{
public:
	Jump_Opt();
	~Jump_Opt();	

	ADT_Opt_Variable_Manager					opt_var_manager;
/*	Constraint_List 							td_constraint_list; // Time Dependent Constraint List, exists for all timesteps
	Constraint_List 							ti_constraint_list;	// Time Independent Constraint List, exists for at a particular timestep
*/

	sejong::Vector 								robot_q_init;
  	sejong::Vector 								robot_qdot_init; 

  	int 										N_total_knotpoints;

 //	WBC_Objective_Function						objective_function;

  	
  	int constraint_size; // Unused

  	// Interface to SNOPT -------------------------------------------------------------------

  	void get_init_opt_vars(std::vector<double> &x_vars);   	
  	void get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp);   	  	

  	void update_opt_vars(std::vector<double> &x_vars); 	  		
  	void get_current_opt_vars(std::vector<double> &x_vars_out);   	  	  	

	void get_F_bounds(std::vector<double> &F_low, std::vector<double> &F_upp);
	void get_F_obj_Row(int &obj_row);	

  	void compute_F(std::vector<double> &F_eval);
  	void compute_F_constraints(std::vector<double> &F_eval);
  	void compute_F_objective_function(double &result_out);

  	void compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG);
	void compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA);

private:
	void Initialization();
	void initialize_starting_configuration();
	void initialize_contact_list();
	void initialize_td_constraint_list();

	void initialize_opt_vars();

	void initialize_objective_func();


};

#endif