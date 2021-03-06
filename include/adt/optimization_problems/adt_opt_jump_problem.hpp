#ifndef ADT_JUMP_OPTIMIZATION_PROBLEM_H
#define ADT_JUMP_OPTIMIZATION_PROBLEM_H

#include <adt/optimization_problems/adt_opt_problem_main.hpp>


#include <adt/containers/adt_opt_variable.hpp>
#include <adt/containers/adt_opt_variable_manager.hpp>
#include <adt/containers/adt_constraint_list.hpp>
#include <adt/containers/adt_contact_list.hpp>

#include <adt/objective_functions/adt_jump_objective_function.hpp>
//#include <adt/objective_functions/adt_objective_function_main.hpp>

#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"

class Jump_Opt: public Optimization_Problem_Main{
public:
  Jump_Opt();
  ~Jump_Opt();	

  ADT_Opt_Variable_Manager			opt_var_manager;
  DracoModel*                   robot_model;

  Contact_List 								  contact_list;

  Constraint_List 							td_constraint_list; // Time Dependent Constraint List, exists for all timesteps
  Constraint_List 							ti_constraint_list;	// Time Independent Constraint List, exists for at a particular timestep
  

  sejong::Vector 								robot_q_init;
  sejong::Vector 								robot_qdot_init; 
  sejong::Vector 								act_z_init;
  sejong::Vector 								act_zdot_init;
  sejong::Vector 								act_delta_init;
  sejong::Vector 								act_delta_dot_init;  	  	


  int 										      N_total_knotpoints;
  int 										      N_d; // Number of friction cone basis vectors

  double										    h_dt_min;
  double										    max_normal_force;
  double										    max_tangential_force;	  	

  Jump_Objective_Function				objective_function;



  int constraint_size; // Unused

  void get_var_manager(ADT_Opt_Variable_Manager* &var_manager_out){
    var_manager_out = &opt_var_manager;
  }

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
  void initialize_ti_constraint_list();  

  void initialize_opt_vars();

  void initialize_specific_variable_bounds();

  void initialize_objective_func();


};

#endif