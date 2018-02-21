#include <Utils/utilities.hpp>
#include <adt/optimization_problems/adt_opt_jump_problem.hpp>
#include <adt/optimization_constants.hpp>

#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>

#include <adt/contacts/adt_draco_contact_toe.hpp>
#include <adt/contacts/adt_draco_contact_heel.hpp>

#include <string>

Jump_Opt::Jump_Opt(){
	problem_name = "Draco Jump Optimization Problem";

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	robot_q_init.setZero(); 
	robot_qdot_init.setZero();
	Initialization();
}

Jump_Opt::~Jump_Opt(){
	std::cout << "[Jump Opt] Destructor Called" << std::endl;
}


// Problem Specific Initialization -------------------------------------
void Jump_Opt::Initialization(){
	std::cout << "[Jump_Opt] Initialization Called" << std::endl;
	N_total_knotpoints = 1;
	initialize_starting_configuration();
	initialize_contact_list();
	initialize_opt_vars();


}
void Jump_Opt::initialize_starting_configuration(){
 // Set Virtual Joints
  // x_pos
  robot_q_init[0] = 0.01;
  // z_pos
  robot_q_init[1] = 0.87 - 0.19;
  // Ry_rot
  robot_q_init[2] = 0.00; //1.135; //1.131; 	

  robot_q_init[SJJointID::bodyPitch] = -1.0;
  robot_q_init[SJJointID::kneePitch] = 2.0;
  robot_q_init[SJJointID::anklePitch] = -1.0;

}

void Jump_Opt::initialize_contact_list(){
	Draco_Toe_Contact* toe_contact = new Draco_Toe_Contact();
	Draco_Heel_Contact* heel_contact = new Draco_Heel_Contact();
	// Append to contact list
	contact_list.append_contact(toe_contact);
	contact_list.append_contact(heel_contact);
}
void Jump_Opt::initialize_td_constraint_list(){}



void Jump_Opt::initialize_opt_vars(){
	// Optimization Variable Order:
	// opt_init = [q_virt, z, qdot_virt, zdot, delta, delta_dot]
	// opt_td_k = [q_virt_k, z_k, qdot_virt_k, zdot_k, delta_k, delta_dot_k, u_k, Fij_k, Bij_k]
	// opt_var = [opt_init, opt_td_1, opt_td_2, ..., opt_td_k]

	// Append to opt_var_manager the initial configuration
	// ------------------------------------------------------------
	// Set Initial Conditions
	// ------------------------------------------------------------
	// At knotpoint 0, we initialize the joint positions of the robot.
	// [q_virt, z]
	for(size_t i = 0; i < NUM_VIRTUAL; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("virtual_q_state_" + std::to_string(i), VAR_TYPE_Q, 0, robot_q_init[i], robot_q_init[i] - OPT_ZERO_EPS, robot_q_init[i] + OPT_ZERO_EPS) );
     }
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
		// Initial values must be fixed
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_z_state_" + std::to_string(i), VAR_TYPE_Z, 0, 0.0, -OPT_INFINITY, OPT_INFINITY) );
	}     
	// [qdot_virt, zdot]
	for(size_t i = 0; i < NUM_VIRTUAL; i++){
	    opt_var_manager.append_variable(new ADT_Opt_Variable("virtual_qdot_state_" + std::to_string(i), VAR_TYPE_QDOT, 0, robot_qdot_init[i], robot_qdot_init[i] - OPT_ZERO_EPS, robot_qdot_init[i] + OPT_ZERO_EPS) );	
     }
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_zdot_state_" + std::to_string(i), VAR_TYPE_ZDOT, 0, 0.0, -OPT_INFINITY, OPT_INFINITY) );        
	}
	// [delta, delta_dot]
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_delta_state_" + std::to_string(i), VAR_TYPE_DELTA, 0, 0.0, -OPT_INFINITY, OPT_INFINITY) );
	}
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_delta_dot_state_" + std::to_string(i), VAR_TYPE_DELTA_DOT, 0, 0.0, -OPT_INFINITY, OPT_INFINITY) );
	}

	// set variable manager initial condition offset = NUM_VIRTUAL*2 + (NUM_STATES_PER_ACTUATOR*NUM_ACT)*2 
	int initial_conditions_offset = NUM_VIRTUAL*2 + (NUM_STATES_PER_ACTUATOR*NUM_ACT_JOINT)*2;
	std::cout << "[Jump_Opt] Predicted Number of states : " << initial_conditions_offset << std::endl;
	std::cout << "[Jump_Opt] Actual : " << opt_var_manager.get_size() << std::endl;	 
	opt_var_manager.initial_conditions_offset = initial_conditions_offset;

	// ------------------------------------------------------------------
	// Set Time Dependent Variables
	// ------------------------------------------------------------------
	for(size_t k = 1; k < N_total_knotpoints; k++){
		// add to variable manager 
	}

}
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


