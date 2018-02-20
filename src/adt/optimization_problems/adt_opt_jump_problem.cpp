#include <Utils/utilities.hpp>
#include <adt/optimization_problems/adt_opt_jump_problem.hpp>

#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>


#include <adt/contacts/adt_draco_contact_toe.hpp>
#include <adt/contacts/adt_draco_contact_heel.hpp>

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
	// At timestep 0, we initialize the joint positions of the robot.
	for(size_t i = 0; i < NUM_VIRTUAL; i++){
		// append to opt_var_manager the virtual joints initial positions
	}
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
		// append to opt_var_manager the actuator z and delta states	
	}	

	// set variable manager initial condition offset = NUM_VIRTUAL + NUM_STATES_PER_ACTUATOR*NUM_ACT 

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


