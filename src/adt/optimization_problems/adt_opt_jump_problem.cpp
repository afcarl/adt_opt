#include <Utils/utilities.hpp>
#include <adt/optimization_problems/adt_opt_jump_problem.hpp>
#include <adt/optimization_constants.hpp>

#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>

#include <adt/contacts/adt_draco_contact_toe.hpp>
#include <adt/contacts/adt_draco_contact_heel.hpp>

#include <adt/hard_constraints/adt_floor_2d_contact_lcp_constraint.hpp>
#include <adt/hard_constraints/adt_friction_cone_2d_constraint.hpp>
#include <adt/hard_constraints/adt_linear_back_euler_time_integration_constraint.hpp>
#include <adt/hard_constraints/adt_position_kinematic_constraint.hpp>


#include <string>

#include <adt/optimization_variable_limits/adt_jump_variable_limits.hpp>
#define _USE_MATH_DEFINES
#include <cmath>

Jump_Opt::Jump_Opt(){
	problem_name = "Draco Jump Optimization Problem";

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	act_z_init.resize(NUM_ACT_JOINT);
	act_zdot_init.resize(NUM_ACT_JOINT);
	act_delta_init.resize(NUM_ACT_JOINT);
	act_delta_dot_init.resize(NUM_ACT_JOINT);


	robot_q_init.setZero(); 
	robot_qdot_init.setZero();
	act_z_init.setZero();
	act_zdot_init.setZero();
	act_delta_init.setZero();
	act_delta_dot_init.setZero();

	Initialization();
}

Jump_Opt::~Jump_Opt(){
	std::cout << "[Jump Opt] Destructor Called" << std::endl;
}


// Problem Specific Initialization -------------------------------------
void Jump_Opt::Initialization(){

	std::cout << "[Jump_Opt] Initialization Called" << std::endl;
	N_total_knotpoints = 1;

	N_d = ND_2D_CONST; // Number of friction cone basis vectors

	h_dt_min = 0.001; // Minimum knotpoint timestep
  	max_normal_force = 1e10;//10000; // Newtons
  	max_tangential_force = 10000; // Newtons  	  	

	initialize_starting_configuration();
	initialize_contact_list();
	initialize_opt_vars();

	initialize_td_constraint_list();
  	initialize_ti_constraint_list();


	initialize_objective_func();

}
void Jump_Opt::initialize_starting_configuration(){
  DracoActuatorModel* actuator_model = DracoActuatorModel::GetDracoActuatorModel();
 // Set Virtual Joints
  // x_pos
  robot_q_init[0] = 0.01;
  // z_pos
  robot_q_init[1] = 0.831165 + 0.00679965;//0.87 - 0.19;
  // Ry_rot
  robot_q_init[2] = 0.00; //1.135; //1.131; 	

  // Need to find equivalent z_positions
  robot_q_init[SJJointID::bodyPitch] = -M_PI/4.0; //-1.0;
  robot_q_init[SJJointID::kneePitch] = M_PI/2.0; //2.0;
  robot_q_init[SJJointID::anklePitch] = -M_PI/4.0;//-1.0;

  // Find equivalent starting z positions
  actuator_model->getFull_act_pos_z(robot_q_init.tail(NUM_ACT_JOINT), act_z_init);
}

void Jump_Opt::initialize_contact_list(){
	Draco_Toe_Contact* toe_contact = new Draco_Toe_Contact();
	Draco_Heel_Contact* heel_contact = new Draco_Heel_Contact();
	// Append to contact list
	contact_list.append_contact(toe_contact);
	contact_list.append_contact(heel_contact);
}

void Jump_Opt::initialize_td_constraint_list(){
	int toe_contact_index = 0;
	int heel_contact_index = 1;	
    td_constraint_list.append_constraint(new Floor_2D_Contact_LCP_Constraint(&contact_list, toe_contact_index)); 
    td_constraint_list.append_constraint(new Floor_2D_Contact_LCP_Constraint(&contact_list, heel_contact_index)); 
    td_constraint_list.append_constraint(new Friction_Cone_2D_Constraint(&contact_list, toe_contact_index));
    td_constraint_list.append_constraint(new Friction_Cone_2D_Constraint(&contact_list, heel_contact_index));     
    td_constraint_list.append_constraint(new Linear_Back_Euler_Time_Integration_Constraint(&contact_list));
}

void Jump_Opt::initialize_ti_constraint_list(){
    int des_knotpoint = N_total_knotpoints/2;
    int pre_final_knotpoint = N_total_knotpoints - 1;
    double min_des_z_height = 0.05;//0.005;
    //double des_hip_ori = -M_PI/2.0;
    //ti_constraint_list.append_constraint(new Position_2D_Kinematic_Constraint(des_knotpoint, SJLinkID::LK_FootToe, Z_DIM, 0.0, min_des_z_height)); 
    //ti_constraint_list.append_constraint(new Position_2D_Kinematic_Constraint(1, SJLinkID::LK_FootToe, Z_DIM, 0.0, OPT_ZERO_EPS));     
    ti_constraint_list.append_constraint(new Position_2D_Kinematic_Constraint(1, SJLinkID::LK_FootToe, Z_DIM, 0.0, OPT_ZERO_EPS));     

    //ti_constraint_list.append_constraint(new Position_2D_Kinematic_Constraint(des_knotpoint, SJLinkID::LK_FootToe, Z_DIM, 0.0, min_des_z_height));     
    //ti_constraint_list.append_constraint(new Position_2D_Kinematic_Constraint(N_total_knotpoints, SJLinkID::LK_body, Z_DIM, min_des_z_height - OPT_ZERO_EPS, min_des_z_height + OPT_ZERO_EPS)); 
}



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
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_z_state_" + std::to_string(i), VAR_TYPE_Z, 0, act_z_init[i], -OPT_INFINITY, OPT_INFINITY) );
	}     
	// [qdot_virt, zdot]
	for(size_t i = 0; i < NUM_VIRTUAL; i++){
	    opt_var_manager.append_variable(new ADT_Opt_Variable("virtual_qdot_state_" + std::to_string(i), VAR_TYPE_QDOT, 0, robot_qdot_init[i], robot_qdot_init[i] - OPT_ZERO_EPS, robot_qdot_init[i] + OPT_ZERO_EPS) );	
     }
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_zdot_state_" + std::to_string(i), VAR_TYPE_ZDOT, 0, act_zdot_init[i], -OPT_INFINITY, OPT_INFINITY) );        
	}
	// [delta, delta_dot]
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_delta_state_" + std::to_string(i), VAR_TYPE_DELTA, 0, act_delta_init[i], -OPT_INFINITY, OPT_INFINITY) );
	}
	for(size_t i = 0; i < NUM_ACT_JOINT; i++){
        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_delta_dot_state_" + std::to_string(i), VAR_TYPE_DELTA_DOT, 0, act_delta_dot_init[i], -OPT_INFINITY, OPT_INFINITY) );
	}

	// set variable manager initial condition offset = NUM_VIRTUAL*2 + (NUM_STATES_PER_ACTUATOR*NUM_ACT)*2 
	int initial_conditions_offset = NUM_VIRTUAL*2 + (NUM_STATES_PER_ACTUATOR*NUM_ACT_JOINT)*2;
	std::cout << "[Jump_Opt] Predicted Number of states : " << initial_conditions_offset << std::endl;
	std::cout << "[Jump_Opt] Actual : " << opt_var_manager.get_size() << std::endl;	 
	opt_var_manager.initial_conditions_offset = initial_conditions_offset;


	Jump_Opt_Variable_Limits opt_var_limits;
	// ------------------------------------------------------------------
	// Set Time Dependent Variables
	// ------------------------------------------------------------------
	for(size_t k = 1; k < N_total_knotpoints + 1; k++){

		for(size_t i = 0; i < NUM_VIRTUAL; i++){
	        opt_var_manager.append_variable(new ADT_Opt_Variable("virtual_q_state_" + std::to_string(i), VAR_TYPE_Q, k, robot_q_init[i], opt_var_limits.l_q_virt_limits[i] , opt_var_limits.u_q_virt_limits[i]) );
	     }
		for(size_t i = 0; i < NUM_ACT_JOINT; i++){
			// Initial values must be fixed
	        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_z_state_" + std::to_string(i), VAR_TYPE_Z, k, 0.0, opt_var_limits.l_z_limits[i], opt_var_limits.u_z_limits[i]) );
		}     
		// [qdot_virt, zdot]
		for(size_t i = 0; i < NUM_VIRTUAL; i++){
		    opt_var_manager.append_variable(new ADT_Opt_Variable("virtual_qdot_state_" + std::to_string(i), VAR_TYPE_QDOT, k, robot_qdot_init[i], opt_var_limits.l_qdot_virt_limits[i], opt_var_limits.u_qdot_virt_limits[i]) );	
	     }
		for(size_t i = 0; i < NUM_ACT_JOINT; i++){
	        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_zdot_state_" + std::to_string(i), VAR_TYPE_ZDOT, k, 0.0, opt_var_limits.l_zdot_limits[i], opt_var_limits.u_zdot_limits[i]) );        
		}
		// [delta, delta_dot]
		for(size_t i = 0; i < NUM_ACT_JOINT; i++){
	        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_delta_state_" + std::to_string(i), VAR_TYPE_DELTA, k, 0.0, opt_var_limits.l_delta_limits[i] , opt_var_limits.u_delta_limits[i]) );
		}
		for(size_t i = 0; i < NUM_ACT_JOINT; i++){
	        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_delta_dot_state_" + std::to_string(i), VAR_TYPE_DELTA_DOT, k, 0.0, opt_var_limits.l_delta_dot_limits[i], opt_var_limits.u_delta_dot_limits[i]) );
		}
		// [current_u]
		for(size_t i = 0; i < NUM_ACT_JOINT; i++){
	        opt_var_manager.append_variable(new ADT_Opt_Variable("actuator_current_u_" + std::to_string(i), VAR_TYPE_U, k, 0.0, opt_var_limits.l_current_limits[i], opt_var_limits.u_current_limits[i]) );
		}
		// [Fr]
		for(size_t i = 0; i < contact_list.get_size(); i++){
			int contact_dim = contact_list.get_contact(i)->contact_dim;		
			for (size_t j = 0; j < contact_dim; j++){
				if (j == 0){
					// Apply tangential force constraints on x direction
			        opt_var_manager.append_variable(new ADT_Opt_Variable("Fr_x_" + std::to_string(i), VAR_TYPE_FR, k, 0.0, -max_tangential_force, max_tangential_force) );
				}else if(j == 1){
					// Apply normal force constraints on z direction		        
			        opt_var_manager.append_variable(new ADT_Opt_Variable("Fr_z_" + std::to_string(i), VAR_TYPE_FR, k, 0.0, 0.0, max_normal_force) );
				}
			}
		}
		// [Beta]		
		for(size_t i = 0; i < contact_list.get_size(); i++){
			for(size_t j = 0; j < N_d; j++){
		        opt_var_manager.append_variable(new ADT_Opt_Variable("Beta_c" + std::to_string(i) + "_b" + std::to_string(j) , VAR_TYPE_BETA, k, 0.0, 0.0, OPT_INFINITY) );
			}
		}
		
		// [h_dt] knotpoint timestep
        opt_var_manager.append_variable(new ADT_Opt_Variable("h_dt_" + std::to_string(k) , VAR_TYPE_H, k, h_dt_min, h_dt_min, OPT_INFINITY) );
	}
  // Assign total knotpoints
  opt_var_manager.total_knotpoints = N_total_knotpoints;


	// Compute the size of time independent variables
	opt_var_manager.compute_size_time_dep_vars();
	int size_of_time_dep_vars = NUM_VIRTUAL*2 + NUM_ACT_JOINT*5 + contact_list.get_size()*2 + contact_list.get_size()*N_d + N_total_knotpoints;
	std::cout << "[Jump_Opt] Predicted Size of Time Dependent Vars : " << size_of_time_dep_vars << std::endl;
	std::cout << "[Jump_Opt] Actual : " << opt_var_manager.get_size_timedependent_vars() << std::endl;	 
	int predicted_size_of_F = size_of_time_dep_vars*N_total_knotpoints;
	std::cout << "[Jump_Opt] Predicted Size of Opt Vars: " << predicted_size_of_F << std::endl;

}
void Jump_Opt::initialize_objective_func(){
  // |F_td| = num of time dependent constraint functions
  // |F_ti| = num of time independent constraint functions	
  // T = total timesteps
  objective_function.objective_function_index = td_constraint_list.get_num_constraint_funcs()*N_total_knotpoints + ti_constraint_list.get_num_constraint_funcs();

  std::cout << "[Jump_Opt] Objective Function has index: " << objective_function.objective_function_index << std::endl;

}


// SNOPT Interface
// Remember to apply the initial conditions offset for the optimization variables 
void Jump_Opt::get_init_opt_vars(std::vector<double> &x_vars){
  opt_var_manager.get_init_opt_vars(x_vars);
}
void Jump_Opt::get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp){
  opt_var_manager.get_opt_vars_bounds(x_low, x_upp);
}   	  	
void Jump_Opt::get_current_opt_vars(std::vector<double> &x_vars_out){
  opt_var_manager.get_current_opt_vars(x_vars_out);
}
void Jump_Opt::update_opt_vars(std::vector<double> &x_vars){
  opt_var_manager.update_opt_vars(x_vars);
} 	  		



void Jump_Opt::get_F_bounds(std::vector<double> &F_low, std::vector<double> &F_upp){
  F_low.clear();
  F_upp.clear();  
  // Initialize Bounds for Time Dependent Constraints
  for(int knotpoint = 1; knotpoint < N_total_knotpoints + 1; knotpoint++){

    for(size_t i = 0; i < td_constraint_list.get_size(); i++){
      for(size_t j = 0; j < td_constraint_list.get_constraint(i)->F_low.size(); j++ ){
        F_low.push_back(td_constraint_list.get_constraint(i)->F_low[j]);
      }
      for(size_t j = 0; j < td_constraint_list.get_constraint(i)->F_upp.size(); j++ ){
        F_upp.push_back(td_constraint_list.get_constraint(i)->F_upp[j]);
      }
    }

  }

  // Initialize Bounds for Time Independent Constraints
  for(size_t i = 0; i < ti_constraint_list.get_size(); i++){
    for(size_t j = 0; j < ti_constraint_list.get_constraint(i)->F_low.size(); j++ ){
      F_low.push_back(ti_constraint_list.get_constraint(i)->F_low[j]);
    }
    for(size_t j = 0; j < ti_constraint_list.get_constraint(i)->F_upp.size(); j++ ){
      F_upp.push_back(ti_constraint_list.get_constraint(i)->F_upp[j]);
    }
  }


  // Initialize Bounds for the Objective Function
  F_low.push_back(objective_function.F_low);
  F_upp.push_back(objective_function.F_upp);  	
}

void Jump_Opt::get_F_obj_Row(int &obj_row){
  obj_row = objective_function.objective_function_index;
  std::cout << "[Jump Opt] Objective Row = " << obj_row << std::endl;
}

void Jump_Opt::compute_F_objective_function(double &result_out){
  objective_function.evaluate_objective_function(opt_var_manager, result_out);
  //std::cout << "[Jump_Opt] cost = " << result_out << std::endl;
}

void Jump_Opt::compute_F(std::vector<double> &F_eval){
  compute_F_constraints(F_eval);
  double cost = 0.0;

  compute_F_objective_function(cost);
  F_eval.push_back(cost);
}


void Jump_Opt::compute_F_constraints(std::vector<double> &F_eval){
  std::vector<double> F_vec_const;
  //std::cout << "[Jump_OPT] Computing F Constraints" << std::endl;

  // Compute Timestep Dependent Constraints
  for(int knotpoint = 1; knotpoint < N_total_knotpoints + 1; knotpoint++){
    for(int i = 0; i < td_constraint_list.get_size(); i++){
      F_vec_const.clear();
      td_constraint_list.get_constraint(i)->evaluate_constraint(knotpoint, opt_var_manager, F_vec_const);
      //std::cout << " Adding TD Constraint " << td_constraint_list.get_constraint(i)->constraint_name << std::endl;

      for(int j = 0; j < F_vec_const.size(); j++){
        //std::cout << "F_td_const[" << j <<"] = " << F_vec_const[j] << std::endl;
        // Add to F_eval
        F_eval.push_back(F_vec_const[j]);
      }
    }
  }

  // Compute Timestep Independent Constraints
  // Code here
  Constraint_Function* current_constraint;
  for(size_t i = 0; i < ti_constraint_list.get_size(); i++){
      F_vec_const.clear();
      current_constraint = ti_constraint_list.get_constraint(i);         
      current_constraint->evaluate_constraint(current_constraint->des_knotpoint, opt_var_manager, F_vec_const);
      //std::cout << " Adding TI Constraint " << current_constraint->constraint_name << std::endl;

      //std::cout << "F_vec_const.size() = " << F_vec_const.size() << std::endl;
    for(size_t j = 0; j < current_constraint->F_low.size(); j++ ){
      //std::cout << " des knotpoint = " << current_constraint->des_knotpoint << std::endl;
      //std::cout << "F_ti_const[" << j <<"] = " << F_vec_const[j] << std::endl;      
      F_eval.push_back(current_constraint->F_low[j]);
    }

  }

  // Debug statement  
  /*  for(int j = 0; j < F_eval.size(); j++){
      std::cout << "F_eval[" << j << "] = " << F_eval[j] << std::endl;
    }*/

}



void Jump_Opt::compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG){}
void Jump_Opt::compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA){}


