#include <adt/hard_constraints/adt_com_kinematic_constraint.hpp>
#include <Utils/utilities.hpp>
#include "DracoP1Rot_Definition.h"

CoM_2D_Kinematic_Constraint::CoM_2D_Kinematic_Constraint(int knotpoint_in, int dim_in, double l_bound_in, double u_bound_in){
	des_knotpoint = knotpoint_in;
	dim = dim_in;
	l_bound = l_bound_in;
	u_bound = u_bound_in;
	Initialization();
}

CoM_2D_Kinematic_Constraint::~CoM_2D_Kinematic_Constraint(){
	std::cout << "[CoM_2D_Kinematic_Constraint] Destructor called" << std::endl;
}

void CoM_2D_Kinematic_Constraint::Initialization(){
	constraint_name = "COM 2D Constraint dim " + std::to_string(dim) + " kp " + std::to_string(des_knotpoint);	

	q_state.resize(NUM_QDOT);
  	x_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);
	xdot_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);  

	q_state.setZero();
	x_state.setZero();
	xdot_state.setZero();		

	robot_model = DracoModel::GetDracoModel();	
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();
	combined_model = Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel();	
	initialize_Flow_Fupp();	

}


void CoM_2D_Kinematic_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

	// Single equality Constraint
	// We want the com position, f(q(z(k)))  to be at a particular position at timestep k 
	// f(q(z(k)) = des_val
	F_low.push_back(l_bound);	
	F_upp.push_back(u_bound);

	constraint_size = F_low.size();
}

void CoM_2D_Kinematic_Constraint::update_states(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager){
	sejong::Vector q_state_virt;
	sejong::Vector z_state;
	sejong::Vector delta_state;

	sejong::Vector qdot_state_virt;  
	sejong::Vector zdot_state;
	sejong::Vector delta_dot_state;

	var_manager.get_q_states(knotpoint, q_state_virt);
	var_manager.get_z_states(knotpoint, z_state);
	var_manager.get_delta_states(knotpoint, delta_state);
	var_manager.get_qdot_states(knotpoint, qdot_state_virt);
	var_manager.get_zdot_states(knotpoint, zdot_state);  
	var_manager.get_delta_dot_states(knotpoint, delta_dot_state);

	x_state.head(NUM_VIRTUAL) = q_state_virt;
	x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = z_state;
	x_state.tail(NUM_ACT_JOINT) = delta_state;

	xdot_state.head(NUM_VIRTUAL) = qdot_state_virt;
	xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = zdot_state;
	xdot_state.tail(NUM_ACT_JOINT) = delta_dot_state;	  
}


void CoM_2D_Kinematic_Constraint::evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
	F_vec.clear();
	update_states(knotpoint, var_manager);
	
	combined_model->UpdateModel(x_state, xdot_state);
	combined_model->convert_x_to_q(x_state, q_state);
	robot_model->getCoMPosition(q_state, pos);

	F_vec.push_back(pos[dim]);
	// // Get robot virtual states and actuator z position states---------------------------------------------------------------------------
	// sejong::Vector q_state_virt;
	// sejong::Vector qdot_state_virt;
	// sejong::Vector z_state;  
	// sejong::Vector zdot_state;    
	// var_manager.get_q_states(knotpoint, q_state_virt);
	// var_manager.get_qdot_states(knotpoint, qdot_state_virt);
	// var_manager.get_z_states(knotpoint, z_state);
	// var_manager.get_zdot_states(knotpoint, zdot_state);  

	// // convert actuator z_states to joint configuration q_state_act
	// sejong::Vector q_state_act;
	// sejong::Vector qdot_state_act;  
	// actuator_model->getFull_joint_pos_q(z_state, q_state_act);
	// actuator_model->getFull_joint_vel_qdot(z_state, zdot_state, qdot_state_act);

	// // Repopulate q_state and qdot_state
	// sejong::Vector q_state; q_state.resize(NUM_QDOT);
	// sejong::Vector qdot_state; qdot_state.resize(NUM_QDOT);

	// q_state.head(NUM_VIRTUAL) = q_state_virt;
	// q_state.tail(NUM_ACT_JOINT) = q_state_act;  
	// qdot_state.head(NUM_VIRTUAL) = qdot_state_virt;
	// qdot_state.head(NUM_ACT_JOINT) = qdot_state_act;   

	// // Update the robot model
	// robot_model->UpdateModel(q_state, qdot_state);  

	// sejong::Vect3 pos_vec;
	// robot_model->getPosition(q_state, link_id, pos_vec);
	//F_vec.push_back(pos_vec[dim]);
}
void CoM_2D_Kinematic_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void CoM_2D_Kinematic_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

