#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>
#include <Utils/utilities.hpp>
#include "DracoP1Rot_Definition.h"


Draco_Combined_Dynamics_Model* Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel(){
    static Draco_Combined_Dynamics_Model draco_combined_dynamics_model_;
    return & draco_combined_dynamics_model_;
}

Draco_Combined_Dynamics_Model::~Draco_Combined_Dynamics_Model(){
	std::cout << "[Draco_Combined_Dynamics_Model] Destroyed" << std::endl;
}

Draco_Combined_Dynamics_Model::Draco_Combined_Dynamics_Model(){
	Initialization();
	std::cout << "[Draco_Combined_Dynamics_Model] Constructed" << std::endl;
}



void Draco_Combined_Dynamics_Model::Initialization(){
	robot_model = DracoModel::GetDracoModel();
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();	

	// Initialize Inertia Matrix Size
	A_mat.resize(NUM_QDOT, NUM_QDOT); A_mat.setZero();
	
	A_bb.resize(NUM_VIRTUAL, NUM_VIRTUAL); A_bb.setZero();
	A_br.resize(NUM_VIRTUAL, NUM_ACT_JOINT); A_br.setZero();
	A_brT.resize(NUM_ACT_JOINT, NUM_VIRTUAL); A_brT.setZero();
	A_rr.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); A_rr.setZero();

	// Initialize Gravity and Coriolis Vectors
	grav.resize(NUM_QDOT);	grav.setZero();
	coriolis.resize(NUM_QDOT); coriolis.setZero();

	// Initialize Actuator to Joint Dynamics Jacobians
	L.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); L.setZero();
	J.resize(NUM_ACT_JOINT, NUM_ACT_JOINT);	J.setZero();

	// Initialize Selection Matrices
	Sv.resize(NUM_VIRTUAL, NUM_QDOT); Sv.setZero();
	Sa.resize(NUM_ACT_JOINT, NUM_QDOT);	Sa.setZero();

	Sv.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL); 
	Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT); 	


	// ---------------------------------------------------------------
	M_act.resize(NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR, NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR); M_act.setZero();
	initialize_actuator_matrices(M_zz);
	initialize_actuator_matrices(M_z_delta);
	initialize_actuator_matrices(M_delta_z);
	initialize_actuator_matrices(M_delta_delta);		

	B_act.resize(NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR, NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR); B_act.setZero();
	initialize_actuator_matrices(B_zz);
	initialize_actuator_matrices(B_z_delta);
	initialize_actuator_matrices(B_delta_z);
	initialize_actuator_matrices(B_delta_delta);		

	K_act.resize(NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR, NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR); K_act.setZero();
	initialize_actuator_matrices(K_zz);
	initialize_actuator_matrices(K_z_delta);
	initialize_actuator_matrices(K_delta_z);
	initialize_actuator_matrices(K_delta_delta);			
	// ---------------------------------------------------------------


	// Initialize Combined Matrices
	M_combined.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_ACT_JOINT + NUM_ACT_JOINT); M_combined.setZero();
	B_combined.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); B_combined.setZero();
	K_combined.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); K_combined.setZero();

	// Initialize x, xdot vectors
	x_state.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); x_state.setZero();
	xdot_state.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); xdot_state.setZero();

	// Initialize z, zdot vectors
	z_state.resize(NUM_ACT_JOINT); z_state.setZero();
	zdot_state.resize(NUM_ACT_JOINT); zdot_state.setZero();	

	// Initialize q_virt, qdot_virt vectors
	q_virt_state.resize(NUM_VIRTUAL); q_virt_state.setZero();
	qdot_virt_state.resize(NUM_VIRTUAL); qdot_virt_state.setZero();	
	q_act.resize(NUM_ACT_JOINT); q_act.setZero();
	qdot_act.resize(NUM_ACT_JOINT); qdot_act.setZero();

/*	delta_state.resize(NUM_ACT_JOINT); delta_state.setZero();
	delta_dot_state.resize(NUM_ACT_JOINT); delta_dot_state.setZero();*/

	// Initialize q, qdot vectors
	q_state.resize(NUM_QDOT); q_state.setZero();
	qdot_state.resize(NUM_QDOT); qdot_state.setZero();
}

void Draco_Combined_Dynamics_Model::initialize_actuator_matrices(sejong::Matrix &Mat){
	Mat.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); Mat.setZero();
}

void Draco_Combined_Dynamics_Model::UpdateModel(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in){
	x_state = x_state_in;
	xdot_state = xdot_state_in;

	// Convert x to q
	convert_x_xdot_to_q_qdot(x_state, xdot_state, q_state, qdot_state);

	// Update the Robot's Model
	robot_model->UpdateModel(q_state, qdot_state);
	robot_model->getMassInertia(A_mat);
    robot_model->getGravity(grav);
    robot_model->getCoriolis(coriolis);

	actuator_model->getFullJacobian_dzdq(z_state, L);
	actuator_model->getFullJacobian_dqdz(q_state.tail(NUM_ACT_JOINT), J);	   

	actuator_model->getMassMatrix(M_act);
	actuator_model->getDampingMatrix(B_act);	   
	actuator_model->getStiffnessMatrix(K_act);	   

	sejong::pretty_print(L, std::cout, "L");
	sejong::pretty_print(J, std::cout, "J");

	sejong::pretty_print(M_act, std::cout, "M_act");
	sejong::pretty_print(B_act, std::cout, "B_act");
	sejong::pretty_print(K_act, std::cout, "K_act");

	formulate_mass_matrix();
	formulate_damping_matrix();
	formulate_stiffness_matrix();
	formulate_joint_link_impedance();	
}


void Draco_Combined_Dynamics_Model::get_combined_mass_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &M_out){}
void Draco_Combined_Dynamics_Model::get_combined_damping_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &B_out){}
void Draco_Combined_Dynamics_Model::get_combined_stiffness_matrix(const sejong::Vector &x_state, sejong::Matrix &K_out){}
void Draco_Combined_Dynamics_Model::get_virtual_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sv_out){}
void Draco_Combined_Dynamics_Model::get_actuated_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sa_out){}			

void Draco_Combined_Dynamics_Model::formulate_mass_matrix(){
	A_bb = A_mat.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL);
	A_br = A_mat.block(0, NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT);
	A_brT = A_mat.block(NUM_VIRTUAL, 0, NUM_ACT_JOINT, NUM_VIRTUAL);
	A_rr = A_mat.block(NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT);

	M_zz = M_act.block(0, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	M_z_delta = M_act.block(0, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);
	M_delta_z = M_act.block(NUM_ACT_JOINT, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	M_delta_delta = M_act.block(NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);	

/*	sejong::pretty_print(A_mat, std::cout, "A_mat");
	sejong::pretty_print(A_bb, std::cout, "A_bb");
	sejong::pretty_print(A_br, std::cout, "A_br");
	sejong::pretty_print(A_brT, std::cout, "A_brT");
	sejong::pretty_print(A_rr, std::cout, "A_rr");	*/

/*	sejong::pretty_print(M_act, std::cout, "M_act");
	sejong::pretty_print(M_zz, std::cout, "M_zz");
	sejong::pretty_print(M_z_delta, std::cout, "M_z_delta");
	sejong::pretty_print(M_delta_z, std::cout, "M_delta_z");
	sejong::pretty_print(M_delta_delta, std::cout, "M_delta_delta");	*/


}
void Draco_Combined_Dynamics_Model::formulate_damping_matrix(){}	
void Draco_Combined_Dynamics_Model::formulate_stiffness_matrix(){}
void Draco_Combined_Dynamics_Model::formulate_joint_link_impedance(){}

void Draco_Combined_Dynamics_Model::convert_x_xdot_to_q_qdot(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Vector q_state_out, sejong::Vector qdot_state_out){
	// extract q_virt and actuator z_states from x_states
	q_virt_state = x_state.head(NUM_VIRTUAL);
	z_state = x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

	qdot_virt_state = xdot_state.head(NUM_VIRTUAL);
	zdot_state = xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

	// Convert z, zdot to q, qdot
	actuator_model->getFull_joint_pos_q(z_state, q_act);
	actuator_model->getFull_joint_vel_qdot(zdot_state, zdot_state, qdot_act);

	q_state_out.resize(NUM_QDOT);
	qdot_state_out.resize(NUM_QDOT);

	q_state_out.head(NUM_VIRTUAL) = q_virt_state;
	q_state_out.tail(NUM_ACT_JOINT) = q_act;

	qdot_state_out.head(NUM_VIRTUAL) = qdot_virt_state;	
	qdot_state_out.tail(NUM_ACT_JOINT) = qdot_act;	

/*	sejong::pretty_print(q_state_out, std::cout, "q_state_out");
	sejong::pretty_print(qdot_state_out, std::cout, "qdot_state_out");	*/
}
