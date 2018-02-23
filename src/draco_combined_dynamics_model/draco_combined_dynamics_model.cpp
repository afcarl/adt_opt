#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>
#include <Utils/utilities.hpp>
#include "DracoP1Rot_Definition.h"


Draco_Combined_Dynamics_Model* Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel(){
    static Draco_Combined_Dynamics_Model draco_combined_dynamics_model_;
    return & draco_combined_dynamics_model_;
}

Draco_Combined_Dynamics_Model::~Draco_Combined_Dynamics_Model(){
	robot_model = DracoModel::GetDracoModel();
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();	
}

Draco_Combined_Dynamics_Model::Draco_Combined_Dynamics_Model(){
	A_mat.resize(NUM_QDOT, NUM_QDOT); A_mat.setZero();
	
	A_bb.resize(NUM_VIRTUAL, NUM_VIRTUAL); A_bb.setZero();
	A_br.resize(NUM_VIRTUAL, NUM_JOINT); A_br.setZero();
	A_brT.resize(NUM_ACT_JOINT, NUM_VIRTUAL); A_brT.setZero();
	A_rr.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); A_rr.setZero();

	grav.resize(NUM_QDOT);	grav.setZero();
	coriolis.resize(NUM_QDOT); coriolis.setZero();

	L.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); L.setZero();
	J.resize(NUM_ACT_JOINT, NUM_ACT_JOINT);	J.setZero();

	Sv.resize(NUM_VIRTUAL, NUM_QDOT); Sv.setZero();
	Sa.resize(NUM_ACT_JOINT, NUM_QDOT);	Sa.setZero();

	M_combined.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_ACT_JOINT + NUM_ACT_JOINT); M_combined.setZero();
	B_combined.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); B_combined.setZero();
	K_combined.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); K_combined.setZero();

	x_state.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); x_state.setZero();
	xdot_state.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); xdot_state.setZero();

	q_state.resize(NUM_QDOT); q_state.setZero();
	qdot_state.resize(NUM_QDOT); qdot_state.setZero();


}

void Draco_Combined_Dynamics_Model::UpdateModel(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in){
	x_state = x_state_in;
	xdot_state = xdot_state_in;
	convert_x_xdot_to_q_qdot(x_state, xdot_state, q_state, qdot_state);

	// Update the Robot's Model
	robot_model->UpdateModel(q_state, qdot_state);

	robot_model->getMassInertia(A_mat);
    robot_model->getGravity(grav);
    robot_model->getCoriolis(coriolis);

	compute_mass_matrix();
	compute_damping_matrix();
	compute_stiffness_matrix();
	compute_joint_link_impedance();	
}


void Draco_Combined_Dynamics_Model::get_combined_mass_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &M_out){}
void Draco_Combined_Dynamics_Model::get_combined_damping_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &B_out){}
void Draco_Combined_Dynamics_Model::get_combined_stiffness_matrix(const sejong::Vector &x_state, sejong::Matrix &K_out){}
void Draco_Combined_Dynamics_Model::get_virtual_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sv_out){}
void Draco_Combined_Dynamics_Model::get_actuated_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sa_out){}			

void Draco_Combined_Dynamics_Model::compute_mass_matrix(){
}
void Draco_Combined_Dynamics_Model::compute_damping_matrix(){
}	
void Draco_Combined_Dynamics_Model::compute_stiffness_matrix(){
}
void Draco_Combined_Dynamics_Model::compute_joint_link_impedance(){
}


void Draco_Combined_Dynamics_Model::convert_x_xdot_to_q_qdot(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Vector q_state_out, sejong::Vector qdot_state_out){
	sejong::Vector q_virt_state = x_state.head(NUM_VIRTUAL);
	sejong::Vector z_state = x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
	//sejong::Vector delta_state = x_state.tail(NUM_ACT_JOINT);

	sejong::Vector qdot_virt_state = xdot_state.head(NUM_VIRTUAL);
	sejong::Vector zdot_state = xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
	//sejong::Vector delta_dot_state = xdot_state.tail(NUM_ACT_JOINT);	

	sejong::Vector q_act;
	sejong::Vector qdot_act;	

	actuator_model->getFull_joint_pos_q(z_state, q_act);
	actuator_model->getFull_joint_vel_qdot(zdot_state, zdot_state, qdot_act);

	q_state_out.resize(NUM_VIRTUAL);
	qdot_state_out.resize(NUM_ACT_JOINT);

	q_state_out.head(NUM_VIRTUAL) = q_virt_state;
	q_state_out.tail(NUM_ACT_JOINT) = q_act;

	qdot_state_out.head(NUM_VIRTUAL) = qdot_virt_state;	
	qdot_state_out.tail(NUM_ACT_JOINT) = qdot_act;	
}
