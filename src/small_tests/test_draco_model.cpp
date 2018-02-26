#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>
#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>

#include <adt/contacts/adt_draco_contact_heel.hpp>
#include <adt/contacts/adt_draco_contact_toe.hpp>

#include <Utils/utilities.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

int main(int argc, char **argv){
	std::cout << "[Main]Testing Draco Model Object" << std::endl;
	DracoModel* robot_model;
	robot_model = DracoModel::GetDracoModel();

	DracoActuatorModel* actuator_model;
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();

	sejong::Matrix M_act;
	sejong::Matrix B_act;
	sejong::Matrix K_act;	
	sejong::Matrix L_act;
	actuator_model->getMassMatrix(M_act);
	actuator_model->getDampingMatrix(B_act);	
	actuator_model->getStiffnessMatrix(K_act);
	sejong::Vector z_act; z_act.resize(NUM_ACT_JOINT); z_act.setZero();
	actuator_model->getFullJacobian_dzdq(z_act, L_act);

	sejong::pretty_print(M_act, std::cout, "M_act");		
	sejong::pretty_print(B_act, std::cout, "B_act");		
	sejong::pretty_print(K_act, std::cout, "K_act");		
	sejong::pretty_print(L_act, std::cout, "L_act");		

	sejong::Vector q_o_init = actuator_model->q_o;
	sejong::pretty_print(q_o_init, std::cout, "q_o_init");		
	// Test Modification of q_init
	//q_o_init.setZero();
	//actuator_model->set_zero_pos_q_o(q_o_init);
	//sejong::pretty_print(q_o_init, std::cout, "New q_o_init");

	sejong::Matrix Km_matrix;

	actuator_model->getKm_Matrix(Km_matrix);
	sejong::pretty_print(Km_matrix, std::cout, "Km_matrix");

	// Test Obtaining the Jacobians
	sejong::Vector q_state;
	q_state.resize(NUM_QDOT);
	q_state.setZero();
	q_state[0] = 0.01;
	q_state[1] = 0.87 - 0.19;
	q_state[SJJointID::bodyPitch] = -1.0;
	q_state[SJJointID::kneePitch] = 2.0;
	q_state[SJJointID::anklePitch] = -1.0;

	q_state[1] = 0.831165;
	q_state[SJJointID::bodyPitch] = -M_PI/4.0;
	q_state[SJJointID::kneePitch] = M_PI/2.0;
	q_state[SJJointID::anklePitch] = -M_PI/4.0;	

	sejong::Vector qdot_state;
	qdot_state.resize(NUM_QDOT);
	qdot_state.setZero();
	sejong::pretty_print(q_state, std::cout, "q_state");

	std::cout << "[Main] Updating Robot Model" << std::endl;
	robot_model->UpdateModel(q_state, qdot_state);
	std::cout << "[Main] Update finished" << std::endl;

	sejong::Matrix Jt;
	int link_id =  SJLinkID::LK_FootToe;


	robot_model->getFullJacobian(q_state, link_id, Jt);
	sejong::pretty_print(Jt, std::cout, "Jt_toe");

	Draco_Toe_Contact dt_contact;
	sejong::Matrix Jt_toe_reduced;
	dt_contact.getContactJacobian(q_state, Jt_toe_reduced);
	sejong::pretty_print(Jt_toe_reduced, std::cout, "Jt_toe_reduced");	


	link_id =  SJLinkID::LK_FootHeel;
	robot_model->getFullJacobian(q_state, link_id, Jt);
	sejong::pretty_print(Jt, std::cout, "Jt_heel");

	Draco_Heel_Contact dh_contact;
	sejong::Matrix Jt_heel_reduced;
	dh_contact.getContactJacobian(q_state, Jt_heel_reduced);
	sejong::pretty_print(Jt_heel_reduced, std::cout, "Jt_heel_reduced");	


	sejong::Vect3 foot_vec_pos;
	robot_model->getPosition(q_state, SJLinkID::LK_foot, foot_vec_pos);
	sejong::pretty_print(foot_vec_pos, std::cout, "Foot_vec_pos");

	sejong::Vect3 toe_vec_pos;
	robot_model->getPosition(q_state, SJLinkID::LK_FootToe, toe_vec_pos);
	sejong::pretty_print(toe_vec_pos, std::cout, "Toe_vec_pos");

	sejong::Vect3 heel_vec_pos;
	robot_model->getPosition(q_state, SJLinkID::LK_FootHeel, heel_vec_pos);
	sejong::pretty_print(heel_vec_pos, std::cout, "heel_vec_pos");	


	Draco_Combined_Dynamics_Model* draco_combined_model = Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel();


	// 
	sejong::Vector q_virt = q_state.head(NUM_VIRTUAL);
	sejong::Vector z_state;
	sejong::Vector delta_state; delta_state.resize(NUM_ACT_JOINT); delta_state.setZero();	

	sejong::Vector qdot_virt = qdot_state.head(NUM_VIRTUAL);
	sejong::Vector zdot_state; zdot_state.resize(NUM_ACT_JOINT); zdot_state.setZero();
	sejong::Vector delta_dot_state;	delta_dot_state.resize(NUM_ACT_JOINT); delta_dot_state.setZero();	

    actuator_model->getFull_act_pos_z(q_state.tail(NUM_ACT_JOINT), z_state);
    sejong::pretty_print(z_state, std::cout, "Hello z_state");

    sejong::Vector x_state; x_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);
    sejong::Vector xdot_state; xdot_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);


    x_state.head(NUM_VIRTUAL) = q_virt;
    x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = z_state;
	x_state.tail(NUM_ACT_JOINT) = delta_state;

	xdot_state.head(NUM_VIRTUAL) = qdot_virt;
    xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = zdot_state;
	xdot_state.tail(NUM_ACT_JOINT) = delta_dot_state;	    

/*	sejong::pretty_print(q_state, std::cout, "q_state");
	sejong::pretty_print(qdot_state, std::cout, "qdot_state");	


	sejong::pretty_print(x_state, std::cout, "x_state");
	sejong::pretty_print(xdot_state, std::cout, "xdot_state");		

	*/
	draco_combined_model->UpdateModel(x_state, xdot_state);


	// Stack Contact Jacobian:
	sejong::Matrix Jc = Jt_toe_reduced;
	int prestack_row_size = Jc.rows();
	Jc.conservativeResize(prestack_row_size + Jt_heel_reduced.rows(), NUM_QDOT);
	Jc.block(prestack_row_size, 0, Jt_heel_reduced.rows(), NUM_QDOT) = Jt_heel_reduced;
	sejong::pretty_print(Jc, std::cout, "Contact Jacobian");

	// Assign Contact Jacobian
	draco_combined_model->setContactJacobian(Jc);

	sejong::Vector u_in; u_in.resize(NUM_ACT_JOINT); u_in.setZero();
	sejong::Vector Fr_in; Fr_in.resize(Jc.rows());	Fr_in.setZero();

	u_in[0] = 1;

	sejong::Vector xddot_out; xddot_out.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);
	draco_combined_model->get_state_acceleration(x_state, xdot_state, u_in, Fr_in, xddot_out);
	sejong::pretty_print(xddot_out, std::cout, "xddot_out");



	return 0;
}