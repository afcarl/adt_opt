#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>

#include <adt/contacts/adt_draco_contact_heel.hpp>
#include <adt/contacts/adt_draco_contact_toe.hpp>

#include <Utils/utilities.hpp>

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
	actuator_model->getFullJacobian(L_act);

	sejong::pretty_print(M_act, std::cout, "M_act");		
	sejong::pretty_print(B_act, std::cout, "B_act");		
	sejong::pretty_print(K_act, std::cout, "K_act");		
	sejong::pretty_print(L_act, std::cout, "L_act");		

	sejong::Vector q_o_init = actuator_model->q_o;
	sejong::pretty_print(q_o_init, std::cout, "q_o_init");		
	// Test Modification of q_init
	q_o_init.setZero();
	actuator_model->set_zero_pos_q_o(q_o_init);
	sejong::pretty_print(q_o_init, std::cout, "New q_o_init");


	// Test Obtaining the Jacobians
	sejong::Vector q_state;
	q_state.resize(NUM_QDOT);
	q_state.setZero();
	q_state[0] = 0.01;
	q_state[1] = 0.87 - 0.19;
	q_state[SJJointID::bodyPitch] = -1.0;
	q_state[SJJointID::kneePitch] = 2.0;
	q_state[SJJointID::anklePitch] = -1.0;

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


	return 0;
}