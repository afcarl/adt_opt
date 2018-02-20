#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>

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

	return 0;
}