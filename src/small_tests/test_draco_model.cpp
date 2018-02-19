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
	actuator_model->getMassMatrix(M_act);
	actuator_model->getDampingMatrix(B_act);	
	actuator_model->getStiffnessMatrix(K_act);


	return 0;
}