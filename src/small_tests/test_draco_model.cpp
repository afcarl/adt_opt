#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>

int main(int argc, char **argv){
	std::cout << "[Main]Testing Draco Model Object" << std::endl;
	DracoModel* robot_model;
	robot_model = DracoModel::GetDracoModel();

	DracoActuatorModel* actuator_model;
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();

	return 0;
}