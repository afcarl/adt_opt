#include  "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"

int main(int argc, char **argv){
	std::cout << "[Main]Testing Draco Model Object" << std::endl;
	DracoModel* robot_model;
	robot_model = DracoModel::GetDracoModel();
	return 0;
}