#include <draco_actuator_model/DracoActuatorModel.hpp>
#include <Utils/utilities.hpp>

DracoActuatorModel* DracoActuatorModel::GetDracoActuatorModel(){
    static DracoActuatorModel draco_act_model_;
    return & draco_act_model_;
}

// Singleton Destructor
DracoActuatorModel::~DracoActuatorModel(){
}

// Singleton Constructor
DracoActuatorModel::DracoActuatorModel(){
	printf("[Draco Actuator Model] Constructed\n");
}