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
	Initialization();
	printf("[Draco Actuator Model] Constructed\n");
}

void DracoActuatorModel::Initialization(){
    // Mass Elements
    M_motor = 3;
    M_spring = 1;
    M_load = 1;

    // Damping Elements
    B_motor = 6;
    B_spring = 1;
    B_load = 2;

    // Spring Elements
    K_motor = 9;
    K_spring = 3;

    K_m = 0.01;    
}

void DracoActuatorModel::getMassMatrix(sejong::Matrix &M_act){
	M_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	M_act.setZero();
	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		M_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = M_motor;
		M_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = M_motor - M_spring;		
		M_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i) = M_load;		
		M_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = M_spring;				
	}
	sejong::pretty_print(M_act, std::cout, "M_act");	
}

void DracoActuatorModel::getDampingMatrix(sejong::Matrix &B_act){
	B_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	B_act.setZero();
	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		B_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = B_motor;
		B_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = B_motor - B_spring;		
		B_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i) = B_load;		
		B_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = B_spring;				
	}
	sejong::pretty_print(B_act, std::cout, "B_act");	
}

void DracoActuatorModel::getStiffnessMatrix(sejong::Matrix &K_act){
	K_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	K_act.setZero();
	// Assign block diagonally	
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		K_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = K_motor;
		K_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = K_motor - K_spring;		
		K_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = K_spring;				
	}
	sejong::pretty_print(K_act, std::cout, "K_act");		
}