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
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
	    // Mass Elements (Kg)
	    M_motor.push_back(3);
	    M_spring.push_back(1);
	    M_load.push_back(1);

	    // Damping Elements (N/m-s)
	    B_motor.push_back(6);
	    B_spring.push_back(1);
	    B_load.push_back(2);

	    // Spring Elements (N/m)
	    K_motor.push_back(9);
	    K_spring.push_back(3);
		// Torque Constant (N-m/Amperes)	    
		K_m.push_back(0.01);

		// Actuator Moment Arm (m)
		r_arm.push_back(0.05); 		
		// Zero spring force actuator position
		z_o.push_back(0.00);
	}

}

void DracoActuatorModel::getMassMatrix(sejong::Matrix &M_act){
	M_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	M_act.setZero();
	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		M_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = M_motor[i];
		M_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = M_motor[i] - M_spring[i];		
		M_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i) = M_load[i];		
		M_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = M_spring[i];				
	}
	sejong::pretty_print(M_act, std::cout, "M_act");	
}

void DracoActuatorModel::getDampingMatrix(sejong::Matrix &B_act){
	B_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	B_act.setZero();
	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		B_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = B_motor[i];
		B_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = B_motor[i] - B_spring[i];		
		B_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i) = B_load[i];		
		B_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = B_spring[i];				
	}
	sejong::pretty_print(B_act, std::cout, "B_act");	
}

void DracoActuatorModel::getStiffnessMatrix(sejong::Matrix &K_act){
	K_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	K_act.setZero();
	// Assign block diagonally	
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		K_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = K_motor[i];
		K_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = K_motor[i] - K_spring[i];		
		K_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = K_spring[i];				
	}
	sejong::pretty_print(K_act, std::cout, "K_act");		
}


// Simple Relationship between actuator position and joint position
// (z - z_o) = r*q 
double DracoActuatorModel::get_joint_pos_q(int &index, double &z_act_pos){
	return (z_act_pos - z_o[index])/r_arm[index];
}
double DracoActuatorModel::get_act_pos_z(int &index, double &q_act_pos){
	return z_o[index] + r_arm[index]*q_act_pos;
}

// dz/dq = r 
double DracoActuatorModel::getJacobian_dzdq(int &index, double &z_act_pos){
	return r_arm[index];
}
