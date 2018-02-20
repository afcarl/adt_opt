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
    // Mass Elements (Kg)
    M_motor.resize(NUM_ACTUATORS); 	M_motor.setZero();
    M_spring.resize(NUM_ACTUATORS); M_spring.setZero();
    M_load.resize(NUM_ACTUATORS);  	M_load.setZero();

    // Damping Elements (N/m-s)
    B_motor.resize(NUM_ACTUATORS); B_motor.setZero();
    B_spring.resize(NUM_ACTUATORS); B_spring.setZero();
    B_load.resize(NUM_ACTUATORS); B_load.setZero();

    // Spring Elements (N/m)
    K_motor.resize(NUM_ACTUATORS); K_motor.setZero();
    K_spring.resize(NUM_ACTUATORS); K_spring.setZero();

	// Torque Constant (N-m/Amperes)	        
    K_m.resize(NUM_ACTUATORS); K_m.setZero();

	// Actuator Moment Arm (m)
    r_arm.resize(NUM_ACTUATORS); r_arm.setZero();
	// Zero spring force actuator position
    z_o.resize(NUM_ACTUATORS); z_o.setZero();		

	// Zero spring force joint position
    q_o.resize(NUM_ACTUATORS); q_o.setZero();		    

    // Set Default Values
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
	    // Mass Elements (Kg)
	    M_motor[i] = 3;
	    M_spring[i] = 1;
	    M_load[i] = 1;

	    // Damping Elements (N/m-s)
	    B_motor[i] = 6;
	    B_spring[i] = 1;
	    B_load[i] = 2;

	    // Spring Elements (N/m)
	    K_motor[i] = 9;
	    K_spring[i] = 3;
		// Torque Constant (N-m/Amperes)	    
		K_m[i] = 0.01;

		// Actuator Moment Arm (m)
		r_arm[i] = 0.05; 		
		// Zero spring force actuator position
		z_o[i] = 0.0;
		q_o[i] = 0.0;
	}

	q_o[0] = -1.0;
	q_o[1] = 2.0;
	q_o[2] = -1.0;

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
}


// Set the zero spring force joint position
void DracoActuatorModel::set_zero_pos_q_o(sejong::Vector &q_o_in){
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		q_o[i] = q_o_in[i];
	}	
}

// Simple Relationship between actuator position and joint position
// (z - z_o) = r*(q - q_o) 
double DracoActuatorModel::get_joint_pos_q(const int &index, const double &z_act_pos){
	return (z_act_pos - z_o[index])/r_arm[index] + q_o[index];
}
double DracoActuatorModel::get_act_pos_z(const int &index, const double &q_act_pos){
	return z_o[index] + r_arm[index]*(q_act_pos + q_o[index]);
}

// dz/dq = r 
double DracoActuatorModel::getJacobian_dzdq(const int &index, const double &z_act_pos){
	return r_arm[index];
}

void DracoActuatorModel::getFullJacobian(sejong::Matrix &L){
	L.resize(NUM_ACTUATORS, NUM_ACTUATORS);
	L.setIdentity();

	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		L(i,i) = r_arm[i];
	}

}

void DracoActuatorModel::getFull_joint_pos_q(const sejong::Vector &z_in, sejong::Vector &q_out){
	q_out.resize(NUM_ACTUATORS);
	q_out.setZero();

	double z_index_val = 0.0;
	for(int i = 0; i < NUM_ACTUATORS; i++){
		z_index_val = z_in[i];
		q_out[i] = get_joint_pos_q(i, z_index_val);
	}	
}

