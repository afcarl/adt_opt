#include <adt/optimization_variable_limits/adt_jump_variable_limits.hpp>

Jump_Opt_Variable_Limits::Jump_Opt_Variable_Limits(){
	Initialization();
}

void Jump_Opt_Variable_Limits::Initialization(){
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();	
	initialize_to_zero();
	set_limits();
}

void Jump_Opt_Variable_Limits::initialize_to_zero(){
	l_q_virt_limits.resize(NUM_VIRTUAL); l_q_virt_limits.setZero();
	u_q_virt_limits.resize(NUM_VIRTUAL); u_q_virt_limits.setZero();

	l_z_limits.resize(NUM_ACT_JOINT); l_z_limits.setZero();
	u_z_limits.resize(NUM_ACT_JOINT); u_z_limits.setZero();	

	l_qdot_virt_limits.resize(NUM_VIRTUAL); l_qdot_virt_limits.setZero();
	u_qdot_virt_limits.resize(NUM_VIRTUAL); u_qdot_virt_limits.setZero();		

	l_zdot_limits.resize(NUM_ACT_JOINT); l_zdot_limits.setZero();
	u_zdot_limits.resize(NUM_ACT_JOINT); u_zdot_limits.setZero();

	l_delta_limits.resize(NUM_ACT_JOINT); l_delta_limits.setZero();
	u_delta_limits.resize(NUM_ACT_JOINT); u_delta_limits.setZero();	

	l_delta_dot_limits.resize(NUM_ACT_JOINT); l_delta_dot_limits.setZero();
	u_delta_dot_limits.resize(NUM_ACT_JOINT); u_delta_dot_limits.setZero();

	l_current_limits.resize(NUM_ACT_JOINT); l_current_limits.setZero();
	u_current_limits.resize(NUM_ACT_JOINT); u_current_limits.setZero();	

}




void Jump_Opt_Variable_Limits::set_limits(){
	l_q_virt_limits[SJJointID::VIRTUAL_X] = -10.0; // x lower limit (m)
	l_q_virt_limits[SJJointID::VIRTUAL_Z] = 0.0;	// z lower limit (m)
	l_q_virt_limits[SJJointID::VIRTUAL_Ry] = -10.0;	// Ry lower limit (m)	

	u_q_virt_limits[SJJointID::VIRTUAL_X] = 10.0; // x upper limit (m)
	u_q_virt_limits[SJJointID::VIRTUAL_Z] = 10.0;	// z upper limit (m)
	u_q_virt_limits[SJJointID::VIRTUAL_Ry] = 10.0;	// Ry upper limit (rads)

	l_z_limits = actuator_model->z_l_bound;
	u_z_limits = actuator_model->z_u_bound;

	l_qdot_virt_limits[SJJointID::VIRTUAL_X] = -linear_velocity_limit; // x velocity lower limit (m/s)
	l_qdot_virt_limits[SJJointID::VIRTUAL_Z] = -linear_velocity_limit; // z velocity lower limit (m/s)
	l_qdot_virt_limits[SJJointID::VIRTUAL_Ry] = -linear_velocity_limit; // Ry velocity lower limit (rad/s)	

	u_qdot_virt_limits[SJJointID::VIRTUAL_X] = linear_velocity_limit; // x velocity upper limit (m/s)
	u_qdot_virt_limits[SJJointID::VIRTUAL_Z] = linear_velocity_limit; // z velocity upper limit (m/s)
	u_qdot_virt_limits[SJJointID::VIRTUAL_Ry] = linear_velocity_limit; // Ry velocity upper limit (rad/s)	

	l_zdot_limits[SJActuatorID::act_bodyPitch] = -actuator_velocity_limit; // z1 act vel lower limit (m/s)
	l_zdot_limits[SJActuatorID::act_kneePitch] = -actuator_velocity_limit;	// z2 act vel lower limit (m/s)
	l_zdot_limits[SJActuatorID::act_anklePitch] = -actuator_velocity_limit;	// z3 act vel lower limit (m/s)

	u_zdot_limits[SJActuatorID::act_bodyPitch] = actuator_velocity_limit; // z1 act vel upper limit (m/s)
	u_zdot_limits[SJActuatorID::act_kneePitch] = actuator_velocity_limit;	// z2 act vel upper limit (m/s)
	u_zdot_limits[SJActuatorID::act_anklePitch] = actuator_velocity_limit;	// z3 act vel upper limit (m/s)	

	l_delta_limits[SJActuatorID::act_bodyPitch] = -1.0; // delta_1 act spring lower limit (m)
	l_delta_limits[SJActuatorID::act_kneePitch] = -1.0;	// delta_2 act spring lower limit (m)
	l_delta_limits[SJActuatorID::act_anklePitch] = -1.0;	// delta_3 act spring lower limit (m)

	u_delta_limits[SJActuatorID::act_bodyPitch] = 1.0; // delta_1 act upper limit (m)
	u_delta_limits[SJActuatorID::act_kneePitch] = 1.0;	// delta_2 act upper limit (m)
	u_delta_limits[SJActuatorID::act_anklePitch] = 1.0;	// delta_3 act upper limit (m)

	l_delta_dot_limits[SJActuatorID::act_bodyPitch] = -actuator_velocity_limit; // delta_1 act spring vel lower limit (m/s)
	l_delta_dot_limits[SJActuatorID::act_kneePitch] = -actuator_velocity_limit;	// delta_2 act spring vel lower limit (m/s)
	l_delta_dot_limits[SJActuatorID::act_anklePitch] = -actuator_velocity_limit;	// delta_3 act spring vel lower limit (m/s)

	u_delta_dot_limits[SJActuatorID::act_bodyPitch] = actuator_velocity_limit; // delta_1 act spring vel upper limit (m/s)
	u_delta_dot_limits[SJActuatorID::act_kneePitch] = actuator_velocity_limit;	// delta_2 act spring vel upper limit (m/s)
	u_delta_dot_limits[SJActuatorID::act_anklePitch] = actuator_velocity_limit;	// delta_3 act spring vel upper limit (m/s)

	l_current_limits[SJActuatorID::act_bodyPitch] = -current_limit; // act_1 current lower limit (A)
	l_current_limits[SJActuatorID::act_kneePitch] = -current_limit;	// act_2 current lower limit (A)
	l_current_limits[SJActuatorID::act_anklePitch] = -current_limit;	// act_3 current lower limit (A)

	u_current_limits[SJActuatorID::act_bodyPitch] = current_limit; // act_1 current upper limit (A)
	u_current_limits[SJActuatorID::act_kneePitch] = current_limit;	// act_2 current upper limit (A)
	u_current_limits[SJActuatorID::act_anklePitch] = current_limit;	// act_3 current upper limit (A)	


}

