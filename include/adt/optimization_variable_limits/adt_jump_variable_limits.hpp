#ifndef ADT_VARIABLE_LIMITS_H
#define ADT_VARIABLE_LIMITS_H

#include <Utils/wrap_eigen.hpp>
#include <adt/optimization_constants.hpp>

#include "DracoP1Rot_Definition.h"
#include <draco_actuator_model/DracoActuatorModel.hpp>
#include <adt/containers/adt_contact_list.hpp>

class Jump_Opt_Variable_Limits{
public:
	// Optimization Variable Order:
	// opt_init = [q_virt, z, qdot_virt, zdot, delta, delta_dot]
	// opt_td_k = [q_virt_k, z_k, qdot_virt_k, zdot_k, delta_k, delta_dot_k, u_k, Fij_k, Bij_k]
	// opt_var = [opt_init, opt_td_1, opt_td_2, ..., opt_td_k]

	Contact_List* contact_list;

	double current_limit = 100.0; // Amps
	double linear_velocity_limit = 10; // m/s
	double actuator_velocity_limit = 15; // m/s
	double contact_forces_limit = 10000.0; // Newtons


	sejong::Vector l_q_virt_limits;
	sejong::Vector u_q_virt_limits;	

	sejong::Vector l_z_limits;
	sejong::Vector u_z_limits;		

	sejong::Vector l_qdot_virt_limits;
	sejong::Vector u_qdot_virt_limits;	

	sejong::Vector l_zdot_limits;
	sejong::Vector u_zdot_limits;	

	sejong::Vector l_delta_limits;
	sejong::Vector u_delta_limits;	

	sejong::Vector l_delta_dot_limits;
	sejong::Vector u_delta_dot_limits;		

	sejong::Vector l_current_limits;
	sejong::Vector u_current_limits;	

	sejong::Vector l_force_limits;
	sejong::Vector u_force_limits;	

	sejong::Vector l_beta_basis_limits;
	sejong::Vector u_beta_basis_limits;		

	Jump_Opt_Variable_Limits();
	Jump_Opt_Variable_Limits(Contact_List* contact_list_in);	
	
	void Initialization();
	void initialize_to_zero();
	void set_limits();
};





#endif