#ifndef DRACO_P1_COMBINED_DYNAMICS_MODEL
#define DRACO_P1_COMBINED_DYNAMICS_MODEL

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>

#include <draco_actuator_model/DracoActuatorModel.hpp>
#include "DracoModel.hpp"

class Draco_Combined_Dynamics_Model{
public:
	Draco_Combined_Dynamics_Model();
	~Draco_Combined_Dynamics_Model();

	DracoModel* robot_model;	
	DracoActuatorModel* actuator_model;


	sejong::Matrix M_combined;
	sejong::Matrix B_combined;
	sejong::Matrix K_combined;

	sejong::Vector A_mat;
	sejong::Vector A_bb;
	sejong::Vector A_br;
	sejong::Vector A_brT;
	sejong::Vector A_rr;

	sejong::Vector grav;
	sejong::Vector coriolis;

	sejong::Matrix Sv; // Virtual Dynamics Selection Matrix
	sejong::Matrix Sa; // Actuated Dynamics Selection Matrix			

	sejong::Matrix Jc; // Contact Jacobian;
	sejong::Matrix L; // dz/dq Jacobian;
	sejong::Matrix J; // dq/dz Jacobian


	void get_combined_mass_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &M_out);
	void get_combined_damping_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &B_out);
	void get_combined_stiffness_matrix(const sejong::Vector &x_state, sejong::Matrix &K_out);

	void get_virtual_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sv_out);
	void get_actuated_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sa_out);			

	void get_state_acceleration(const sejong::Vector &x_state, const::sejong::Vector &xdot_state, sejong::Vector &xddot_state_out);
}