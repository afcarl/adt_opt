#ifndef DRACO_P1_COMBINED_DYNAMICS_MODEL
#define DRACO_P1_COMBINED_DYNAMICS_MODEL

#include <Utils/wrap_eigen.hpp>

#include <draco_actuator_model/DracoActuatorModel.hpp>
#include "DracoModel.hpp"

class Draco_Combined_Dynamics_Model{
public:
    static Draco_Combined_Dynamics_Model* GetDracoCombinedDynamicsModel();
    ~Draco_Combined_Dynamics_Model(void);

	DracoModel* robot_model;	
	DracoActuatorModel* actuator_model;


	sejong::Matrix M_combined;
	sejong::Matrix B_combined;
	sejong::Matrix K_combined;

	sejong::Vector A_mat; // NUM_QDOT x NUM_QDT
	sejong::Vector A_bb; // NUM_VIRTUAL x NUM_VIRTUAL
	sejong::Vector A_br; // NUM_VIRTUAL x NUM_ACT_JOINT
	sejong::Vector A_brT; // NUM_ACT_JOINT x NUM_VIRTUAL
	sejong::Vector A_rr; // NUM_ACT_JOINT x NUM_ACT_JOINT

	sejong::Vector grav;
	sejong::Vector coriolis;

	sejong::Matrix Sv; // Virtual Dynamics Selection Matrix
	sejong::Matrix Sa; // Actuated Dynamics Selection Matrix			

	sejong::Matrix Jc; // Contact Jacobian;
	sejong::Matrix L; // dz/dq Jacobian;
	sejong::Matrix J; // dq/dz Jacobian

	sejong::Vector x_state;
	sejong::Vector xdot_state;	

	sejong::Vector q_state;
	sejong::Vector qdot_state;	

	sejong::Vector q_virt;
	sejong::Vector qdot_virt;	
	sejong::Vector q_act;
	sejong::Vector qdot_act;

	void UpdateModel(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in);
	void setContactJacobian(sejong::Matrix Jc_in);

	void get_combined_mass_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &M_out);
	void get_combined_damping_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &B_out);
	void get_combined_stiffness_matrix(const sejong::Vector &x_state, sejong::Matrix &K_out);

	void get_virtual_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sv_out);
	void get_actuated_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sa_out);			

	void get_state_acceleration(const sejong::Vector &x_state, const::sejong::Vector &xdot_state, sejong::Vector &xddot_state_out);


protected:
	void convert_x_xdot_to_q_qdot(const sejong::Vector &x_state, const sejong::Vector &xdot_state);

	void compute_mass_matrix();
	void compute_damping_matrix();	
	void compute_stiffness_matrix();
	void compute_joint_link_impedance();			



private:
    Draco_Combined_Dynamics_Model();

};

#endif