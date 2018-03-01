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
	sejong::Matrix M_combined_inv;
	sejong::Matrix B_combined;
	sejong::Matrix K_combined;

	sejong::Matrix M_act;
	sejong::Matrix M_zz;	
	sejong::Matrix M_z_delta;
	sejong::Matrix M_delta_z;	
	sejong::Matrix M_delta_delta;	

	sejong::Matrix B_act;
	sejong::Matrix B_zz;	
	sejong::Matrix B_z_delta;
	sejong::Matrix B_delta_z;	
	sejong::Matrix B_delta_delta;

	sejong::Matrix K_act;		
	sejong::Matrix K_zz;	
	sejong::Matrix K_z_delta;
	sejong::Matrix K_delta_z;	
	sejong::Matrix K_delta_delta;

	sejong::Matrix Km_act;

	sejong::Matrix A_mat; // NUM_QDOT x NUM_QDT
	sejong::Matrix A_bb; // NUM_VIRTUAL x NUM_VIRTUAL
	sejong::Matrix A_br; // NUM_VIRTUAL x NUM_ACT_JOINT
	sejong::Matrix A_brT; // NUM_ACT_JOINT x NUM_VIRTUAL
	sejong::Matrix A_rr; // NUM_ACT_JOINT x NUM_ACT_JOINT

	sejong::Vector grav;
	sejong::Vector coriolis;

	sejong::Matrix Sv; // Virtual Dynamics Selection Matrix
	sejong::Matrix Sa; // Actuated Dynamics Selection Matrix			

	sejong::Matrix Jc; // Contact Jacobian;
	sejong::Matrix L; // dz/dq Jacobian;
	sejong::Matrix J; // dq/dz Jacobian

	sejong::Vector x_state;
	sejong::Vector xdot_state;	

	sejong::Vector z_state;
	sejong::Vector zdot_state;	
	
	sejong::Vector q_virt_state;
	sejong::Vector qdot_virt_state;
	
	sejong::Vector q_act;
	sejong::Vector qdot_act;	

	//sejong::Vector delta_state;
	//sejong::Vector delta_dot_state;

	sejong::Vector q_state;
	sejong::Vector qdot_state;	

	// Input/Impedances
	sejong::Vector total_imp;
	sejong::Vector virt_imp;
	sejong::Vector current_input;
	sejong::Vector joint_imp;		



	void UpdateModel(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in);
	void setContactJacobian(sejong::Matrix &Jc_in);

	void get_combined_mass_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &M_out);
	void get_combined_damping_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &B_out);
	void get_combined_stiffness_matrix(const sejong::Vector &x_state, sejong::Matrix &K_out);

	void get_virtual_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sv_out);
	void get_actuated_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sa_out);			

	void get_state_acceleration(const sejong::Vector &x_state_in, const::sejong::Vector &xdot_state_in, 
							    const sejong::Vector &u_current_in, const::sejong::Vector &Fr_state_in,
							    sejong::Vector &xddot_state_out);
	
	void convert_x_xdot_to_q_qdot(const sejong::Vector &x_state, 
								  const sejong::Vector &xdot_state, 
								  sejong::Vector &q_state_out, 
								  sejong::Vector &qdot_state_out);

	void convert_x_to_q(const sejong::Vector &x_state, sejong::Vector &q_state_out);
	void convert_xdot_to_qdot(const sejong::Vector &xdot_state, sejong::Vector &qdot_state_out);

	void getDynamics_constraint(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in, const sejong::Vector &xddot_state_in,
							    const sejong::Vector &u_current_in, const sejong::Vector &Fr_state_in, sejong::Vector &dynamics_out);
							    				
	void getDynamics_constraint(const sejong::Vector &x_state_k, const sejong::Vector &xdot_state_k, const sejong::Vector &xdot_state_k_prev, 
                                                           const sejong::Vector &u_current_k, const sejong::Vector &Fr_state_k, double h_k, sejong::Vector &dynamics_out);


protected:

	void formulate_mass_matrix();
	void formulate_damping_matrix();	
	void formulate_stiffness_matrix();
	void formulate_joint_link_impedance(const::sejong::Vector &Fr_state_in);			

	void Initialization();
	void initialize_actuator_matrices(sejong::Matrix &Mat);

private:
    Draco_Combined_Dynamics_Model();

};

#endif