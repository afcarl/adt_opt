#ifndef ADT_TIME_INTEGRATION_CONSTRAINT_H
#define ADT_TIME_INTEGRATION_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <adt/hard_constraints/adt_constraint_main.hpp>
#include <adt/containers/adt_contact_list.hpp>

#include "DracoModel.hpp"
#include <draco_actuator_model/DracoActuatorModel.hpp>
#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>

#include <adt/containers/adt_contact_list.hpp>

class Dynamics_and_Linear_Back_Euler_Time_Integration_Constraint: public Constraint_Function{
public:
	Dynamics_and_Linear_Back_Euler_Time_Integration_Constraint();
	Dynamics_and_Linear_Back_Euler_Time_Integration_Constraint(Contact_List* contact_list_in);	
	~Dynamics_and_Linear_Back_Euler_Time_Integration_Constraint();

	DracoModel* robot_model;	
	DracoActuatorModel* actuator_model;
	Draco_Combined_Dynamics_Model* combined_model;	

	// These states are exclusively for timestep k
	sejong::Matrix A_mat;
	sejong::Matrix B_mat;
	sejong::Matrix K_mat;
	sejong::Matrix Jc; // Contact Jacobian
	sejong::Vector input;

	sejong::Vector q_state_virt;
	sejong::Vector z_state;  
	sejong::Vector delta_state;

	sejong::Vector qdot_state_virt;
	sejong::Vector zdot_state;    
	sejong::Vector delta_dot_state;  

	void setContact_List(Contact_List* contact_list_in);

	void evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	Contact_List* contact_list_obj;

	void Initialization();
	void initialize_Flow_Fupp();

	void Update_Contact_Jacobian_Jc(sejong::Vector &x_state);
	void get_states(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, sejong::Vector &x_state, sejong::Vector &xdot_state);
};
#endif