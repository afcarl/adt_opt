#ifndef ADT_POS_2D_KINEMATIC_CONSTRAINT_H
#define ADT_POS_2D_KINEMATIC_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include "DracoModel.hpp"
#include <adt/hard_constraints/adt_constraint_main.hpp>
#include <draco_actuator_model/DracoActuatorModel.hpp>
#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>

#define X_DIM 0
#define Z_DIM 1
#define Ry_DIM 2


class Position_2D_Kinematic_Constraint: public Constraint_Function{
public:
	Position_2D_Kinematic_Constraint(int knotpoint_in, int link_id_in, int dim_in, double l_bound_in, double u_bound_in);	
	~Position_2D_Kinematic_Constraint();

	int link_id;
	int dim;

	double l_bound;
	double u_bound;

	DracoModel* robot_model;	
	DracoActuatorModel* actuator_model;
	Draco_Combined_Dynamics_Model* combined_model;	

  	sejong::Vector x_state;
	sejong::Vector xdot_state;

	sejong::Vector q_state;
	sejong::Vect3 pos;

	void evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	void Initialization();
	void initialize_Flow_Fupp();
	void update_states(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager);


};
#endif