#ifndef ADT_FLOOR_2D_CONTACT_LCP_CONSTRAINT_H
#define ADT_FLOOR_2D_CONTACT_LCP_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <adt/hard_constraints/adt_constraint_main.hpp>
#include <adt/containers/adt_contact_list.hpp>

#include "DracoModel.hpp"

class Floor_2D_Contact_LCP_Constraint: public Constraint_Function{
public:
	Floor_2D_Contact_LCP_Constraint();
	Floor_2D_Contact_LCP_Constraint(Contact_List* contact_list_in, int index_in);	
	~Floor_2D_Contact_LCP_Constraint();

	DracoModel* robot_model;	

	void setContact_List(Contact_List* contact_list_in);
	void setContact_index(int index_in);	

	void evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	const int num_constraints = 2;
	Contact_List* contact_list_obj;
	int contact_index = -1;	

	void Initialization();
	void initialize_Flow_Fupp();

	void UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);
};
#endif