#ifndef ADT_2D_FRICTION_CONE_CONSTRAINT_H
#define ADT_2D_FRICTION_CONE_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <adt/hard_constraints/adt_constraint_main.hpp>
#include <adt/containers/adt_contact_list.hpp>

class Friction_Cone_2D_Constraint: public Constraint_Function{
public:
	Friction_Cone_2D_Constraint();
	Friction_Cone_2D_Constraint(Contact_List* contact_list_in, int &index_in);	
	~Friction_Cone_2D_Constraint();

	void setContact_List(Contact_List* contact_list_in);
	void setContact_index(int index_in);	

	void evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	Contact_List* contact_list_obj;
	int contact_index = -1;	
	int Nd = 2; // Number of basis vectors
	double mu = 0.8; // Coefficient of friction

	sejong::Vector n1; 	sejong::Vector n2;
	sejong::Vector d1; 	sejong::Vector d2;		
	sejong::Vector w1;  sejong::Vector w2;

	void Initialization();
	void initialize_Flow_Fupp();

	void UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);
};
#endif