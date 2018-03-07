#ifndef ADT_TIME_STEPPING_CONSTRAINT_H
#define ADT_TIME_STEPPING_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <adt/hard_constraints/adt_constraint_main.hpp>
#include <adt/containers/adt_opt_variable_manager.hpp>

class Time_Stepping_Constraint: public Constraint_Function{
public:
	Time_Stepping_Constraint(int j_in);
	~Time_Stepping_Constraint();

	void evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	

private:
	int internal_j;

	void Initialization();
	void initialize_Flow_Fupp();

};
#endif