#include <adt/objective_functions/adt_jump_objective_function.hpp>
#include <Utils/utilities.hpp>

Jump_Objective_Function::Jump_Objective_Function(){}

Jump_Objective_Function::~Jump_Objective_Function(){
	std::cout << "Jump Objective Function Destructor called" << std::endl;
}

void Jump_Objective_Function::set_var_manager(ADT_Opt_Variable_Manager& var_manager){
	num_q_virt = var_manager.get_num_q_vars();
	num_z = var_manager.get_num_z_vars();
	num_Fr = var_manager.get_num_Fr_vars();
	num_u = var_manager.get_num_u_vars();
	N_total_knotpoints =  var_manager.total_knotpoints;
}

void Jump_Objective_Function::evaluate_objective_function(ADT_Opt_Variable_Manager& var_manager, double result){}
void Jump_Objective_Function::evaluate_objective_gradient(ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Jump_Objective_Function::evaluate_sparse_A_matrix(ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

void Jump_Objective_Function::setQ_vals(const int &i, const int &j, const double &value){}