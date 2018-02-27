#ifndef ADT_OBJ_FUNC_WBC_SIMPLE_H
#define ADT_OBJ_FUNC_WBC_SIMPLE_H

#include <adt/objective_functions/adt_objective_function_main.hpp>
class Jump_Objective_Function: public Objective_Function{
public:
	Jump_Objective_Function();
	~Jump_Objective_Function();


	void evaluate_objective_function(ADT_Opt_Variable_Manager& var_manager, double &result);
	void evaluate_objective_gradient(ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA) ;

	void set_var_manager(ADT_Opt_Variable_Manager& var_manager);
	void setQ_vals(const int &i, const int &j, const double &value);

	std::string objective_function_name = "adt_jump_objective function";	

	sejong::Matrix Q_u; // Cost matrix for current input
	sejong::Matrix Q_qdotvirt; // Cost matrix
	sejong::Matrix Q_zdot; // Cost matrix for actuator vel states	
	sejong::Matrix Q_delta_dot; // Cost matrix for spring vel
	double c_u;
	double c_qdotvirt;
	double c_zdot;
	double c_delta_dot;	

	sejong::Matrix Q_mat; // Cost Matrix for reaction forces
	sejong::Matrix Qdot_mat; // Cost Matrix for time derivative reaction forces

	sejong::Matrix N_mat; // Cost Matrix for task acceleration
	sejong::Matrix R_mat; // Cost Matrix for keyframes

	sejong::Matrix S_mat; // Cost Matrix for qdot states

	int num_q_virt;
	int num_z;
	int num_delta;
	int num_Fr;
	int num_u;
	int N_total_knotpoints;

};



#endif