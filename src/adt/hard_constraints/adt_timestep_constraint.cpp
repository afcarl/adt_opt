#include <adt/hard_constraints/adt_timestep_constraint.hpp>
#include <Utils/utilities.hpp>

Time_Stepping_Constraint::Time_Stepping_Constraint(int j_in){
	internal_j = j_in;
	Initialization();
}

Time_Stepping_Constraint::~Time_Stepping_Constraint(){
	std::cout << "[Time_Stepping_Constraint] Destructor called" << std::endl;
}

void Time_Stepping_Constraint::Initialization(){
	constraint_name = "Time_Stepping_Constraint 2D Constraint at starting j " + std::to_string(internal_j);	
	initialize_Flow_Fupp();	
}


void Time_Stepping_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

	// Single equality Constraint
	// h_{2j-1} + h_{2j} = h_{2j+1} + h_{2j+2}
	F_low.push_back(0.0);	
	F_upp.push_back(0.0);

	constraint_size = F_low.size();
}

void Time_Stepping_Constraint::evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
	F_vec.clear();

	int k1 = 2*internal_j - 1;
	int k2 = 2*internal_j;
	int k3 = 2*internal_j + 1;
	int k4 = 2*internal_j + 2;

	double h1, h2, h3, h4;
	var_manager.get_var_knotpoint_dt(k1 - 1, h1);
	var_manager.get_var_knotpoint_dt(k2 - 1, h2);
	var_manager.get_var_knotpoint_dt(k3 - 1, h3);
	var_manager.get_var_knotpoint_dt(k4 - 1, h4);

	double time_step_constriant = h1 + h2 - h3 - h4;			

	F_vec.push_back(time_step_constriant);

}
void Time_Stepping_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Time_Stepping_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

