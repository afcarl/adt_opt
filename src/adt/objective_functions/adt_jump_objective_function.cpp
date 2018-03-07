#include <adt/objective_functions/adt_jump_objective_function.hpp>
#include <Utils/utilities.hpp>

Jump_Objective_Function::Jump_Objective_Function(){}

Jump_Objective_Function::~Jump_Objective_Function(){
	std::cout << "Jump Objective Function Destructor called" << std::endl;
}

void Jump_Objective_Function::set_var_manager(ADT_Opt_Variable_Manager& var_manager){
	num_q_virt = var_manager.get_num_q_vars();
	num_z = var_manager.get_num_z_vars();
	num_delta = var_manager.get_num_delta_vars();
	num_Fr = var_manager.get_num_Fr_vars();
	num_u = var_manager.get_num_u_vars();
	num_beta = var_manager.get_num_beta_vars();
	N_total_knotpoints = var_manager.total_knotpoints;

	int num_xddot_all = num_q_virt + num_z + num_delta;

	c_u = 20.0;
	c_qdotvirt = 10.0;
	c_zdot = 10.0;
	c_delta_dot = 10.0;
	c_fr = 1e-2;
	c_beta = c_fr;
	c_ik = 1e-4;

	Q_qvirt = c_ik*sejong::Matrix::Identity(num_q_virt, num_q_virt);
	Q_z = c_ik*sejong::Matrix::Identity(num_z, num_z);

	Q_u = c_u * sejong::Matrix::Identity(num_u, num_u);
	Q_qdotvirt = c_qdotvirt * sejong::Matrix::Identity(num_q_virt, num_q_virt);
	Q_zdot = c_zdot * sejong::Matrix::Identity(num_z, num_z);
	Q_delta_dot = c_delta_dot * sejong::Matrix::Identity(num_delta, num_delta);	
	Qfr_mat = c_fr * sejong::Matrix::Identity(num_Fr, num_Fr);
	Q_beta = c_beta * sejong::Matrix::Identity(num_beta, num_beta);

}

void Jump_Objective_Function::evaluate_objective_function(ADT_Opt_Variable_Manager& var_manager, double &result){
	sejong::Vector u_states;	
	sejong::Vector qdot_states;
	sejong::Vector zdot_states;
	sejong::Vector delta_dot_states;
	sejong::Vector Fr_states;
	sejong::Vector beta_states;

	sejong::Vector qvirt_init;
	sejong::Vector z_init;

	sejong::Vector qvirt_state;
	sejong::Vector z_state;	

	sejong::Vector xddot_all_states;	
	double cost = 0.0;
	double h_k = 1.0; 

	var_manager.get_q_states(0, qvirt_init);
	var_manager.get_z_states(0, z_init);		

	for(size_t k = 1; k < N_total_knotpoints + 1; k++){
		var_manager.get_u_states(k, u_states);
		var_manager.get_qdot_states(k, qdot_states);
		var_manager.get_zdot_states(k, zdot_states);
		var_manager.get_delta_dot_states(k, delta_dot_states);
		var_manager.get_xddot_all_states(k, xddot_all_states);
		var_manager.get_var_knotpoint_dt(k-1, h_k);
		var_manager.get_var_reaction_forces(k, Fr_states);

		var_manager.get_q_states(k, qvirt_state);
		var_manager.get_z_states(k, z_state);		
		var_manager.get_beta_states(k, beta_states);  

		cost += (qvirt_state - qvirt_init).transpose()*Q_qvirt*(qvirt_state - qvirt_init);
		cost += (z_state - z_init).transpose()*Q_z*(z_state - z_init);
		cost += u_states.transpose()*Q_u*u_states;
		cost += qdot_states.transpose()*Q_qdotvirt*qdot_states;
		cost += zdot_states.transpose()*Q_zdot*zdot_states;
		// cost += delta_dot_states.transpose()*Q_delta_dot*delta_dot_states;
		//cost += Fr_states.transpose()*Qfr_mat*Fr_states;	
		cost += beta_states.transpose()*Q_beta*beta_states;
		cost *= h_k;

		//std::cout << "Fr cost:" << Fr_states.transpose()*Qfr_mat*Fr_states << std::endl;
		//std::cout << "qdot cost:" << qdot_states.transpose()*Q_qdotvirt*qdot_states << std::endl;		

		//std::cout << "cost = " << cost << std::endl;
	}
	result = cost;
}


void Jump_Objective_Function::evaluate_objective_gradient(ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Jump_Objective_Function::evaluate_sparse_A_matrix(ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

void Jump_Objective_Function::setQ_vals(const int &i, const int &j, const double &value){}