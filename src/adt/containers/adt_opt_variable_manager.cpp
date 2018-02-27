#include <adt/containers/adt_opt_variable_manager.hpp>
#include <iostream>

ADT_Opt_Variable_Manager::ADT_Opt_Variable_Manager():total_knotpoints(0){}
ADT_Opt_Variable_Manager::~ADT_Opt_Variable_Manager(){
	for(size_t i = 0; i < opt_var_list.size(); i++){
		delete opt_var_list[i];
	}
	opt_var_list.clear();
	std::cout << "[ADT_Opt_Variable_Manager] Optimization Variable Manager Destructor Called" << std::endl;
}

void ADT_Opt_Variable_Manager::append_variable(ADT_Opt_Variable* opt_variable){
	opt_var_list.push_back(opt_variable);
	std::cout << "[ADT_Opt_Variable_Manager] Adding " << opt_variable->name 
			  << " (knotpoint, value, lower, uppers) = " 
			  << "(" << opt_variable->knotpoint << ", " << opt_variable->value << ", " << opt_variable->l_bound << ", " << opt_variable->u_bound << ")"
			  << std::endl;	
	if (opt_variable->type == VAR_TYPE_Q){
		add_variable_to_map(knotpoint_to_q_state_vars, opt_variable);
	}else if(opt_variable->type == VAR_TYPE_QDOT){
		add_variable_to_map(knotpoint_to_qdot_state_vars, opt_variable);
	}else if(opt_variable->type == VAR_TYPE_TA){
		add_variable_to_map(knotpoint_to_xddot_vars, opt_variable);		
	}else if(opt_variable->type == VAR_TYPE_FR){
		add_variable_to_map(knotpoint_to_Fr_vars, opt_variable);		
	}else if(opt_variable->type == VAR_TYPE_KF){
		add_variable_to_map(knotpoint_to_keyframe_vars, opt_variable);		
	}else if(opt_variable->type == VAR_TYPE_Z){
		add_variable_to_map(knotpoint_to_z_vars, opt_variable);					
	}else if(opt_variable->type == VAR_TYPE_ZDOT){
		add_variable_to_map(knotpoint_to_zdot_vars, opt_variable);				
	}else if(opt_variable->type == VAR_TYPE_DELTA){
		add_variable_to_map(knotpoint_to_delta_vars, opt_variable);				
	}else if(opt_variable->type == VAR_TYPE_DELTA_DOT){
		add_variable_to_map(knotpoint_to_delta_dot_vars, opt_variable);				
	}else if(opt_variable->type == VAR_TYPE_U){
		add_variable_to_map(knotpoint_to_u_vars, opt_variable);
	}else if(opt_variable->type == VAR_TYPE_BETA){
		add_variable_to_map(knotpoint_to_beta_vars, opt_variable);						
	}else if(opt_variable->type == VAR_TYPE_H){
		knotpoint_to_dt.push_back(opt_variable);
	}

/*	std::cout << "Size of Q map[knotpoint = 0]:    " << knotpoint_to_q_state_vars[0].size() << std::endl;
	std::cout << "Size of Qdot map[knotpoint = 0]: " << knotpoint_to_qdot_state_vars[0].size() << std::endl;	
	std::cout << "Size of xddot map[knotpoint = 0]:    " << knotpoint_to_xddot_vars[0].size() << std::endl;
	std::cout << "Size of Fr map[knotpoint = 0]: " << knotpoint_to_Fr_vars[0].size() << std::endl;		
	std::cout << "Size of KF map[knotpoint = 0]: " << knotpoint_to_keyframe_vars[0].size() << std::endl;	*/		
}


int ADT_Opt_Variable_Manager::get_size(){
	return opt_var_list.size();
}

ADT_Opt_Variable* ADT_Opt_Variable_Manager::get_opt_variable(const int index){
	if ((index >= 0) && (index < opt_var_list.size())){
		return opt_var_list[index];
	}else{
		std::cerr << "Error retrieving optimization variable. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}


void ADT_Opt_Variable_Manager::add_variable_to_map(std::map<int, std::vector<ADT_Opt_Variable*>  > &map_kp_to_var_vec, ADT_Opt_Variable* opt_variable){
	int knotpoint = opt_variable->knotpoint;
	if (map_kp_to_var_vec.count(knotpoint) != 0){
		// If key is found add this optimization variable to the current time step
		map_kp_to_var_vec[knotpoint].push_back(opt_variable);
	}else{
		// If key is not found, construct a new vector and add it to the map.
		std::vector<ADT_Opt_Variable*> var_vec = {opt_variable};
		map_kp_to_var_vec.insert(  std::make_pair(knotpoint, var_vec) );
	}
}

void ADT_Opt_Variable_Manager::get_var_states(const int &knotpoint, sejong::Vector &q_state, sejong::Vector &qdot_state){
		convert_to_vector(knotpoint, knotpoint_to_q_state_vars, q_state);
		convert_to_vector(knotpoint, knotpoint_to_qdot_state_vars, qdot_state);		

/*		for(size_t i = 0; i < q_state.size(); i++){
			std::cout << " q_vec[i] val = " << q_state[i] << std::endl;
		}*/
/*		for(size_t i = 0; i < qdot_state.size(); i++){
			std::cout << " qdot_vec[i] val = " << qdot_state[i] << std::endl;
		}		*/

}

void ADT_Opt_Variable_Manager::get_task_accelerations(const int &knotpoint, sejong::Vector &xddot){
		convert_to_vector(knotpoint, knotpoint_to_xddot_vars, xddot);
/*		for(size_t i = 0; i < xddot.size(); i++){
			std::cout << " xddot[i] val = " << xddot[i] << std::endl;
		}		*/

}

void ADT_Opt_Variable_Manager::get_var_reaction_forces(const int &knotpoint, sejong::Vector &Fr_state){
		convert_to_vector(knotpoint, knotpoint_to_Fr_vars, Fr_state);
/*		for(size_t i = 0; i < Fr_state.size(); i++){
			std::cout << " Fr_state[i] val = " << Fr_state[i] << std::endl;
		}		*/
		
}

void ADT_Opt_Variable_Manager::get_var_keyframes(const int &knotpoint, sejong::Vector &keyframe_state){
		convert_to_vector(knotpoint, knotpoint_to_keyframe_vars, keyframe_state);
}


void ADT_Opt_Variable_Manager::get_var_knotpoint_dt(const int &knotpoint, double &h_dt){
	h_dt = knotpoint_to_dt[knotpoint]->value;
}




void ADT_Opt_Variable_Manager::get_q_states(const int &knotpoint, sejong::Vector &q_state){
	convert_to_vector(knotpoint, knotpoint_to_q_state_vars, q_state);
}
void ADT_Opt_Variable_Manager::get_qdot_states(const int &knotpoint, sejong::Vector &qdot_state){
	convert_to_vector(knotpoint, knotpoint_to_qdot_state_vars, qdot_state);
}			
void ADT_Opt_Variable_Manager::get_z_states(const int &knotpoint, sejong::Vector &z_state){
	convert_to_vector(knotpoint, knotpoint_to_z_vars, z_state);
}
void ADT_Opt_Variable_Manager::get_zdot_states(const int &knotpoint, sejong::Vector &zdot_state){
	convert_to_vector(knotpoint, knotpoint_to_zdot_vars, zdot_state);	
}

void ADT_Opt_Variable_Manager::get_delta_states(const int &knotpoint, sejong::Vector &delta_state){
	convert_to_vector(knotpoint, knotpoint_to_delta_vars, delta_state);
}		

void ADT_Opt_Variable_Manager::get_delta_dot_states(const int &knotpoint, sejong::Vector &delta_dot_state){
	convert_to_vector(knotpoint, knotpoint_to_delta_dot_vars, delta_dot_state);	
}
void ADT_Opt_Variable_Manager::get_u_states(const int &knotpoint, sejong::Vector &u_state){
	convert_to_vector(knotpoint, knotpoint_to_u_vars, u_state);
}

void ADT_Opt_Variable_Manager::get_beta_states(const int &knotpoint, sejong::Vector &beta_state){
	convert_to_vector(knotpoint, knotpoint_to_beta_vars, beta_state);
}


void ADT_Opt_Variable_Manager::convert_to_vector(const int &knotpoint, 
						  	 	   			  std::map<int, std::vector<ADT_Opt_Variable*> > &map_kp_to_var_vec,
						   				 	  sejong::Vector &vec_out){
	
	std::map<int, std::vector<ADT_Opt_Variable*> >::iterator it;
	it = map_kp_to_var_vec.find(knotpoint);

	// if the key in the map has been found
	if (it != map_kp_to_var_vec.end()){
		int size_of_vec = it->second.size(); //is the size of std::vector<ADT_Opt_Variable*>
		vec_out.resize(size_of_vec);

		// Populate vector
		for(size_t i = 0; i < it->second.size(); i++){
			vec_out[i] = it->second[i]->value;
		}

	}

}

int ADT_Opt_Variable_Manager::get_num_q_vars(){
	return num_q_vars;
}
int ADT_Opt_Variable_Manager::get_num_qdot_vars(){
	return num_qdot_vars;
}
int ADT_Opt_Variable_Manager::get_num_xddot_vars(){
	return num_xddot_vars;
}
int ADT_Opt_Variable_Manager::get_num_Fr_vars(){
	return num_Fr_vars;
}
int ADT_Opt_Variable_Manager::get_num_keyframe_vars(){
	return num_keyframe_vars;
}

int ADT_Opt_Variable_Manager::get_num_var_knotpoint_dt(){
	num_knotpoint_dt_vars = knotpoint_to_dt.size();
	return num_knotpoint_dt_vars;
}


int ADT_Opt_Variable_Manager::get_num_z_vars(){
	return num_z_vars;
}
int ADT_Opt_Variable_Manager::get_num_u_vars(){
	return num_u_vars;
}



void ADT_Opt_Variable_Manager::compute_size_time_dep_vars(){
	int knotpoint = 1;
	int total_j_size = 0;
	
	num_q_vars = count_num_vars_in_map(knotpoint, knotpoint_to_q_state_vars);
	num_qdot_vars = count_num_vars_in_map(knotpoint, knotpoint_to_qdot_state_vars);
	num_xddot_vars = count_num_vars_in_map(knotpoint, knotpoint_to_xddot_vars);
	num_Fr_vars = count_num_vars_in_map(knotpoint, knotpoint_to_Fr_vars);
	num_keyframe_vars = count_num_vars_in_map(knotpoint, knotpoint_to_keyframe_vars);

	num_z_vars = count_num_vars_in_map(knotpoint, knotpoint_to_z_vars);
	num_zdot_vars = count_num_vars_in_map(knotpoint, knotpoint_to_zdot_vars);
	num_delta_vars = count_num_vars_in_map(knotpoint, knotpoint_to_delta_vars);
	num_delta_dot_vars = count_num_vars_in_map(knotpoint, knotpoint_to_delta_dot_vars);
	num_u_vars = count_num_vars_in_map(knotpoint, knotpoint_to_u_vars);
	num_beta_vars = count_num_vars_in_map(knotpoint, knotpoint_to_beta_vars);
	num_h = knotpoint_to_dt.size();


/*	std::cout << std::endl;
	std::cout << "num_xddot_vars = " << num_xddot_vars << std::endl;
	std::cout << "num_keyframe_vars = " << num_keyframe_vars << std::endl;
	std::cout << std::endl;

	std::cout << "num_q_vars = " << num_q_vars << std::endl;
	std::cout << "num_qdot_vars = " << num_qdot_vars << std::endl;
	std::cout << "num_z_vars = " << num_z_vars << std::endl;
	std::cout << "num_zdot_vars = " << num_zdot_vars << std::endl;
	std::cout << "num_delta_vars = " << num_delta_vars << std::endl;
	std::cout << "num_delta_dot_vars = " << num_delta_dot_vars << std::endl;
	std::cout << "num_u_vars = " << num_u_vars << std::endl;
	std::cout << "num_Fr_vars = " << num_Fr_vars << std::endl;		
	std::cout << "num_beta_vars = " << num_beta_vars << std::endl;
	std::cout << "num_h = " << num_h << std::endl;
*/
	num_timedep_vars = num_q_vars + num_qdot_vars + num_xddot_vars + num_Fr_vars
					   + num_keyframe_vars
					   + num_z_vars
					   + num_zdot_vars
					   + num_delta_vars
					   + num_delta_dot_vars					   
					   + num_u_vars 
					   + num_beta_vars
					   + num_h;					   
}

int ADT_Opt_Variable_Manager::get_size_timedependent_vars(){
	return num_timedep_vars;
}

int ADT_Opt_Variable_Manager::count_num_vars_in_map(const int &knotpoint, std::map<int, std::vector<ADT_Opt_Variable*> > &map_kp_to_var_vec){
	std::map<int, std::vector<ADT_Opt_Variable*> >::iterator it;	
	it = map_kp_to_var_vec.find(knotpoint);

	if (it != map_kp_to_var_vec.end()){
		return it->second.size(); //is the size of std::vector<ADT_Opt_Variable*>
	}else{
		return 0;
	}

}

void ADT_Opt_Variable_Manager::update_x(std::vector<double> &x_in){
	if (x_in.size() == (opt_var_list.size() - initial_conditions_offset) ){
		//std::cout << "[VAR LIST] input and stored sizes are equal" << std::endl;
		// Update the values
		for (size_t i = 0; i < x_in.size(); i++){
//			std::cout << "old var_list[" << i << "] = " << opt_var_list[i]->value << std::endl;
			opt_var_list[i + initial_conditions_offset]->value = x_in[i];
//			std::cout << "new var_list[" << i << "] = " << opt_var_list[i]->value << std::endl;			
		}
	}else{
		std::cout << "[VAR LIST] Error. Input and stored sizes are not equal" << std::endl;
	}
}

void ADT_Opt_Variable_Manager::populate_x(std::vector<double> &x_out){
	x_out.clear();
	for (size_t i = initial_conditions_offset; i < opt_var_list.size(); i++){
		x_out.push_back(opt_var_list[i]->value);
	}

}


