#include <iostream>
#include <Utils/utilities.hpp>

#include <adt/optimization_problems/adt_opt_jump_problem.hpp>
#include <adt/adt_snopt_wrapper.hpp>
#include <adt/containers/adt_opt_variable_manager.hpp>

void parse_output(Optimization_Problem_Main* opt_prob){
	ADT_Opt_Variable_Manager* var_manager;

	opt_prob->get_var_manager(var_manager);
	std::vector<sejong::Vector> vec_q_virt_states;
	std::vector<sejong::Vector> vec_z_states;	
	std::vector<sejong::Vector> vec_delta_states;
	std::vector<sejong::Vector> vec_qdot_virt_states;
	std::vector<sejong::Vector> vec_zdot_states;	
	std::vector<sejong::Vector> vec_delta_dot_states;

	std::vector<sejong::Vector> vec_u_current;	
	std::vector<sejong::Vector> vec_Fr_states;	
	std::vector<sejong::Vector> vec_beta_states;		
	std::vector<double> vec_h_dt;		

	sejong::Vector q_virt_states;
	sejong::Vector z_states;	
	sejong::Vector delta_states;
	sejong::Vector qdot_virt_states;
	sejong::Vector zdot_states;	
	sejong::Vector delta_dot_states;

	sejong::Vector u_current;	
	sejong::Vector Fr_states;	
	sejong::Vector beta_states;		
	double h_dt;


	for(size_t k = 1; k < var_manager->total_knotpoints+1; k++){
	 	std::cout << "--------------------------" << std::endl;
	 	std::cout << "knotpoint = " << k << std::endl;
	 	var_manager->get_q_states(k, q_virt_states); 	
	 	var_manager->get_z_states(k, z_states); 
	 	var_manager->get_delta_states(k, delta_states);
	 	var_manager->get_qdot_states(k, qdot_virt_states); 	
	 	var_manager->get_zdot_states(k, zdot_states);
	 	var_manager->get_delta_dot_states(k, delta_dot_states); 		 	
	 	var_manager->get_u_states(k, u_current); 		 	
	 	var_manager->get_var_reaction_forces(k, Fr_states); 		 	
	 	var_manager->get_beta_states(k, beta_states);
	 	var_manager->get_var_knotpoint_dt(k-1, h_dt);

	 	sejong::pretty_print(q_virt_states, std::cout, "q_virt_states");
	 	sejong::pretty_print(z_states, std::cout, "z_states");	 	
	 	sejong::pretty_print(delta_states, std::cout, "delta_states");	
	 	sejong::pretty_print(qdot_virt_states, std::cout, "q_virt_states"); 	
	 	sejong::pretty_print(zdot_states, std::cout, "zdot_states"); 	
	 	sejong::pretty_print(delta_dot_states, std::cout, "delta_dot_states"); 	
	 	sejong::pretty_print(u_current, std::cout, "u_current");	
	 	sejong::pretty_print(Fr_states, std::cout, "Fr_states");	 		 	
	 	sejong::pretty_print(beta_states, std::cout, "beta_states");	 		 	
	 	std::cout << "h_dt = " << h_dt << std::endl;
	 	std::cout << "--------------------------" << std::endl;
	}	
}

int main(int argc, char **argv)
{
	std::cout << "[Main] Running Actuator Dynamics Optimization Problem" << std::endl;
	Optimization_Problem_Main* 	adt_problem = new Jump_Opt();

	//snopt_wrapper::solve_problem_partial_gradients(wbc_problem);	
	snopt_wrapper::solve_problem_no_gradients(adt_problem);

	parse_output(adt_problem);
	

	delete adt_problem;
	return 0;
}
