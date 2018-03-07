#include <iostream>
#include <Utils/utilities.hpp>

#include <adt/optimization_problems/adt_opt_jump_problem.hpp>
#include <adt/adt_snopt_wrapper.hpp>
#include <adt/containers/adt_opt_variable_manager.hpp>

#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>
#include "DracoP1Rot_Definition.h"

void parse_output(Optimization_Problem_Main* opt_prob){
	ADT_Opt_Variable_Manager* var_manager;
	Draco_Combined_Dynamics_Model* combined_model = Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel();	
	DracoModel* robot_model = DracoModel::GetDracoModel();

	opt_prob->get_var_manager(var_manager);
	std::vector<sejong::Vector> vec_x_states;
	std::vector<sejong::Vector> vec_xdot_states;
	std::vector<sejong::Vector> vec_xddot_states;			

	std::vector<sejong::Vector> vec_u_current;	
	std::vector<sejong::Vector> vec_Fr_states;	
	std::vector<sejong::Vector> vec_beta_states;		
	std::vector<double> vec_h_dt;		

	sejong::Vector q_virt_states;
	sejong::Vector z_states;	
	sejong::Vector delta_states;
	sejong::Vector x_state; x_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);

	sejong::Vector qdot_virt_states;
	sejong::Vector zdot_states;	
	sejong::Vector delta_dot_states;
	sejong::Vector xdot; xdot.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);

	sejong::Vector u_current;	
	sejong::Vector Fr_states;	
	sejong::Vector beta_states;		
	double h_dt;

	sejong::Vector qdot_virt_states_prev;
	sejong::Vector zdot_states_prev;	
	sejong::Vector delta_dot_states_prev;
	sejong::Vector xdot_prev;  xdot_prev.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);
	sejong::Vector xddot_est;

	sejong::Vector q_state;
	sejong::Vector qdot_state;	

	sejong::Vect3 body_pos;
	sejong::Vect3 upperleg_pos;
	sejong::Vect3 foot_toe_pos;
	sejong::Vect3 foot_heel_pos;
	sejong::Vect3 com_pos;
	sejong::Vect3 com_vel;			

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

	 	var_manager->get_qdot_states(k-1, qdot_virt_states_prev); 	
	 	var_manager->get_zdot_states(k-1, zdot_states_prev);
	 	var_manager->get_delta_dot_states(k-1, delta_dot_states_prev); 		 	

	 	x_state.head(NUM_VIRTUAL) = q_virt_states;
	 	x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = z_states;
	 	x_state.tail(NUM_VIRTUAL) = delta_states;

	 	xdot.head(NUM_VIRTUAL) = qdot_virt_states;
	 	xdot.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = zdot_states;
	 	xdot.tail(NUM_VIRTUAL) = delta_dot_states;

	 	xdot_prev.head(NUM_VIRTUAL) = qdot_virt_states_prev;
	 	xdot_prev.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = zdot_states_prev;
	 	xdot_prev.tail(NUM_VIRTUAL) = delta_dot_states_prev;

	 	xddot_est = (xdot - xdot_prev)/h_dt;

	 	combined_model->convert_x_to_q(x_state, q_state);
	 	combined_model->convert_xdot_to_qdot(xdot, qdot_state);	 	

	 	robot_model->UpdateModel(q_state, qdot_state);
	 	robot_model->getCoMPosition(q_state, com_pos);
	 	robot_model->getCoMVelocity(q_state, qdot_state, com_vel);
	 	robot_model->getPosition(q_state, SJLinkID::LK_body, body_pos);
	 	robot_model->getPosition(q_state, SJLinkID::LK_upperLeg, upperleg_pos);	 	
	 	robot_model->getPosition(q_state, SJLinkID::LK_FootToe, foot_toe_pos);
	 	robot_model->getPosition(q_state, SJLinkID::LK_FootHeel, foot_heel_pos);	 


	 		

	 	sejong::pretty_print(x_state, std::cout, "x_state (qvirt, z, delta)");
	 	sejong::pretty_print(xdot, std::cout, "xdot");
	 	sejong::pretty_print(xddot_est, std::cout, "xddot_est");	 		 	

	 	sejong::pretty_print(q_state, std::cout, "q_state");
	 	sejong::pretty_print(qdot_state, std::cout, "qdot_state");


	 	sejong::pretty_print(u_current, std::cout, "u_current");	
	 	sejong::pretty_print(Fr_states, std::cout, "Fr_states");	 		 	
	 	sejong::pretty_print(beta_states, std::cout, "beta_states");	 		 	
	 	std::cout << "h_dt = " << h_dt << std::endl;

	 	std::cout << " " << std::endl;
	 	sejong::pretty_print(com_pos, std::cout, "com_pos (x,y,z)");
	 	sejong::pretty_print(com_vel, std::cout, "com_vel (xdot_com, ydot_com, zdot_com");	 

	 	sejong::pretty_print(body_pos, std::cout, "body_pos (x, z, Ry)");
	 	sejong::pretty_print(upperleg_pos, std::cout, "upperleg_pos (x, z, Ry)");	 	 	
	 	sejong::pretty_print(foot_toe_pos, std::cout, "foot_toe_pos (x, z, Ry)");	 	
	 	sejong::pretty_print(foot_heel_pos, std::cout, "foot_heel_pos (x, z, Ry)");	 	 		

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
