#ifndef WBT_OPT_VARS_LIST_H
#define WBT_OPT_VARS_LIST_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <map>
#include <adt/containers/adt_opt_variable.hpp>

class ADT_Opt_Variable_Manager{
public:
	ADT_Opt_Variable_Manager();
	~ADT_Opt_Variable_Manager();	

	void append_variable(ADT_Opt_Variable* opt_variable);
	void compute_size_time_dep_vars(); // This function needs to have been called at least once if a getter function is used.


	ADT_Opt_Variable* get_opt_variable(const int index);

	void get_var_states(const int &knotpoint, sejong::Vector &q_state, sejong::Vector &qdot_state);	
	void get_task_accelerations(const int &knotpoint, sejong::Vector &xddot);		
	void get_var_reaction_forces(const int &knotpoint, sejong::Vector &Fr_state);		
	void get_var_keyframes(const int &knotpoint, sejong::Vector &keyframe_state);		
	void get_var_knotpoint_dt(const int &knotpoint, double &h_dt);


	void get_q_states(const int &knotpoint, sejong::Vector &q_state);		
	void get_qdot_states(const int &knotpoint, sejong::Vector &qdot_state);			

	void get_z_states(const int &knotpoint, sejong::Vector &z_state);		
	void get_zdot_states(const int &knotpoint, sejong::Vector &zdot_state);			

	void get_delta_states(const int &knotpoint, sejong::Vector &delta_state);		
	void get_delta_dot_states(const int &knotpoint, sejong::Vector &delta_dot_state);		


	void get_u_states(const int &knotpoint, sejong::Vector &u_state);		
	void get_beta_states(const int &knotpoint, sejong::Vector &beta_state);				


	int get_size();
	int get_size_timedependent_vars();

	void update_x(std::vector<double> &x_in);	 // To do
	void populate_x(std::vector<double> &x_out); // To do


	int initial_conditions_offset = 0;
	int total_knotpoints;

	int get_num_q_vars();
	int get_num_qdot_vars();
	int get_num_xddot_vars();
	int get_num_Fr_vars();
	int get_num_keyframe_vars();		

	int get_num_var_knotpoint_dt();
private:
	void add_variable_to_map(std::map<int, std::vector<ADT_Opt_Variable*> > &map_kp_to_var_vec, ADT_Opt_Variable* opt_variable);

	void convert_to_vector(const int &knotpoint, 
						   std::map<int, std::vector<ADT_Opt_Variable*> > &map_kp_to_var_vec,
						   sejong::Vector &vec_out);

	std::vector<ADT_Opt_Variable*> opt_var_list;

	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_q_state_vars;
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_qdot_state_vars;

	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_xddot_vars;
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_Fr_vars;
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_keyframe_vars;

	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_z_vars;
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_zdot_vars;
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_delta_vars;
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_delta_dot_vars;

	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_u_vars;	
	std::map<int, std::vector<ADT_Opt_Variable*> > knotpoint_to_beta_vars;


	std::vector<ADT_Opt_Variable*> knotpoint_to_dt;
	

	int count_num_vars_in_map(const int &knotpoint, std::map<int, std::vector<ADT_Opt_Variable*> > &map_kp_to_var_vec);
	//std::vector< int, std::vector<ADT_Opt_Variable*> > 
	int num_timedep_vars = 0; 

	int num_q_vars = 0;
	int num_qdot_vars = 0;
	int num_xddot_vars = 0;
	int num_Fr_vars = 0;
	int num_keyframe_vars = 0;

	int num_knotpoint_dt_vars = 0;	

	int num_z_vars = 0;
	int num_zdot_vars = 0; 
	int num_delta_vars = 0;
	int num_delta_dot_vars = 0;
	int num_u_vars = 0;
	int num_beta_vars = 0;
	int num_h = 0;

};

#endif