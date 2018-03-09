#include <adt/hard_constraints/adt_floor_2d_contact_lcp_constraint.hpp>
#include <Utils/utilities.hpp>
#include "DracoP1Rot_Definition.h"
 #include <adt/optimization_constants.hpp>
#include <cmath>

Floor_2D_Contact_LCP_Constraint::Floor_2D_Contact_LCP_Constraint(){
  Initialization();  
}

Floor_2D_Contact_LCP_Constraint::Floor_2D_Contact_LCP_Constraint(Contact_List* contact_list_in, int index_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);
  Initialization();
}

Floor_2D_Contact_LCP_Constraint::~Floor_2D_Contact_LCP_Constraint(){
  std::cout << "[Floor_2D_Contact_LCP_Constraint] Destructor called" << std::endl;  
}

void Floor_2D_Contact_LCP_Constraint::Initialization(){
  std::cout << "[Floor_2D_Contact_LCP_Constraint] Initialization called" << std::endl;
  constraint_name = "2D LCP Constraint";  
  robot_model = DracoModel::GetDracoModel();  
  actuator_model = DracoActuatorModel::GetDracoActuatorModel();
	initialize_Flow_Fupp();	
}

void Floor_2D_Contact_LCP_Constraint::initialize_Flow_Fupp(){
	// Phi(q)*||Fr|| = 0. Try: Phi(q)*||Fr|| <= 0
	// Phi(q) >= 0
  // F_n * (c_k - c_{k-1}) = 0. Try: F_n * fabs(c_k - c_{k-1}) <= 0
  F_low.push_back(0.0); 
  F_low.push_back(0.0);
  F_low.push_back(0.0); 

  F_upp.push_back(1e-3);
  F_upp.push_back(OPT_INFINITY);      
  F_upp.push_back(1e-3);


  // Try gamma_1 = ||Fr||
  F_low.push_back(0.0);
  F_upp.push_back(0.0);



  // F_low.push_back(-OPT_INFINITY); 
  // F_low.push_back(0.0);
  // F_low.push_back(-OPT_INFINITY); 

  // F_upp.push_back(0.0);
  // F_upp.push_back(OPT_INFINITY);      
  // F_upp.push_back(0.0);

  // // also try alpha_1 = phi(q), gamma_1 = ||Fr||, alpha_2 = F_n, gamma_2 = abs(c_k - c_{k-1})
  // for(size_t i = 0; i < num_lcp_vars; i++){
  //    F_low.push_back(0.0); 
  //    F_upp.push_back(0.0);     
  // }
  // double epsilon = 0.0001; // for initial convergence
  // // alpha_1 gamma_1 <= 0
  // F_low.push_back(-OPT_INFINITY); F_upp.push_back(epsilon);     

  // // alpha_2 gamma_2 <= 0
  // F_low.push_back(-OPT_INFINITY); F_upp.push_back(epsilon);       

  constraint_size = F_low.size();
}

void Floor_2D_Contact_LCP_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}
void Floor_2D_Contact_LCP_Constraint::setContact_index(int index_in){
  contact_index = index_in;  
}  

void Floor_2D_Contact_LCP_Constraint::evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
  Contact* current_contact = contact_list_obj->get_contact(contact_index);
  int contact_link_id = current_contact->contact_link_id;

  // Get robot virtual states and actuator z position states---------------------------------------------------------------------------
  sejong::Vector q_state_virt;
  sejong::Vector qdot_state_virt;
  sejong::Vector z_state;  
  sejong::Vector zdot_state;    
  var_manager.get_q_states(knotpoint, q_state_virt);
  var_manager.get_qdot_states(knotpoint, qdot_state_virt);
  var_manager.get_z_states(knotpoint, z_state);
  var_manager.get_zdot_states(knotpoint, zdot_state);  

  // convert actuator z_states to joint configuration q_state_act
  sejong::Vector q_state_act;
  sejong::Vector qdot_state_act;  
  actuator_model->getFull_joint_pos_q(z_state, q_state_act);
  actuator_model->getFull_joint_vel_qdot(z_state, zdot_state, qdot_state_act);

  // Repopulate q_state and qdot_state
  sejong::Vector q_state; q_state.resize(NUM_QDOT);
  sejong::Vector qdot_state; qdot_state.resize(NUM_QDOT);

  q_state.head(NUM_VIRTUAL) = q_state_virt;
  q_state.tail(NUM_ACT_JOINT) = q_state_act;  
  qdot_state.head(NUM_VIRTUAL) = qdot_state_virt;
  qdot_state.head(NUM_ACT_JOINT) = qdot_state_act;   

  // Update the robot model
  robot_model->UpdateModel(q_state, qdot_state);  

  // Get Fr_states--------------------------------------------------------------------------
  sejong::Vector Fr_all;
  var_manager.get_var_reaction_forces(knotpoint, Fr_all);
  int current_contact_size = current_contact->contact_dim;

  // Extract the segment of Reaction Forces corresponding to this contact-------------------
  int index_offset = 0;
  for (size_t i = 0; i < contact_index; i++){
    index_offset += contact_list_obj->get_contact(i)->contact_dim;   
  }
  sejong::Vector Fr_contact = Fr_all.segment(index_offset, current_contact_size);

  // Get normal Force only
  // sejong::Vector Fr_z; 
  // Fr_z.resize(1);
  // Fr_z[0] = Fr_contact[1];

  // double Fr_l2_norm_squared = std::pow(Fr_z.lpNorm<2>(), 2);

  double Fr_l2_norm_squared = std::pow(Fr_contact.lpNorm<2>(), 2);
  sejong::Vect3 contact_pos_vec;
  robot_model->getPosition(q_state, contact_link_id, contact_pos_vec);


  // For now, this is just a ground contact
  double phi_contact_dis = contact_pos_vec[1];
  double complimentary_constraint = phi_contact_dis*Fr_l2_norm_squared;


  // Try Small Substitutions
  sejong::Vector gamma_vars;
  var_manager.get_gamma_states(knotpoint, gamma_vars);    
  double gamma_1 = gamma_vars[contact_index*num_lcps];
  double gamma_1_const = gamma_1 - Fr_l2_norm_squared;
  complimentary_constraint = phi_contact_dis*gamma_1; //use substitition  
  //


  // Add complimentary no slip constraint -----------------
  sejong::Vector q_state_virt_prev;
  sejong::Vector qdot_state_virt_prev;
  sejong::Vector z_state_prev;  
  sejong::Vector zdot_state_prev;    
  var_manager.get_q_states(knotpoint-1, q_state_virt_prev);
  var_manager.get_qdot_states(knotpoint-1, qdot_state_virt_prev);
  var_manager.get_z_states(knotpoint-1, z_state_prev);
  var_manager.get_zdot_states(knotpoint-1, zdot_state_prev);  

  // convert actuator z_states to joint configuration q_state_act
  sejong::Vector q_state_act_prev;
  sejong::Vector qdot_state_act_prev;  
  actuator_model->getFull_joint_pos_q(z_state, q_state_act_prev);
  actuator_model->getFull_joint_vel_qdot(z_state, zdot_state, qdot_state_act_prev);

  // Repopulate q_state_prev and qdot_state_prev
  sejong::Vector q_state_prev; q_state_prev.resize(NUM_QDOT);
  sejong::Vector qdot_state_prev; qdot_state_prev.resize(NUM_QDOT);

  q_state_prev.head(NUM_VIRTUAL) = q_state_virt_prev;
  q_state_prev.tail(NUM_ACT_JOINT) = q_state_act_prev;  
  qdot_state_prev.head(NUM_VIRTUAL) = qdot_state_virt_prev;
  qdot_state_prev.head(NUM_ACT_JOINT) = qdot_state_act_prev;   

  robot_model->UpdateModel(q_state_prev, qdot_state_prev);  
  sejong::Vect3 contact_pos_vec_prev;
  robot_model->getPosition(q_state_prev, contact_link_id, contact_pos_vec_prev);  

  double F_z = Fr_contact[1];
  double F_no_slip = F_z*std::fabs(contact_pos_vec[0] - contact_pos_vec_prev[0]);
  //std::cout << "F_no_slip = " << F_no_slip << std::endl;


  // alpha, gamma try
  // sejong::Vector alpha_vars;
  // sejong::Vector gamma_vars;
  // var_manager.get_alpha_states(knotpoint, alpha_vars);  
  // var_manager.get_gamma_states(knotpoint, gamma_vars);    

  // double alpha_1 = alpha_vars[contact_index*num_lcps];
  // double alpha_2 = alpha_vars[contact_index*num_lcps + 1];
  // double gamma_1 = gamma_vars[contact_index*num_lcps];
  // double gamma_2 = gamma_vars[contact_index*num_lcps + 1];

  // double alpha_1_const = alpha_1 - phi_contact_dis;
  // double gamma_1_const = gamma_1 - Fr_l2_norm_squared;
  // double alpha_2_const = alpha_2 - F_z;
  // double gamma_2_const = gamma_2 - std::fabs(contact_pos_vec[0] - contact_pos_vec_prev[0]);

  // F_vec.push_back(alpha_1_const);
  // F_vec.push_back(gamma_1_const);  
  // F_vec.push_back(alpha_2_const);
  // F_vec.push_back(gamma_2_const);  

  // F_vec.push_back(alpha_1*gamma_1);
  // F_vec.push_back(alpha_2*gamma_2);
  // ------------


  F_vec.push_back(complimentary_constraint); // Phi >= 0
  F_vec.push_back(phi_contact_dis);    
  F_vec.push_back(F_no_slip);

  F_vec.push_back(gamma_1_const);


}

void Floor_2D_Contact_LCP_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
}

void Floor_2D_Contact_LCP_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
}
