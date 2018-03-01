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
	// Phi(q)*||Fr|| = 0
	// Phi(q) >= 0
	F_low.push_back(0.00);	
	F_low.push_back(0.00);

	F_upp.push_back(0.001);
	F_upp.push_back(OPT_INFINITY);			

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
  sejong::Vector Fr_z; 
  Fr_z.resize(1);
  Fr_z[0] = Fr_contact[1];

  double Fr_l2_norm_squared = std::pow(Fr_z.lpNorm<2>(), 2);

  sejong::Vect3 contact_pos_vec;
  robot_model->getPosition(q_state, contact_link_id, contact_pos_vec);

  // Set contact to be rectangular surface. 
    // Calculate distance to surface. 
    // ...

  // For now, this is just a ground contact

  // contact_pos_vec = [x, z, Ry]

  //std::cout << "Fr_z = " << Fr_z[0] << std::endl;
  //std::cout << "Fr_l2_norm_squared = " << Fr_l2_norm_squared  << std::endl;
  double phi_contact_dis = contact_pos_vec[1];
  double complimentary_constraint = phi_contact_dis*Fr_l2_norm_squared;

  // std::cout << "phi_contact_dis = " << phi_contact_dis  << std::endl;
  // std::cout << "complimentary_constraint = " << complimentary_constraint << std::endl;

  F_vec.push_back(complimentary_constraint); // Phi >= 0
  F_vec.push_back(phi_contact_dis);          // 

}

void Floor_2D_Contact_LCP_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
}

void Floor_2D_Contact_LCP_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
}
