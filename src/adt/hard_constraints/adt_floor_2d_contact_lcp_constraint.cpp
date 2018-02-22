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
  robot_model = DracoModel::GetDracoModel();  
	initialize_Flow_Fupp();	
}

void Floor_2D_Contact_LCP_Constraint::initialize_Flow_Fupp(){
	// Phi(q)*||Fr|| = 0
	// Phi(q) >= 0
	F_low.push_back(0.0);	
	F_low.push_back(0.0);

	F_upp.push_back(0.0);
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

  // Get q_states---------------------------------------------------------------------------
  sejong::Vector q_state;
  sejong::Vector qdot_state;  
  var_manager.get_var_states(knotpoint, q_state, qdot_state);

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

  double phi_contact_dis = contact_pos_vec[2];
  double complimentary_constraint = phi_contact_dis*Fr_l2_norm_squared;

  F_vec.push_back(complimentary_constraint); // Phi >= 0
  F_vec.push_back(phi_contact_dis);          // 

}

void Floor_2D_Contact_LCP_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
}

void Floor_2D_Contact_LCP_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
}
