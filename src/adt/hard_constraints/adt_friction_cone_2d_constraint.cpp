#include <adt/hard_constraints/adt_friction_cone_2d_constraint.hpp>
#include <Utils/utilities.hpp>

Friction_Cone_2D_Constraint::Friction_Cone_2D_Constraint(){
  Initialization();  
}

Friction_Cone_2D_Constraint::Friction_Cone_2D_Constraint(Contact_List* contact_list_in, int &index_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);
  Initialization();
}

void Friction_Cone_2D_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}
void Friction_Cone_2D_Constraint::setContact_index(int index_in){
  contact_index = index_in;  
}  

Friction_Cone_2D_Constraint::~Friction_Cone_2D_Constraint(){
  std::cout << "[Friction_Cone_2D_Constraint] Destructor called" << std::endl;  
}


void Friction_Cone_2D_Constraint::Initialization(){
	constraint_name = "Friction Cone 2D Constraint";  

	n1.resize(2); d1.resize(2); n1.setZero(); d1.setZero();
	n2.resize(2); d2.resize(2); n2.setZero(); d2.setZero();

	n1[1] =  1.0; n2[1] = 1.0; // Normal Vectors point up;
	// Tangential Vectors d1 = [-1, 0] d2 ] [1,0]	
	d1[0] = -1;    d2[0] = 1;

	w1 = n1 + mu*d1;
	w2 = n2 + mu*d2;

	initialize_Flow_Fupp();	
}

void Friction_Cone_2D_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();
	// Fr_j = sum_ij beta_ij wij
	// 2D so only Fx and Fz forces are present
	F_low.push_back(0.0);	
	F_low.push_back(0.0);

	F_upp.push_back(0.0);
	F_upp.push_back(0.0);			
	constraint_size = F_low.size();
}


void Friction_Cone_2D_Constraint::evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
  Contact* current_contact = contact_list_obj->get_contact(contact_index);
  int contact_link_id = current_contact->contact_link_id;

  // Get Fr_states and beta states
  sejong::Vector Fr_all;
  sejong::Vector beta_all;
  var_manager.get_var_reaction_forces(knotpoint, Fr_all);
  var_manager.get_beta_states(knotpoint, beta_all);  

  // Extract the segment of Reaction Forces corresponding to this contact
  int index_offset = 0;
  for (size_t i = 0; i < contact_index; i++){
    index_offset += contact_list_obj->get_contact(i)->contact_dim;   
  }

  int current_contact_size = current_contact->contact_dim;
  //std::cout << "[Friction_Cone_2D_Constraint]: (contact_index, index_offset, contact_dim) = (" << contact_index << "," << index_offset << "," << current_contact_size << ")"  << std::endl;
  sejong::Vector Fr_contact = Fr_all.segment(index_offset, current_contact_size);

  int beta_index = contact_index*Nd;
  sejong::Vector betas_contact = beta_all.segment(beta_index, Nd);
  // std::cout << "[Friction_Cone_2D_Constraint]: beta_all.size() = " << beta_all.size()  << std::endl;
  // std::cout << "[Friction_Cone_2D_Constraint]: betas_contact.size() = " << betas_contact.size()  << std::endl;  

  double beta_1j = betas_contact[0];
  double beta_2j = betas_contact[1];  

  sejong::Vector const_eval = Fr_contact - (beta_1j*w1 + beta_2j*w2);

  //sejong::pretty_print(const_eval, std::cout, "Fr_j - sum_i beta w_ij");

  for (size_t i = 0; i < const_eval.size(); i++){
    F_vec.push_back(const_eval[i]);
  }


}




void Friction_Cone_2D_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Friction_Cone_2D_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}	
