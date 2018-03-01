#include <adt/hard_constraints/adt_dynamics_constraint.hpp>
#include "DracoP1Rot_Definition.h"
#include <Utils/utilities.hpp>

Dynamics_Constraint::Dynamics_Constraint(){
	Initialization();
}

Dynamics_Constraint::Dynamics_Constraint(Contact_List* contact_list_in){
	Initialization();
	setContact_List(contact_list_in);
}

Dynamics_Constraint::~Dynamics_Constraint(){
		std::cout << "[Dynamics_Constraint] Destructor called" << std::endl;
}

void Dynamics_Constraint::Initialization(){
	constraint_name = "BDynamics_Constraint";	

	robot_model = DracoModel::GetDracoModel();	
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();
	combined_model = Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel();	
	initialize_Flow_Fupp();	

}

void Dynamics_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}

void Dynamics_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

  // Mxddot + Bxdot + Kx - impedance = 0
	for(size_t i = 0; i < (NUM_VIRTUAL + NUM_ACT_JOINT*NUM_STATES_PER_ACTUATOR); i++){
		F_low.push_back(0.0);	
		F_upp.push_back(0.0);
	}

  //Mxddot + Bxdot + Kx - impedance = 0
  // for(size_t i = 0; i < (NUM_VIRTUAL); i++){
  //   F_low.push_back(0.0); 
  //   F_upp.push_back(0.0);
  // }

	constraint_size = F_low.size();
}

void Dynamics_Constraint::Update_Contact_Jacobian_Jc(sejong::Vector &x_state){
	sejong::Vector q_states;
	combined_model->convert_x_to_q(x_state, q_states);

  // Stack the contact Jacobians
  // Extract the segment of Reaction Forces corresponding to this contact-------------------
  sejong::Matrix J_tmp;
  int prev_row_size = 0;
  for (size_t i = 0; i < contact_list_obj->get_size(); i++){
    // Get the Jacobian for the current contact
    contact_list_obj->get_contact(i)->getContactJacobian(q_states, J_tmp);   
    Jc.conservativeResize(prev_row_size + J_tmp.rows(), NUM_QDOT);
   	Jc.block(prev_row_size, 0, J_tmp.rows(), NUM_QDOT) = J_tmp;
   	prev_row_size += J_tmp.rows();
  }
  //sejong::pretty_print(Jc, std::cout, "Contact Jacobian Jc");
  combined_model->setContactJacobian(Jc);

}

void Dynamics_Constraint::get_states(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, sejong::Vector &x_state, sejong::Vector &xdot_state){
  var_manager.get_q_states(knotpoint, q_state_virt);
  var_manager.get_z_states(knotpoint, z_state);
  var_manager.get_delta_states(knotpoint, delta_state);
  var_manager.get_qdot_states(knotpoint, qdot_state_virt);
  var_manager.get_zdot_states(knotpoint, zdot_state);  
  var_manager.get_delta_dot_states(knotpoint, delta_dot_state);

  x_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);
  xdot_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);  

  x_state.head(NUM_VIRTUAL) = q_state_virt;
  x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = z_state;
  x_state.tail(NUM_ACT_JOINT) = delta_state;

  xdot_state.head(NUM_VIRTUAL) = qdot_state_virt;
  xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = zdot_state;
  xdot_state.tail(NUM_ACT_JOINT) = delta_dot_state;	   
}



void Dynamics_Constraint::evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
//   F_vec.clear();

//   sejong::Vector x_state_k; 
//   sejong::Vector xdot_state_k; 
//   sejong::Vector xddot_state_k;
//   sejong::Vector u_state_k;
//   sejong::Vector Fr_state_k;
//   sejong::Vector dynamics_k;

//   get_states(knotpoint, var_manager, x_state_k, xdot_state_k);
//   var_manager.get_u_states(knotpoint, u_state_k);
//   var_manager.get_var_reaction_forces(knotpoint, Fr_state_k);  
//   var_manager.get_xddot_all_states(knotpoint, xddot_state_k);

//   // Update the Jacobians and the model
//   // This order is important!
//   combined_model->UpdateModel(x_state_k, xdot_state_k);
//   Update_Contact_Jacobian_Jc(x_state_k); 
//   // ------

//   combined_model->getDynamics_constraint(x_state_k, xdot_state_k, xddot_state_k, u_state_k, Fr_state_k, dynamics_k);

// //  for(size_t i = 0; i < dynamics_k.size(); i++){
//   for(size_t i = 0; i < dynamics_k.size(); i++){
//     F_vec.push_back(dynamics_k[i]);    
//   }

  F_vec.clear();

  sejong::Vector x_state_k; 
  sejong::Vector x_state_k_prev;   
  sejong::Vector xdot_state_k; 
  sejong::Vector xdot_state_k_prev; 

  double h_k;
  var_manager.get_var_knotpoint_dt(knotpoint - 1, h_k);
  get_states(knotpoint, var_manager, x_state_k, xdot_state_k);
  get_states(knotpoint-1, var_manager, x_state_k_prev, xdot_state_k_prev);  

  sejong::Vector u_state_k;
  sejong::Vector Fr_state_k;
  sejong::Vector dynamics_k; 
  var_manager.get_u_states(knotpoint, u_state_k);
  var_manager.get_var_reaction_forces(knotpoint, Fr_state_k);  

  // Update the Jacobians and the model
  // This order is important!
  combined_model->UpdateModel(x_state_k, xdot_state_k);
  Update_Contact_Jacobian_Jc(x_state_k); 
  // ------

  combined_model->getDynamics_constraint(x_state_k, xdot_state_k, xdot_state_k_prev, u_state_k, Fr_state_k, h_k, dynamics_k);

//  for(size_t i = 0; i < dynamics_k.size(); i++){
  for(size_t i = 0; i < dynamics_k.size(); i++){
    F_vec.push_back(dynamics_k[i]);    
  }





}

void Dynamics_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Dynamics_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}


