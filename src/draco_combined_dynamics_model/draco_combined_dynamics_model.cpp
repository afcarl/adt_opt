#include <draco_combined_dynamics_model/draco_combined_dynamics_model.hpp>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

#include "DracoP1Rot_Definition.h"


Draco_Combined_Dynamics_Model* Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel(){
    static Draco_Combined_Dynamics_Model draco_combined_dynamics_model_;
    return & draco_combined_dynamics_model_;
}

Draco_Combined_Dynamics_Model::~Draco_Combined_Dynamics_Model(){
	std::cout << "[Draco_Combined_Dynamics_Model] Destroyed" << std::endl;
}

Draco_Combined_Dynamics_Model::Draco_Combined_Dynamics_Model(){
	Initialization();
	std::cout << "[Draco_Combined_Dynamics_Model] Constructed" << std::endl;
}



void Draco_Combined_Dynamics_Model::Initialization(){
	robot_model = DracoModel::GetDracoModel();
	actuator_model = DracoActuatorModel::GetDracoActuatorModel();	

	// Initialize Inertia Matrix Size
	A_mat.resize(NUM_QDOT, NUM_QDOT); A_mat.setZero();
	
	A_bb.resize(NUM_VIRTUAL, NUM_VIRTUAL); A_bb.setZero();
	A_br.resize(NUM_VIRTUAL, NUM_ACT_JOINT); A_br.setZero();
	A_brT.resize(NUM_ACT_JOINT, NUM_VIRTUAL); A_brT.setZero();
	A_rr.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); A_rr.setZero();

	// Initialize Gravity and Coriolis Vectors
	grav.resize(NUM_QDOT);	grav.setZero();
	coriolis.resize(NUM_QDOT); coriolis.setZero();

	// Initialize Actuator to Joint Dynamics Jacobians
	L.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); L.setZero();
	J.resize(NUM_ACT_JOINT, NUM_ACT_JOINT);	J.setZero();

	// Initialize Selection Matrices
	Sv.resize(NUM_VIRTUAL, NUM_QDOT); Sv.setZero();
	Sa.resize(NUM_ACT_JOINT, NUM_QDOT);	Sa.setZero();

	Sv.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL); 
	Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT); 	


	// ---------------------------------------------------------------
	M_act.resize(NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR, NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR); M_act.setZero();
	initialize_actuator_matrices(M_zz);
	initialize_actuator_matrices(M_z_delta);
	initialize_actuator_matrices(M_delta_z);
	initialize_actuator_matrices(M_delta_delta);		

	B_act.resize(NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR, NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR); B_act.setZero();
	initialize_actuator_matrices(B_zz);
	initialize_actuator_matrices(B_z_delta);
	initialize_actuator_matrices(B_delta_z);
	initialize_actuator_matrices(B_delta_delta);		

	K_act.resize(NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR, NUM_ACT_JOINT * NUM_STATES_PER_ACTUATOR); K_act.setZero();
	initialize_actuator_matrices(K_zz);
	initialize_actuator_matrices(K_z_delta);
	initialize_actuator_matrices(K_delta_z);
	initialize_actuator_matrices(K_delta_delta);

	initialize_actuator_matrices(Km_act);			
	// ---------------------------------------------------------------


	// Initialize Combined Matrices
	M_combined.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); M_combined.setZero();
	M_combined_inv.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); M_combined_inv.setZero();

	B_combined.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); B_combined.setZero();
	K_combined.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); K_combined.setZero();

	// Initialize x, xdot vectors
	x_state.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); x_state.setZero();
	xdot_state.resize(NUM_QDOT + NUM_ACT_JOINT + NUM_ACT_JOINT); xdot_state.setZero();

	// Initialize z, zdot vectors
	z_state.resize(NUM_ACT_JOINT); z_state.setZero();
	zdot_state.resize(NUM_ACT_JOINT); zdot_state.setZero();	

	// Initialize q_virt, qdot_virt vectors
	q_virt_state.resize(NUM_VIRTUAL); q_virt_state.setZero();
	qdot_virt_state.resize(NUM_VIRTUAL); qdot_virt_state.setZero();	
	q_act.resize(NUM_ACT_JOINT); q_act.setZero();
	qdot_act.resize(NUM_ACT_JOINT); qdot_act.setZero();

/*	delta_state.resize(NUM_ACT_JOINT); delta_state.setZero();
	delta_dot_state.resize(NUM_ACT_JOINT); delta_dot_state.setZero();*/

	// Initialize q, qdot vectors
	q_state.resize(NUM_QDOT); q_state.setZero();
	qdot_state.resize(NUM_QDOT); qdot_state.setZero();


	total_imp.resize(NUM_VIRTUAL + NUM_ACT_JOINT); total_imp.setZero();
	virt_imp.resize(NUM_VIRTUAL); virt_imp.setZero();
	current_input.resize(NUM_ACT_JOINT); current_input.setZero();
	joint_imp.resize(NUM_ACT_JOINT); joint_imp.setZero();

}

void Draco_Combined_Dynamics_Model::initialize_actuator_matrices(sejong::Matrix &Mat){
	Mat.resize(NUM_ACT_JOINT, NUM_ACT_JOINT); Mat.setZero();
}

void Draco_Combined_Dynamics_Model::UpdateModel(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in){
	x_state = x_state_in;
	xdot_state = xdot_state_in;

	// Convert x to q
	convert_x_xdot_to_q_qdot(x_state, xdot_state, q_state, qdot_state);

	// Update the Robot's Model
	robot_model->UpdateModel(q_state, qdot_state);
	robot_model->getMassInertia(A_mat);
    robot_model->getGravity(grav);
    robot_model->getCoriolis(coriolis);

	actuator_model->getFullJacobian_dzdq(z_state, L);
	actuator_model->getFullJacobian_dqdz(q_state.tail(NUM_ACT_JOINT), J);	   

	actuator_model->getMassMatrix(M_act);
	actuator_model->getDampingMatrix(B_act);	   
	actuator_model->getStiffnessMatrix(K_act);	   
    actuator_model->getKm_Matrix(Km_act);

/*	sejong::pretty_print(L, std::cout, "L");
	sejong::pretty_print(J, std::cout, "J");

	sejong::pretty_print(M_act, std::cout, "M_act");
	sejong::pretty_print(B_act, std::cout, "B_act");
	sejong::pretty_print(K_act, std::cout, "K_act");*/

	formulate_mass_matrix();
	formulate_damping_matrix();
	formulate_stiffness_matrix();
}


void Draco_Combined_Dynamics_Model::get_combined_mass_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &M_out){}
void Draco_Combined_Dynamics_Model::get_combined_damping_matrix(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Matrix &B_out){}
void Draco_Combined_Dynamics_Model::get_combined_stiffness_matrix(const sejong::Vector &x_state, sejong::Matrix &K_out){}
void Draco_Combined_Dynamics_Model::get_virtual_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sv_out){}
void Draco_Combined_Dynamics_Model::get_actuated_joints_impedance(const sejong::Vector &x_state, const sejong::Vector &xdot_state, const sejong::Vector &Fr_states, sejong::Vector &sa_out){}			

void Draco_Combined_Dynamics_Model::formulate_mass_matrix(){
	A_bb = A_mat.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL);
	A_br = A_mat.block(0, NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT);
	A_brT = A_mat.block(NUM_VIRTUAL, 0, NUM_ACT_JOINT, NUM_VIRTUAL);
	A_rr = A_mat.block(NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT);

	M_zz = M_act.block(0, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	M_z_delta = M_act.block(0, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);
	M_delta_z = M_act.block(NUM_ACT_JOINT, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	M_delta_delta = M_act.block(NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);	

	M_combined.block(0, 0, NUM_VIRTUAL, NUM_VIRTUAL) = A_bb;
	M_combined.block(0, NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT) = A_br*J;
	M_combined.block(NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = M_zz;
	M_combined.block(NUM_VIRTUAL, NUM_VIRTUAL + NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT) = M_z_delta;	
	M_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, 0, NUM_ACT_JOINT, NUM_VIRTUAL) = A_brT;
	M_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_VIRTUAL) = A_rr*J + L.transpose()*M_delta_z;	
	M_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT) = L.transpose()*M_delta_delta;	

//	sejong::pretty_print(M_combined, std::cout, "M_combined");
}

void Draco_Combined_Dynamics_Model::formulate_damping_matrix(){
	B_zz = B_act.block(0, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	B_z_delta = B_act.block(0, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);
	B_delta_z = B_act.block(NUM_ACT_JOINT, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	B_delta_delta = B_act.block(NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);	

	B_combined.block(NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = B_zz;
	B_combined.block(NUM_VIRTUAL, NUM_VIRTUAL + NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT) = B_z_delta;	
	B_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_VIRTUAL) = L.transpose()*B_delta_z;
	B_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT) = L.transpose()*B_delta_delta;	

//	sejong::pretty_print(B_combined, std::cout, "B_combined");
}	

void Draco_Combined_Dynamics_Model::formulate_stiffness_matrix(){
	K_zz = K_act.block(0, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	K_z_delta = K_act.block(0, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);
	K_delta_z = K_act.block(NUM_ACT_JOINT, 0, NUM_ACT_JOINT, NUM_ACT_JOINT);
	K_delta_delta = K_act.block(NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT);	

	K_combined.block(NUM_VIRTUAL, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = K_zz;
	K_combined.block(NUM_VIRTUAL, NUM_VIRTUAL + NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT) = K_z_delta;	
	K_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_VIRTUAL) = L.transpose()*K_delta_z;
	K_combined.block(NUM_VIRTUAL + NUM_ACT_JOINT, NUM_VIRTUAL + NUM_ACT_JOINT, NUM_ACT_JOINT, NUM_ACT_JOINT) = L.transpose()*K_delta_delta;	

//	sejong::pretty_print(K_act, std::cout, "K_act");
//	sejong::pretty_print(K_combined, std::cout, "K_combined");
}
void Draco_Combined_Dynamics_Model::formulate_joint_link_impedance(const::sejong::Vector &Fr_state_in){
	total_imp = -coriolis - grav + Jc.transpose()*Fr_state_in;
	virt_imp = Sv*(total_imp);
	joint_imp = Sa*(total_imp);
}

void Draco_Combined_Dynamics_Model::convert_x_xdot_to_q_qdot(const sejong::Vector &x_state, const sejong::Vector &xdot_state, sejong::Vector &q_state_out, sejong::Vector &qdot_state_out){
/*	// extract q_virt and actuator z_states from x_states
	q_virt_state = x_state.head(NUM_VIRTUAL);
	z_state = x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

	qdot_virt_state = xdot_state.head(NUM_VIRTUAL);
	zdot_state = xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

	// Convert z, zdot to q, qdot
	actuator_model->getFull_joint_pos_q(z_state, q_act);
	actuator_model->getFull_joint_vel_qdot(zdot_state, zdot_state, qdot_act);

	q_state_out.resize(NUM_QDOT);
	qdot_state_out.resize(NUM_QDOT);

	q_state_out.head(NUM_VIRTUAL) = q_virt_state;
	q_state_out.tail(NUM_ACT_JOINT) = q_act;

	qdot_state_out.head(NUM_VIRTUAL) = qdot_virt_state;	
	qdot_state_out.tail(NUM_ACT_JOINT) = qdot_act;	*/

	convert_x_to_q(x_state, q_state_out);
	convert_xdot_to_qdot(xdot_state, qdot_state_out);		

/*	sejong::pretty_print(q_state_out, std::cout, "q_state_out");
	sejong::pretty_print(qdot_state_out, std::cout, "qdot_state_out");	*/
}

void Draco_Combined_Dynamics_Model::convert_x_to_q(const sejong::Vector &x_state, sejong::Vector &q_state_out){
	q_virt_state = x_state.head(NUM_VIRTUAL);
	z_state = x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

	// Convert z, to q
	actuator_model->getFull_joint_pos_q(z_state, q_act);

	q_state_out.resize(NUM_QDOT);
	q_state_out.head(NUM_VIRTUAL) = q_virt_state;
	q_state_out.tail(NUM_ACT_JOINT) = q_act;
}

void Draco_Combined_Dynamics_Model::convert_xdot_to_qdot(const sejong::Vector &xdot_state, sejong::Vector &qdot_state_out){
	qdot_virt_state = xdot_state.head(NUM_VIRTUAL);
	zdot_state = xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);

	// Convert zdot to qdot
	actuator_model->getFull_joint_vel_qdot(zdot_state, zdot_state, qdot_act);

	qdot_state_out.resize(NUM_QDOT);
	qdot_state_out.head(NUM_VIRTUAL) = qdot_virt_state;	
	qdot_state_out.tail(NUM_ACT_JOINT) = qdot_act;	
}

void Draco_Combined_Dynamics_Model::setContactJacobian(sejong::Matrix &Jc_in){
	Jc = Jc_in;
	//sejong::pretty_print(Jc, std::cout, "Jc_in");
}

void Draco_Combined_Dynamics_Model::get_state_acceleration(const sejong::Vector &x_state_in, const::sejong::Vector &xdot_state_in, 
							    						   const sejong::Vector &u_current_in, const::sejong::Vector &Fr_state_in,
							    						   sejong::Vector &xddot_state_out){
	UpdateModel(x_state_in, xdot_state_in);
    sejong::pseudoInverse(M_combined, 1.e-10, M_combined_inv, 0);
    formulate_joint_link_impedance(Fr_state_in);

    sejong::Vector total_input; total_input.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); total_input.setZero();
    total_input.head(NUM_VIRTUAL) = virt_imp - A_br*J*xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
    total_input.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = Km_act*u_current_in;
    total_input.tail(NUM_ACT_JOINT) = joint_imp;

    // sejong::Matrix A_mat_inv;
    // sejong::pseudoInverse(A_mat, 1.e-10, A_mat_inv, 0);
    // sejong::Vector tau_test; tau_test.resize(NUM_QDOT); tau_test.setZero();
    // tau_test[3] = -157;

    // sejong::Vector test_regular_qddot = A_mat_inv*(-coriolis - grav + tau_test);
    // sejong::pretty_print(test_regular_qddot, std::cout, "test_regular_qddot");

    // xddot_state_out = M_combined_inv*(total_input - B_combined*xdot_state_in - K_combined*x_state_in);
    // sejong::pretty_print(K_combined, std::cout, "K_combined");
    // sejong::pretty_print(xdot_state_in, std::cout, "xdot_state_in");
    // sejong::pretty_print(x_state_in, std::cout, "x_state_in");    

    // sejong::pretty_print(u_current_in, std::cout, "u_current_in");
    // sejong::pretty_print(total_input, std::cout, "total_input");

    xddot_state_out = M_combined_inv*(total_input - B_combined*xdot_state_in - K_combined*x_state_in);

}

void Draco_Combined_Dynamics_Model::getDynamics_constraint(const sejong::Vector &x_state_in, const sejong::Vector &xdot_state_in, const sejong::Vector &xddot_state_in,
						  							       const sejong::Vector &u_current_in, const sejong::Vector &Fr_state_in, sejong::Vector &dynamics_out){

	UpdateModel(x_state_in, xdot_state_in);
    formulate_joint_link_impedance(Fr_state_in);

    sejong::Vector total_input; total_input.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); total_input.setZero();
    total_input.head(NUM_VIRTUAL) = virt_imp - A_br*J*xdot_state_in.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
    total_input.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = Km_act*u_current_in;
    total_input.tail(NUM_ACT_JOINT) = joint_imp;


    // sejong::Vector virt_forces =  A_br*J*xdot_state_in.segment(NUM_VIRTUAL, NUM_ACT_JOINT)+ Sv*(coriolis + grav - Jc.transpose()*Fr_state_in);
    // sejong::Vector act_forces =  -Km_act*u_current_in;
    // sejong::Vector joint_forces = Sa*(coriolis + grav - Jc.transpose()*Fr_state_in);

    // dynamics_out.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); dynamics_out.setZero();
    // dynamics_out.head(NUM_VIRTUAL) = virt_forces;
    // dynamics_out.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = act_forces;
    //  dynamics_out.tail(NUM_ACT_JOINT) = joint_forces;


	//dynamics_out = M_combined*xddot_state_in + B_combined*xdot_state_in +K_combined*x_state_in - total_input;
    dynamics_out = M_combined*xddot_state_in + B_combined*xdot_state_in +K_combined*x_state_in - total_input;    
//    dynamics_out += M_combined*xddot_state_in + B_combined*xdot_state_in + K_combined*x_state_in;

    sejong::pretty_print(grav, std::cout, "grav");

    // sejong::pretty_print(xddot_state_in, std::cout, "xddot_state_in");
    // sejong::pretty_print(virt_forces, std::cout, "virt_forces");
    // sejong::pretty_print(total_input, std::cout, "total_input");    
    sejong::pretty_print(Fr_state_in, std::cout, "Fr_state_in");
    // sejong::pretty_print(u_current_in, std::cout, "u_current_in");
    // sejong::pretty_print(dynamics_out, std::cout, "dynamics_out");

   // sejong::Vector total_input; total_input.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); total_input.setZero();
   // total_input.head(NUM_VIRTUAL) = virt_imp - A_br*J*xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
   // total_input.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = Km_act*u_current_in;
   // total_input.tail(NUM_ACT_JOINT) = joint_imp;	

   //  dynamics_out.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); dynamics_out.setZero();
   //  dynamics_out.head(NUM_VIRTUAL) = Sv*(coriolis + grav - Jc.transpose()*Fr_state_in);
   //  sejong::Vector virtual_only = total_input.head(NUM_VIRTUAL);
   //  sejong::Vector test_vec = Sv*(coriolis + grav - Jc.transpose()*Fr_state_in);



//    sejong::pretty_print(dynamics_out, std::cout, "dynamics_out");

    
}

void Draco_Combined_Dynamics_Model::getDynamics_constraint(const sejong::Vector &x_state_k, const sejong::Vector &xdot_state_k, const sejong::Vector &xdot_state_k_prev, 
                                                           const sejong::Vector &u_current_k, const sejong::Vector &Fr_state_k, double h_k, sejong::Vector &dynamics_out){
    // Update the model and impedances
    UpdateModel(x_state_k, xdot_state_k);
    formulate_joint_link_impedance(Fr_state_k);
    // Construct the dynamics input and impedances
    sejong::Vector total_input; total_input.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT); total_input.setZero();
    total_input.head(NUM_VIRTUAL) = virt_imp - A_br*J*xdot_state_k.segment(NUM_VIRTUAL, NUM_ACT_JOINT);
    total_input.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = Km_act*u_current_k;
    total_input.tail(NUM_ACT_JOINT) = joint_imp  ;  

    dynamics_out = M_combined*(xdot_state_k - xdot_state_k_prev) + h_k*(B_combined*xdot_state_k + K_combined*x_state_k - total_input);    

    sejong::Vector xddot_est = (xdot_state_k - xdot_state_k_prev) / h_k;
    //sejong::pretty_print(xddot_est, std::cout, "xddot_est");
    //sejong::pretty_print(Fr_state_k, std::cout, "Fr_state_k");

}