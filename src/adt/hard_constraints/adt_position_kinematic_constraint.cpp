#include <adt/hard_constraints/adt_position_kinematic_constraint.hpp>
#include <Utils/utilities.hpp>

Position_2D_Kinematic_Constraint::Position_2D_Kinematic_Constraint(int knotpoint_in, int link_id_in, int dim_in, double des_val_in){
	des_knotpoint = knotpoint_in;
	link_id = link_id_in;
	dim = dim_in;
	des_val = des_val_in;
	Initialization();
}

Position_2D_Kinematic_Constraint::~Position_2D_Kinematic_Constraint(){
	std::cout << "[Position_2D_Kinematic_Constraint] Destructor called" << std::endl;
}

void Position_2D_Kinematic_Constraint::Initialization(){
	constraint_name = "Position 2D Constraint on link id " + std::to_string(link_id) + " dim " + std::to_string(dim) + " kp " + std::to_string(des_knotpoint);	
	robot_model = DracoModel::GetDracoModel();	
	combined_model = Draco_Combined_Dynamics_Model::GetDracoCombinedDynamicsModel();	
	initialize_Flow_Fupp();	
}


void Position_2D_Kinematic_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

	// Single equality Constraint
	// We want the joint position, q(z(k))  to be at a particular position at timestep k 
	// f(q(z(k)) = des_pos
	F_low.push_back(0.0);	
	F_upp.push_back(0.0);
	constraint_size = F_low.size();
}


void Position_2D_Kinematic_Constraint::evaluate_constraint(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
	F_vec.clear();
}
void Position_2D_Kinematic_Constraint::evaluate_sparse_gradient(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Position_2D_Kinematic_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, ADT_Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

