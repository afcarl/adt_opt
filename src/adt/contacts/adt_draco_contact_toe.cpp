#include <adt/contacts/adt_draco_contact_toe.hpp>
#include <iostream>

// Define LeftFoot Contact ---------------------------------------------------------------
Draco_Toe_Contact::Draco_Toe_Contact(){
    std::cout << "[Draco Toe Contact] Constructed" << std::endl;
	robot_model = DracoModel::GetDracoModel();
	contact_dim = 6;
    contact_name = "Draco Toe Contact";
    contact_link_id = SJLinkID::LK_FootToe;
    std::cout << "[Draco Toe Contact] Link id: " << contact_link_id << std::endl;
}
Draco_Toe_Contact::~Draco_Toe_Contact(){}

void Draco_Toe_Contact::getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jtmp;
    robot_model->getFullJacobian(q_state, contact_link_id, Jtmp);
    Jt = Jtmp; //Jtmp.block(3, 0, 3, NUM_QDOT);
}

void Draco_Toe_Contact::getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_tmp;    
    robot_model->getFullJacobianDot(q_state, qdot_state, contact_link_id, Jdot_tmp);
    sejong::Matrix Jdot_task = Jdot_tmp;//Jdot_tmp.block(3, 0, 3, NUM_QDOT);

	JtDotQdot = Jdot_task*qdot_state;    
}
