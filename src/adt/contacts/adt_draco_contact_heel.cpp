#include <adt/contacts/adt_draco_contact_heel.hpp>
#include <iostream>

// Define LeftFoot Contact ---------------------------------------------------------------
Draco_Heel_Contact::Draco_Heel_Contact(){
    std::cout << "[Draco Heel Contact] Constructed" << std::endl;
	robot_model = DracoModel::GetDracoModel();
	contact_dim = 2;
    contact_name = "Draco Heel Contact";
    contact_link_id = SJLinkID::LK_FootHeel;
    std::cout << "[Draco Heel Contact] Contact Name: " << contact_name << ", Link id: " << contact_link_id << std::endl;
}
Draco_Heel_Contact::~Draco_Heel_Contact(){}

void Draco_Heel_Contact::getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jtmp;
    robot_model->getFullJacobian(q_state, contact_link_id, Jtmp);
    //Jt = Jtmp; //Jtmp.block(3, 0, 3, NUM_QDOT);
    Jt = Jtmp.block(0, 0, 2, NUM_QDOT);    
}

void Draco_Heel_Contact::getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_tmp;    
    robot_model->getFullJacobianDot(q_state, qdot_state, contact_link_id, Jdot_tmp);
    //sejong::Matrix Jdot_task = Jdot_tmp;//Jdot_tmp.block(3, 0, 3, NUM_QDOT);
    sejong::Matrix Jdot_task = Jdot_tmp.block(0, 0, 2, NUM_QDOT);    

	JtDotQdot = Jdot_task*qdot_state;    
}
