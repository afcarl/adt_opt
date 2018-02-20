#ifndef ADT_DRACO_CONTACT_TOE_H
#define ADT_DRACO_CONTACT_TOE_H

#include <adt/contacts/adt_contact_main.hpp>
#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"

class Draco_Toe_Contact: public Contact{
public:
	Draco_Toe_Contact();
	~Draco_Toe_Contact();	
	DracoModel* robot_model;

	void getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getContactJacobianDotQdot(const sejong::Vector &q_state, 
  								   const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};

#endif