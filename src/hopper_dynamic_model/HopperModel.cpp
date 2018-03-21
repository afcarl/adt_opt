#include "HopperModel.hpp"

#include <Utils/utilities.hpp>
#include <stdio.h>

HopperModel* HopperModel::GetRobotModel(){
    static HopperModel hopper_model;
    return & hopper_model;
}

HopperModel::HopperModel(){
	std::cout << "[HopperModel] Hopper Object Created" << std::endl;
	m_base = 5; //kg
	m_leg = 1; //kg	


    A_int = sejong::Matrix::Zero(NUM_QDOT, NUM_QDOT);
    A_int(0,0) = m_base;
    A_int(0,1) = m_leg;
    A_int(1,0) = m_leg;    
    A_int(1,1) = m_leg;
}

HopperModel::~HopperModel(){
	std::cout << "[HopperModel] Hopper Object Destroyed" << std::endl;
}

bool HopperModel::getMassInertia(sejong::Matrix & A){
	A = A_int;
}

bool HopperModel::getInverseMassInertia(sejong::Matrix & Ainv){
	Ainv = sejong::Matrix::Zero(NUM_QDOT, NUM_QDOT);
	Ainv(0,0) = m_leg;
	Ainv(0,1) = -m_leg;	
	Ainv(1,0) = -m_leg;
	Ainv(1,1) = m_base;	

	double det = (m_base*m_leg - m_leg*m_leg);
	Ainv *= (1.0/det);
}


bool HopperModel::getGravity(Vector & grav){
	grav = sejong::Vector::Zero(NUM_QDOT);
	grav[0] = (m_base + m_leg)*grav_const;
	grav[1] = (m_leg)*grav_const;	
}

bool HopperModel::getCoriolis(Vector & coriolis){
	coriolis = sejong::Vector::Zero(NUM_QDOT);
}

void HopperModel::getPosition(const Vector & q, int link_id, sejong::Vector & pos){
	pos = sejong::Vector::Zero(1);
	if (link_id == SJ_Hopper_LinkID::LK_body){
		// std::cout << "[HopperModel] Body Position" << std::endl;
		pos[0] = q[0];
	}else if(link_id == SJ_Hopper_LinkID::LK_leg){
		// std::cout << "[HopperModel] Leg Position" << std::endl;
		pos[0] = q[0] + q[1]/2.0;		
	}else if(link_id == SJ_Hopper_LinkID::LK_foot){
		// std::cout << "[HopperModel] Foot Position" << std::endl;
		pos[0] = q[0] + q[1];		
	}
}

void HopperModel::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J){
	J = sejong::Matrix::Zero(1, NUM_QDOT);
	if (link_id == SJ_Hopper_LinkID::LK_body){
		// std::cout << "[HopperModel] Body Position" << std::endl;
		J(0, 0) = 1.0;
	}else if(link_id == SJ_Hopper_LinkID::LK_leg){
		// std::cout << "[HopperModel] Leg Position" << std::endl;
		J(0, 0) = 1.0;
		J(0, 1) = 1.0/2.0;
	}else if(link_id == SJ_Hopper_LinkID::LK_foot){
		// std::cout << "[HopperModel] Foot Position" << std::endl;
		J(0, 0) = 1.0;
		J(0, 1) = 1.0;
	}	

}