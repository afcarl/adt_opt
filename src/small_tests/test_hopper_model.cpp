#include "HopperModel.hpp"
#include "Hopper_Definition.h"

#include <Utils/utilities.hpp>

int main(int argc, char **argv){
	std::cout << "[Main] Testing Hopper Model Object" << std::endl;
	HopperModel* robot_model = HopperModel::GetRobotModel(); 

	// Test Dynamic Quantities
	sejong::Matrix A_mat;
	sejong::Matrix Ainv_mat;
	sejong::Vector grav_vec;
	sejong::Vector coriolis_vec;
	robot_model->getMassInertia(A_mat);
	robot_model->getInverseMassInertia(Ainv_mat);	
	robot_model->getGravity(grav_vec);		
	robot_model->getCoriolis(coriolis_vec);

	sejong::pretty_print(A_mat, std::cout, "A_mat");
	sejong::pretty_print(Ainv_mat, std::cout, "Ainv_mat");
	sejong::pretty_print(grav_vec, std::cout, "grav_vec");
	sejong::pretty_print(coriolis_vec, std::cout, "coriolis_vec");

	// Test Kinematic Quantities
	// Initialize Generalized Coordinates
	sejong::Vector q;
	q = sejong::Vector::Zero(2);
	q[0] = 1.0;
	q[1] = -1.0;

	// Test Positions
	sejong::Vector body_pos;
	sejong::Vector leg_pos;	
	sejong::Vector foot_pos;	

	robot_model->getPosition(q, SJ_Hopper_LinkID::LK_body, body_pos);
	sejong::pretty_print(body_pos, std::cout, "body_pos");

	robot_model->getPosition(q, SJ_Hopper_LinkID::LK_leg, leg_pos);
	sejong::pretty_print(leg_pos, std::cout, "leg_pos");	

	robot_model->getPosition(q, SJ_Hopper_LinkID::LK_foot, foot_pos);
	sejong::pretty_print(foot_pos, std::cout, "foot_pos");		

	// Test Jacobians
	sejong::Matrix J_body;
	sejong::Matrix J_leg;
	sejong::Matrix J_foot;

	robot_model->getFullJacobian(q, SJ_Hopper_LinkID::LK_body, J_body);
	sejong::pretty_print(J_body, std::cout, "J_body");

	robot_model->getFullJacobian(q, SJ_Hopper_LinkID::LK_leg, J_leg);
	sejong::pretty_print(J_leg, std::cout, "J_leg");	

	robot_model->getFullJacobian(q, SJ_Hopper_LinkID::LK_foot, J_foot);
	sejong::pretty_print(J_foot, std::cout, "J_foot");		

	return 0;
}