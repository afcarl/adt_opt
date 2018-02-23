#ifndef DRACO_P1_ACTUATOR_MODEL
#define DRACO_P1_ACTUATOR_MODEL

#include <Utils/wrap_eigen.hpp>
#include <vector>

#define NUM_ACTUATORS 3
#define NUM_STATES_PER_ACTUATOR 2
#define NUM_TOTAL_STATES NUM_ACTUATORS*NUM_STATES_PER_ACTUATOR

// Draco has 3 actuators with
// States are z = Actuator linear position and 
//            delta = spring position

class DracoActuatorModel{
public:
    static DracoActuatorModel* GetDracoActuatorModel();
    ~DracoActuatorModel(void);

    void getMassMatrix(sejong::Matrix &M_act);
    void getDampingMatrix(sejong::Matrix &B_act);
    void getStiffnessMatrix(sejong::Matrix &K_act);
    void getKm_Matrix(sejong::Matrix &Km_act);


    double get_joint_pos_q(const int &index, const double &z_act_pos);
    double get_joint_vel_qdot(const int &index, const double z_act_pos, const double &z_act_vel);

    double get_act_pos_z(const int &index, const double &q_act_pos); 
    double getJacobian_dqdz(const int &index, const double &q_act_pos);
    double getJacobian_dzdq(const int &index, const double &z_act_pos);

    void getFullJacobian_dzdq(const sejong::Vector &z_pos, sejong::Matrix &L);   
    void getFullJacobian_dqdz(const sejong::Vector &q_pos, sejong::Matrix &J);

    void getFull_joint_pos_q(const sejong::Vector &z_in, sejong::Vector &q_out);
    void getFull_joint_vel_qdot(const sejong::Vector &z_in, const sejong::Vector &zdot_in, sejong::Vector &qdot_out);

    void getFull_act_pos_z(const sejong::Vector &q_in, sejong::Vector &z_out);


    void set_zero_pos_q_o(sejong::Vector &q_o_in);
    // double
    sejong::Vector r_arm; // Moment arm

    // Mass
    sejong::Vector M_motor;
    sejong::Vector M_spring;
    sejong::Vector M_load;

    // Damping
    sejong::Vector B_motor;
    sejong::Vector B_spring;
    sejong::Vector B_load;

    // Spring Elements
    sejong::Vector K_motor;
    sejong::Vector K_spring;
        
    sejong::Vector K_m; // Torque Constant (N-m / Amps)
    sejong::Vector z_o; // initial actuator position for which spring force is 0.
    sejong::Vector q_o; // initial joint position for which the spring force is 0.    


    sejong::Vector q_l_bound;
    sejong::Vector q_u_bound;

    sejong::Vector z_l_bound;
    sejong::Vector z_u_bound;    

private:
    void Initialization();
    DracoActuatorModel();
};

#endif
