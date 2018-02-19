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

    double get_joint_pos_q(int &index, double &z_act_pos);
    double get_act_pos_z(int &index, double &q_act_pos); 
    double getJacobian_dzdq(int &index, double &z_act_pos);   

    // double
    std::vector<double> r_arm; // Moment arm

    // Mass
    std::vector<double> M_motor;
    std::vector<double> M_spring;
    std::vector<double> M_load;

    // Damping
    std::vector<double> B_motor;
    std::vector<double> B_spring;
    std::vector<double> B_load;

    // Spring Elements
    std::vector<double> K_motor;
    std::vector<double> K_spring;
    
     
    std::vector<double> K_m; // Torque Constant (N-m / Amps)
    std::vector<double> z_o; // initial actuator position for which spring force is 0.

private:
    void Initialization();
    DracoActuatorModel();
};

#endif
