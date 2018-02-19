#ifndef DRACO_P1_ACTUATOR_MODEL
#define DRACO_P1_ACTUATOR_MODEL

#include <Utils/wrap_eigen.hpp>

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

    // Mass
    double M_motor;
    double M_spring;
    double M_load;

    // Damping
    double B_motor;
    double B_spring;
    double B_load;

    // Spring Elements
    double K_motor;
    double K_spring;
    
    // Torque Constant (N-m / Amps) 
    double K_m;

private:
    void Initialization();
    DracoActuatorModel();
};

#endif
