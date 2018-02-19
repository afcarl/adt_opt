#ifndef DRACO_P1_ACTUATOR_MODEL
#define DRACO_P1_ACTUATOR_MODEL

#include <Utils/wrap_eigen.hpp>

class DracoActuatorModel{
public:
    static DracoActuatorModel* GetDracoActuatorModel();
    virtual ~DracoActuatorModel(void);


    void getMassMatrix(sejong::Matrix &M_act);
    void getDampingMatrix(sejong::Matrix &B_act);
    void getStiffnessMatrix(sejong::Matrix &K_act);

private:
    DracoActuatorModel();
};

#endif
