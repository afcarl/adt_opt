#ifndef HOPPER_MODEL
#define HOPPER_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>
#include "Hopper_Definition.h"

using namespace sejong;

class HopperModel{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static HopperModel* GetRobotModel();
    virtual ~HopperModel(void);

    bool getMassInertia(sejong::Matrix & A);
    bool getInverseMassInertia(sejong::Matrix & Ainv) ;
    bool getGravity(Vector & grav) ;
    bool getCoriolis(Vector & coriolis) ;

    // virtual void getCentroidJacobian(sejong::Matrix & Jcent);
    // virtual void getCentroidInertia(sejong::Matrix & Icent);
    void getPosition(const Vector & q,
                           int link_id, sejong::Vector & pos) ;
    void getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J);
    // virtual void getCoMPosition(const Vector & q, Vect3 & com_pos, bool update= false);
    // virtual void getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel);

    //void getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & J) const ;
    // void getVelocity(const Vector & q, const Vector &qdot,
    //                  int link_id, Vect3 & vel) ;

    // void getCentroidVelocity(sejong::Vector & centroid_vel);
    // void getCoMJacobian(const Vector & q, sejong::Matrix & J);
    // void UpdateKinematics(const Vector & q, const Vector &qdot);
    // void UpdateModel(const sejong::Vector & q, const sejong::Vector & qdot);

protected:
    double m_base;
    double m_leg;
    double grav_const = 9.81; // m/s^2

    sejong::Matrix A_int;

private:
    HopperModel();
};

#endif
