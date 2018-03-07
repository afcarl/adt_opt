#ifndef ADT_OPT_CONSTANTS_H
#define ADT_OPT_CONSTANTS_H
  #define OPT_INFINITY 1.0e20
  #define OPT_ZERO_EPS 1.0e-4


  #define OPT_ZERO_GRADIENT_EPS 1.0e-8

  #define OPT_TIMESTEP 0.01

  // These values are fixed
  #define VAR_TYPE_NONE -1 // No variable type
  #define VAR_TYPE_Q 0	   // Q State Variable Type
  #define VAR_TYPE_QDOT 1  // Qdot State Variable Type 
  #define VAR_TYPE_TA 2    // Task Acceleration Variable Type
  #define VAR_TYPE_FR 3    // Reaction Force Variable Type
  #define VAR_TYPE_KF 4    // Key Frame Variable Type
  #define VAR_TYPE_H 5    // Knot Point Time Step Variable Type
  #define VAR_TYPE_Z 6    // Actuator z-position variable type
  #define VAR_TYPE_ZDOT 7    // Actuator z-velocity variable type
  #define VAR_TYPE_DELTA 8    // Actuator spring delta variable type
  #define VAR_TYPE_DELTA_DOT 9    // Actuator spring delta velocity variable type 
  #define VAR_TYPE_U 10    // Actuator current input variable type
  #define VAR_TYPE_BETA 11 // Beta basis vectors for the friction cone constraint
  #define VAR_TYPE_QDDOT_VIRT 12 // virtual joint acceleration
  #define VAR_TYPE_XDDOT_ALL 13 // virtual virtual, and actuator accelerations

  #define VAR_TYPE_ALPHA 14 // alpha_lcp_constraint
  #define VAR_TYPE_GAMMA 15 // beta_lcp_constraint

  #define ND_2D_CONST 2 // Number of friction cone basis vectors for a 2D plane. This number is fixed.
  #define CALC_F_MODE 0 // Decides if we are computing F
  #define CALC_G_MODE 1 // Decides if we are computing the gradient of F


#endif 