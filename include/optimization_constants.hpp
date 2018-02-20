#ifndef ADT_OPT_CONSTANTS_H
#define ADT_OPT_CONSTANTS_H
  #define OPT_INFINITY 1.0e20
  #define OPT_ZERO_EPS 1.0e-4


  #define OPT_ZERO_GRADIENT_EPS 1.0e-8

  #define OPT_TIMESTEP 0.01

  #define VAR_TYPE_NONE -1 // No variable type
  #define VAR_TYPE_Q 0	   // Q State Variable Type
  #define VAR_TYPE_QDOT 1  // Qdot State Variable Type 
  #define VAR_TYPE_TA 2    // Task Acceleration Variable Type
  #define VAR_TYPE_FR 3    // Reaction Force Variable Type
  #define VAR_TYPE_KF 4    // Key Frame Variable Type
  #define VAR_TYPE_H 5    // Knot Point Time Step Variable Type

  #define VAR_TYPE_Z 6    // Actuator z-position variable type
  #define VAR_TYPE_DEL 7    // Actuator spring delta variable type
  #define VAR_TYPE_U 8    // Actuator current input variable type

#endif 