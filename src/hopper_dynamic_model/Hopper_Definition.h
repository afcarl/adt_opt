#ifndef HOPPER_DEFINITION
#define HOPPER_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>

enum SJ_Hopper_LinkID{
  LK_body = 0,
  LK_leg,
  LK_foot,
  NUM_LINK
};

enum SJ_Hopper_JointID{
  VIRTUAL_Z = 0,
  leg_extend,
  NUM_JOINT
};

enum SJ_Hopper_ActuatorID{
  act_leg_extend = 0,
};

#define NUM_QDOT 2
#define NUM_VIRTUAL 1
#define NUM_Q NUM_QDOT
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
