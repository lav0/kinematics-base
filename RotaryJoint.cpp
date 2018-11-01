#include "RotaryJoint.h"

using namespace kinematics;

RotaryJoint::RotaryJoint(const DHConvention & convention)
  : JointBase(convention)
{
}

double & kinematics::RotaryJoint::value()
{
  return convention_.joint_angle;
}
