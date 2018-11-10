#include "RotaryJoint.h"

using namespace kinematics;

RotaryJoint::RotaryJoint(const DHConvention & convention)
  : JointBase(convention)
{
}

std::unique_ptr<IJoint> kinematics::RotaryJoint::deepCopy() const
{
  return std::make_unique<RotaryJoint>(*this);
}

double & kinematics::RotaryJoint::value()
{
  return convention_.joint_angle;
}
