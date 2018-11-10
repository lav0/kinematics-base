#include "LinearJoint.h"

using namespace kinematics;

kinematics::LinearJoint::LinearJoint(const DHConvention & convention)
  : JointBase(convention)
{
}

std::unique_ptr<IJoint> kinematics::LinearJoint::deepCopy() const
{
  return std::make_unique<LinearJoint>(*this);
}

double & kinematics::LinearJoint::value()
{
  return convention_.link_offset;
}
